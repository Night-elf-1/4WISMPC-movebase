#include "mpc_local_planner/nmpc_core/diffmpc.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

// ===================== 工具函数：保留与线性版本一致的接口 =====================

std::vector<double> diffMpcController::calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed) {
    std::vector<double> referenceSpeeds;
    referenceSpeeds.reserve(curvatures.size()); 
    for (double k : curvatures) {
        // 用倒数曲线根据曲率限速
        double speed = max_speed / (1.0 + 3.0 * std::fabs(k));
        // speed 必然 <= max_speed，因此只需限制下限
        speed = std::max(0.05, speed); 
        referenceSpeeds.push_back(speed);
    }
    return referenceSpeeds;

}

void diffMpcController::smooth_yaw(std::vector<double>& cyaw) {
    for (int i = 0; i < (int)cyaw.size() - 1; i++) {
        double dyaw = cyaw[i + 1] - cyaw[i];
        while (dyaw > M_PI / 2.0) {
            cyaw[i + 1] -= M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
        while (dyaw < -M_PI / 2.0) {
            cyaw[i + 1] += M_PI * 2.0;
            dyaw = cyaw[i + 1] - cyaw[i];
        }
    }
}

static std::tuple<int, double> calc_nearest_index_diff(double current_x, double current_y,
                                                         const std::vector<double>& cx,
                                                         const std::vector<double>& cy) {
    const size_t point_count = std::min(cx.size(), cy.size());
    if (point_count == 0) {
        return std::make_tuple(0, std::numeric_limits<double>::max());
    }

    double min_squared_distance = std::numeric_limits<double>::max();
    int ind = 0;
    for (size_t i = 0; i < point_count; ++i) {
        double idx = current_x - cx[i];
        double idy = current_y - cy[i];
        double squared_distance = idx * idx + idy * idy;
        if (squared_distance < min_squared_distance) {
            min_squared_distance = squared_distance;
            ind = static_cast<int>(i);
        }
    }
    return std::make_tuple(ind, std::sqrt(min_squared_distance));
}

std::tuple<int, double> diffMpcController::calc_ref_trajectory(double current_x, double current_y,
                                                                 const std::vector<double>& cx,
                                                                 const std::vector<double>& cy) const {
    return calc_nearest_index_diff(current_x, current_y, cx, cy);
}

// 按路径真实累计弧长插值预测参考，不依赖全局规划器的点间距。
M_XREF_DIFF diffMpcController::build_horizon_reference(int min_index,
                                                        const std::vector<double>& cx,
                                                        const std::vector<double>& cy,
                                                        const std::vector<double>& cyaw,
                                                        const std::vector<double>& speed,
                                                        double current_v,
                                                        int& target_ind) {
    M_XREF_DIFF xref = M_XREF_DIFF::Zero();
    const size_t ncourse = std::min({cx.size(), cy.size(), cyaw.size(), speed.size()});
    if (ncourse == 0) {
        target_ind = 0;
        return xref;
    }

    int ind = std::clamp(std::max(min_index, target_ind), 0,
                         static_cast<int>(ncourse) - 1);
    int segment_index = ind;
    double segment_start_distance = 0.0;

    auto sample_at_distance = [&](double target_distance, int column) {
        constexpr double kMinSegmentLength = 1e-9;
        while (segment_index + 1 < static_cast<int>(ncourse)) {
            const double dx = cx[segment_index + 1] - cx[segment_index];
            const double dy = cy[segment_index + 1] - cy[segment_index];
            const double segment_length = std::hypot(dx, dy);

            if (segment_length <= kMinSegmentLength) {
                ++segment_index;
                continue;
            }

            if (segment_start_distance + segment_length >= target_distance) {
                const double alpha = std::clamp(
                    (target_distance - segment_start_distance) / segment_length, 0.0, 1.0);
                const double yaw_delta = std::atan2(
                    std::sin(cyaw[segment_index + 1] - cyaw[segment_index]),
                    std::cos(cyaw[segment_index + 1] - cyaw[segment_index]));
                xref(0, column) = cx[segment_index] + alpha * dx;
                xref(1, column) = cy[segment_index] + alpha * dy;
                xref(2, column) = cyaw[segment_index] + alpha * yaw_delta;
                xref(3, column) = speed[segment_index] +
                                  alpha * (speed[segment_index + 1] - speed[segment_index]);
                return;
            }

            segment_start_distance += segment_length;
            ++segment_index;
        }

        const size_t last = ncourse - 1;
        xref(0, column) = cx[last];
        xref(1, column) = cy[last];
        xref(2, column) = cyaw[last];
        xref(3, column) = speed[last];
    };

    double travel = 0.0;
    for (int i = 0; i < NMPC_T; ++i) {
        if (i > 0) {
            const double step_speed = (i == 1) ? std::abs(current_v)
                                                : std::abs(xref(3, i - 1));
            travel += step_speed * NMPC_DT;
        }
        sample_at_distance(travel, i);
    }

    target_ind = ind;
    return xref;
}

// ===================== NMPC 代价 / 约束函数 =====================

FG_EVAL_DIFF::FG_EVAL_DIFF(const M_XREF_DIFF &trajRef, const Eigen::VectorXd &uPrev, double L_, double W_)
    : traj_ref(trajRef), U_prev(uPrev), L(L_), W(W_) {}

void FG_EVAL_DIFF::operator()(FG_EVAL_DIFF::ADvector &fg, const FG_EVAL_DIFF::ADvector &vars) {
    fg[0] = 0;

    // 四个轮子在车体坐标系下的位置 (FL, RL, RR, FR)
    double xw[4] = {L, -L, -L, L};
    double yw[4] = {W, W, -W, -W};
    double denom = 4.0 * (L * L + W * W);

    const double w_pos = 1.0;     // 位置跟踪权重       1.0
    const double w_yaw = 2.0;     // 航向跟踪权重   0.5
    const double w_v   = 2.0;     // 速度跟踪权重   2.0
    const double w_u   = 0.01;    // 控制量大小权重
    const double w_du  = 0.5;     // 控制量变化率权重   0.5

    // ---- 控制量代价（大小 + 相邻步变化率 + 速度跟踪） ----
    for (int i = 0; i < NMPC_T - 1; i++) {
        AD<double> v1 = vars[v1_start_ + i];
        AD<double> v2 = vars[v2_start_ + i];
        AD<double> v3 = vars[v3_start_ + i];
        AD<double> v4 = vars[v4_start_ + i];
        AD<double> d1 = vars[d1_start_ + i];
        AD<double> d2 = vars[d2_start_ + i];
        AD<double> d3 = vars[d3_start_ + i];
        AD<double> d4 = vars[d4_start_ + i];

        fg[0] += w_u * (CppAD::pow(v1, 2) + CppAD::pow(v2, 2) + CppAD::pow(v3, 2) + CppAD::pow(v4, 2));
        fg[0] += w_u * (CppAD::pow(d1, 2) + CppAD::pow(d2, 2) + CppAD::pow(d3, 2) + CppAD::pow(d4, 2));

        AD<double> v_avg = 0.25 * (v1 + v2 + v3 + v4);
        fg[0] += w_v * CppAD::pow(traj_ref(3, i + 1) - v_avg, 2);

        AD<double> pv1 = (i == 0) ? AD<double>(U_prev(0)) : vars[v1_start_ + i - 1];
        AD<double> pv2 = (i == 0) ? AD<double>(U_prev(1)) : vars[v2_start_ + i - 1];
        AD<double> pv3 = (i == 0) ? AD<double>(U_prev(2)) : vars[v3_start_ + i - 1];
        AD<double> pv4 = (i == 0) ? AD<double>(U_prev(3)) : vars[v4_start_ + i - 1];
        AD<double> pd1 = (i == 0) ? AD<double>(U_prev(4)) : vars[d1_start_ + i - 1];
        AD<double> pd2 = (i == 0) ? AD<double>(U_prev(5)) : vars[d2_start_ + i - 1];
        AD<double> pd3 = (i == 0) ? AD<double>(U_prev(6)) : vars[d3_start_ + i - 1];
        AD<double> pd4 = (i == 0) ? AD<double>(U_prev(7)) : vars[d4_start_ + i - 1];

        fg[0] += w_du * (CppAD::pow(v1 - pv1, 2) + CppAD::pow(v2 - pv2, 2) +
                          CppAD::pow(v3 - pv3, 2) + CppAD::pow(v4 - pv4, 2));
        fg[0] += w_du * (CppAD::pow(d1 - pd1, 2) + CppAD::pow(d2 - pd2, 2) +
                          CppAD::pow(d3 - pd3, 2) + CppAD::pow(d4 - pd4, 2));
    }

    // ---- 初始状态约束 ----
    fg[1 + x_start_]   = vars[x_start_];
    fg[1 + y_start_]   = vars[y_start_];
    fg[1 + yaw_start_] = vars[yaw_start_];

    // ---- 动力学约束 + 轨迹跟踪代价 ----
    for (int i = 0; i < NMPC_T - 1; i++) {
        AD<double> x0   = vars[x_start_ + i];
        AD<double> y0   = vars[y_start_ + i];
        AD<double> yaw0 = vars[yaw_start_ + i];

        AD<double> x1   = vars[x_start_ + i + 1];
        AD<double> y1   = vars[y_start_ + i + 1];
        AD<double> yaw1 = vars[yaw_start_ + i + 1];

        AD<double> v1 = vars[v1_start_ + i];
        AD<double> v2 = vars[v2_start_ + i];
        AD<double> v3 = vars[v3_start_ + i];
        AD<double> v4 = vars[v4_start_ + i];
        AD<double> d1 = vars[d1_start_ + i];
        AD<double> d2 = vars[d2_start_ + i];
        AD<double> d3 = vars[d3_start_ + i];
        AD<double> d4 = vars[d4_start_ + i];

        AD<double> dx = 0.25 * (v1 * CppAD::cos(d1 + yaw0) + v2 * CppAD::cos(d2 + yaw0) +
                                 v3 * CppAD::cos(d3 + yaw0) + v4 * CppAD::cos(d4 + yaw0));
        AD<double> dy = 0.25 * (v1 * CppAD::sin(d1 + yaw0) + v2 * CppAD::sin(d2 + yaw0) +
                                 v3 * CppAD::sin(d3 + yaw0) + v4 * CppAD::sin(d4 + yaw0));

        AD<double> dtheta = ((-yw[0] * CppAD::cos(d1) + xw[0] * CppAD::sin(d1)) * v1 +
                              (-yw[1] * CppAD::cos(d2) + xw[1] * CppAD::sin(d2)) * v2 +
                              (-yw[2] * CppAD::cos(d3) + xw[2] * CppAD::sin(d3)) * v3 +
                              (-yw[3] * CppAD::cos(d4) + xw[3] * CppAD::sin(d4)) * v4) / denom;

        AD<double> x_pred   = x0 + dx * NMPC_DT;
        AD<double> y_pred   = y0 + dy * NMPC_DT;
        AD<double> yaw_pred = yaw0 + dtheta * NMPC_DT;

        fg[2 + x_start_ + i]   = x1 - x_pred;
        fg[2 + y_start_ + i]   = y1 - y_pred;
        fg[2 + yaw_start_ + i] = yaw1 - yaw_pred;

        fg[0] += w_pos * CppAD::pow(traj_ref(0, i + 1) - x_pred, 2);
        fg[0] += w_pos * CppAD::pow(traj_ref(1, i + 1) - y_pred, 2);
        AD<double> dyaw = yaw_pred - traj_ref(2, i + 1);
        fg[0] += w_yaw * CppAD::pow(CppAD::atan2(CppAD::sin(dyaw), CppAD::cos(dyaw)), 2);
    }

    // ---- 转向角/轮速变化率硬约束 ----
    int idx = 1 + 3 * NMPC_T; 
    for (int i = 0; i < NMPC_T - 1; i++) {
        fg[idx++] = vars[d1_start_ + i] - ((i == 0) ? AD<double>(U_prev(4)) : vars[d1_start_ + i - 1]);
        fg[idx++] = vars[d2_start_ + i] - ((i == 0) ? AD<double>(U_prev(5)) : vars[d2_start_ + i - 1]);
        fg[idx++] = vars[d3_start_ + i] - ((i == 0) ? AD<double>(U_prev(6)) : vars[d3_start_ + i - 1]);
        fg[idx++] = vars[d4_start_ + i] - ((i == 0) ? AD<double>(U_prev(7)) : vars[d4_start_ + i - 1]);
    }
    for (int i = 0; i < NMPC_T - 1; i++) {
        fg[idx++] = vars[v1_start_ + i] - ((i == 0) ? AD<double>(U_prev(0)) : vars[v1_start_ + i - 1]);
        fg[idx++] = vars[v2_start_ + i] - ((i == 0) ? AD<double>(U_prev(1)) : vars[v2_start_ + i - 1]);
        fg[idx++] = vars[v3_start_ + i] - ((i == 0) ? AD<double>(U_prev(2)) : vars[v3_start_ + i - 1]);
        fg[idx++] = vars[v4_start_ + i] - ((i == 0) ? AD<double>(U_prev(3)) : vars[v4_start_ + i - 1]);
    }

    // ---- 新增：刚体几何一致性等式约束 ----
    // 约束每一个控制步 (共 T-1 步) 的 4WIS 组合必须符合刚体瞬心相交原理
    for (int i = 0; i < NMPC_T - 1; i++) {
        AD<double> v1 = vars[v1_start_ + i]; // FL
        AD<double> v2 = vars[v2_start_ + i]; // RL
        AD<double> v3 = vars[v3_start_ + i]; // RR
        AD<double> v4 = vars[v4_start_ + i]; // FR
        AD<double> d1 = vars[d1_start_ + i];
        AD<double> d2 = vars[d2_start_ + i];
        AD<double> d3 = vars[d3_start_ + i];
        AD<double> d4 = vars[d4_start_ + i];

        // 1. 左侧前后轮纵向速度分量相等
        fg[idx++] = v1 * CppAD::cos(d1) - v2 * CppAD::cos(d2);
        // 2. 右侧前后轮纵向速度分量相等
        fg[idx++] = v3 * CppAD::cos(d3) - v4 * CppAD::cos(d4);
        // 3. 前侧左右轮横向速度分量相等
        fg[idx++] = v1 * CppAD::sin(d1) - v4 * CppAD::sin(d4);
        // 4. 后侧左右轮横向速度分量相等
        fg[idx++] = v2 * CppAD::sin(d2) - v3 * CppAD::sin(d3);
        // 5. 刚体旋转相容性约束（阿克曼瞬心约束）
        fg[idx++] = W * (v1 * CppAD::sin(d1) - v2 * CppAD::sin(d2)) - L * (v4 * CppAD::cos(d4) - v1 * CppAD::cos(d1));
    }
}

Eigen::VectorXd diffMpcController::mpc_solve(const std::vector<double>& cx,
                                              const std::vector<double>& cy,
                                              const std::vector<double>& cyaw,
                                              const std::vector<double>& speed,
                                              const Eigen::Vector3d& initial_x,
                                              int min_index,
                                              double reference_speed,
                                              const parameters& params_,
                                              const Eigen::VectorXd& applied_control) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x   = initial_x(0);
    double y   = initial_x(1);
    double yaw = initial_x(2);

    if (target_ind_state < min_index) target_ind_state = min_index;
    M_XREF_DIFF traj_ref = build_horizon_reference(min_index, cx, cy, cyaw, speed,
                                                    reference_speed, target_ind_state);

    size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
    
    // 初始/动力学 + 每一控制步的变化率 + 每一控制步的刚体几何约束。
    size_t n_constraints = NMPC_T * 3 + 8 * (NMPC_T - 1) + 5 * (NMPC_T - 1);

    Eigen::VectorXd previous_control = Eigen::VectorXd::Zero(NMPC_NU);
    if (applied_control.size() == NMPC_NU && applied_control.allFinite()) {
        previous_control = applied_control;
    }

    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) vars[i] = 0.0;

    vars[x_start_]   = x;
    vars[y_start_]   = y;
    vars[yaw_start_] = yaw;

    for (int i = 0; i < NMPC_T - 1; i++) {
        vars[v1_start_ + i] = std::clamp(U(0), -params_.control_limits.max_wheel_speed, params_.control_limits.max_wheel_speed);
        vars[v2_start_ + i] = std::clamp(U(1), -params_.control_limits.max_wheel_speed, params_.control_limits.max_wheel_speed);
        vars[v3_start_ + i] = std::clamp(U(2), -params_.control_limits.max_wheel_speed, params_.control_limits.max_wheel_speed);
        vars[v4_start_ + i] = std::clamp(U(3), -params_.control_limits.max_wheel_speed, params_.control_limits.max_wheel_speed);
        vars[d1_start_ + i] = std::clamp(U(4), -params_.control_limits.max_steering_angle, params_.control_limits.max_steering_angle);
        vars[d2_start_ + i] = std::clamp(U(5), -params_.control_limits.max_steering_angle, params_.control_limits.max_steering_angle);
        vars[d3_start_ + i] = std::clamp(U(6), -params_.control_limits.max_steering_angle, params_.control_limits.max_steering_angle);
        vars[d4_start_ + i] = std::clamp(U(7), -params_.control_limits.max_steering_angle, params_.control_limits.max_steering_angle);
    }

    {
        double xw[4] = {params_.L, -params_.L, -params_.L, params_.L};
        double yw[4] = {params_.W, params_.W, -params_.W, -params_.W};
        double denom = 4.0 * (params_.L * params_.L + params_.W * params_.W);
        double sx = x, sy = y, syaw = yaw;
        for (int i = 0; i < NMPC_T - 1; i++) {
            double v1 = vars[v1_start_ + i], v2 = vars[v2_start_ + i];
            double v3 = vars[v3_start_ + i], v4 = vars[v4_start_ + i];
            double d1 = vars[d1_start_ + i], d2 = vars[d2_start_ + i];
            double d3 = vars[d3_start_ + i], d4 = vars[d4_start_ + i];

            double dx = 0.25 * (v1 * std::cos(d1 + syaw) + v2 * std::cos(d2 + syaw) +
                                 v3 * std::cos(d3 + syaw) + v4 * std::cos(d4 + syaw));
            double dy = 0.25 * (v1 * std::sin(d1 + syaw) + v2 * std::sin(d2 + syaw) +
                                 v3 * std::sin(d3 + syaw) + v4 * std::sin(d4 + syaw));
            double dtheta = ((-yw[0]*std::cos(d1)+xw[0]*std::sin(d1))*v1 +
                              (-yw[1]*std::cos(d2)+xw[1]*std::sin(d2))*v2 +
                              (-yw[2]*std::cos(d3)+xw[2]*std::sin(d3))*v3 +
                              (-yw[3]*std::cos(d4)+xw[3]*std::sin(d4))*v4) / denom;

            sx += dx * NMPC_DT;
            sy += dy * NMPC_DT;
            syaw += dtheta * NMPC_DT;

            vars[x_start_ + i + 1]   = sx;
            vars[y_start_ + i + 1]   = sy;
            vars[yaw_start_ + i + 1] = syaw;
        }
    }

    Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);
    for (size_t i = 0; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0e8;
        vars_upperbound[i] =  1.0e8;
    }
    auto set_bounds = [&](int start, double lo, double hi) {
        for (int i = start; i < start + NMPC_T - 1; i++) {
            vars_lowerbound[i] = lo;
            vars_upperbound[i] = hi;
        }
    };
    const auto& limits = params_.control_limits;
    set_bounds(v1_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(v2_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(v3_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(v4_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(d1_start_, -limits.max_steering_angle, limits.max_steering_angle);
    set_bounds(d2_start_, -limits.max_steering_angle, limits.max_steering_angle);
    set_bounds(d3_start_, -limits.max_steering_angle, limits.max_steering_angle);
    set_bounds(d4_start_, -limits.max_steering_angle, limits.max_steering_angle);

    Dvector constraints_lowerbound(n_constraints), constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start_]   = x;
    constraints_upperbound[x_start_]   = x;
    constraints_lowerbound[y_start_]   = y;
    constraints_upperbound[y_start_]   = y;
    constraints_lowerbound[yaw_start_] = yaw;
    constraints_upperbound[yaw_start_] = yaw;

    {
        int off = NMPC_T * 3; 
        const double max_steering_step = limits.max_steering_rate * params_.dt;
        const double max_speed_step = limits.max_wheel_acceleration * params_.dt;
        for (int i = 0; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 4; k++) {
                constraints_lowerbound[off] = -max_steering_step;
                constraints_upperbound[off] =  max_steering_step;
                off++;
            }
        }
        for (int i = 0; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 4; k++) {
                constraints_lowerbound[off] = -max_speed_step;
                constraints_upperbound[off] =  max_speed_step;
                off++;
            }
        }

        // 修改/新增：为新增的 5 * (T-1) 个刚体几何约束设置上下界
        // 既然是严格等式约束，上下界必须强行锁死在 0.0
        for (int i = 0; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 5; k++) {
                constraints_lowerbound[off] = 0.0;
                constraints_upperbound[off] = 0.0;
                off++;
            }
        }
    }

    FG_EVAL_DIFF fg_eval(traj_ref, previous_control, params_.L, params_.W);

    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        reverse\n";
    options += "Integer max_iter      150\n";
    options += "Numeric max_cpu_time          0.3\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve<Dvector, FG_EVAL_DIFF>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
    if (!ok) {
        std::cout << "[diffMpcController] IPOPT 未能成功收敛, status=" << solution.status
                   << "，本周期沿用上一时刻控制量，避免采用病态解" << std::endl;
        return previous_control;
    }

    Eigen::VectorXd U_out(NMPC_NU);
    U_out(0) = solution.x[v1_start_];
    U_out(1) = solution.x[v2_start_];
    U_out(2) = solution.x[v3_start_];
    U_out(3) = solution.x[v4_start_];
    U_out(4) = solution.x[d1_start_];
    U_out(5) = solution.x[d2_start_];
    U_out(6) = solution.x[d3_start_];
    U_out(7) = solution.x[d4_start_];

    U = U_out;
    return U_out;
}
