#include "mpc_local_planner/nmpc_core/diffmpc.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

// ===================== 工具函数：保留与线性版本一致的接口 =====================

std::vector<double> diffMpcController::calculateReferenceSpeeds(
    const std::vector<double>& curvatures,
    const double& max_speed,
    double curvature_gain) {
    std::vector<double> referenceSpeeds;
    referenceSpeeds.reserve(curvatures.size()); 
    const double gain = std::isfinite(curvature_gain) && curvature_gain >= 0.0
                            ? curvature_gain
                            : 3.0;
    for (double k : curvatures) {
        // 用倒数曲线根据曲率限速
        double speed = max_speed / (1.0 + gain * std::fabs(k));
        // speed 必然 <= max_speed，因此只需限制下限
        speed = std::max(0.05, speed); 
        referenceSpeeds.push_back(speed);
    }
    return referenceSpeeds;

}

void diffMpcController::smooth_yaw(std::vector<double>& cyaw) {
    // 将航向角平滑到 [-pi, pi] 范围内，避免跳变
    for (int i = 0; i < (int)cyaw.size() - 1; i++) { // 从第一个点开始，逐个比较当前点 cyaw[i] 和下一个点 cyaw[i + 1]
        double dyaw = cyaw[i + 1] - cyaw[i]; // 计算前后两个点的角度差 dyaw 正常情况下，相邻两个路径点的角度变化应该非常小
        while (dyaw > M_PI / 2.0) { // 如果算出来的差值 dyaw > 90°（比如从 $-179^\circ$ 突然变成了 $179^\circ$，差值是 $358^\circ$），说明发生了跳变
            cyaw[i + 1] -= M_PI * 2.0;  // 强行给下一个点的角度减去 $360^\circ$（M_PI * 2.0），让它从 $179^\circ$ 变成 $-181^\circ$。这样它跟前一个点（$-179^\circ$）就连续了
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
    // 计算 min_index 对应的累计弧长
    int ind = std::clamp(std::max(min_index, target_ind), 0,
                         static_cast<int>(ncourse) - 1);
    int segment_index = ind;
    double segment_start_distance = 0.0;
    // 计算从起点到 segment_index 的累计弧长
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
    fg[0] = 0;  // 目标函数初始化   每次 IPOPT 调用该函数，都要从零重新累计目标函数。fg[0] 是 AD<double>，赋值 0 会自动转换成 AD 常量。

    // 四个轮子在车体坐标系下的位置 (FL, RL, RR, FR)
    double xw[4] = {L, -L, -L, L};
    double yw[4] = {W, W, -W, -W};
    double denom = 4.0 * (L * L + W * W);   // 分母：四个轮子到车辆中心距离平方之和

    const double w_pos = 1.0;     // 位置跟踪权重       1.0
    const double w_yaw = 2.0;     // 航向跟踪权重   0.5
    const double w_v   = 2.0;     // 速度跟踪权重   2.0
    const double w_u   = 0.01;    // 控制量大小权重
    const double w_du  = 0.5;     // 控制量变化率权重   0.5 w_du 越大 → 控制更平滑，但响应可能更慢 w_du 越小 → 响应更快，但控制变化可能更剧烈

    // ---- 控制量代价（大小 + 相邻步变化率 + 速度跟踪） ----
    // 每个控制步的代价可概括为 J_control =0.01 × Σ(v_j² + d_j²)+ 2.0  × (v_ref - v_avg)²+ 0.5  × ||u_i - u_previous||²
    // 第一项抑制过大的轮速和转角；第二项跟踪参考速度；第三项让控制变化更平滑
    for (int i = 0; i < NMPC_T - 1; i++) {
        AD<double> v1 = vars[v1_start_ + i];
        AD<double> v2 = vars[v2_start_ + i];
        AD<double> v3 = vars[v3_start_ + i];
        AD<double> v4 = vars[v4_start_ + i];
        AD<double> d1 = vars[d1_start_ + i];
        AD<double> d2 = vars[d2_start_ + i];
        AD<double> d3 = vars[d3_start_ + i];
        AD<double> d4 = vars[d4_start_ + i];
        // 惩罚过大的轮速 fg[0] += w_u × (v1² + v2² + v3² + v4²)
        fg[0] += w_u * (CppAD::pow(v1, 2) + CppAD::pow(v2, 2) + CppAD::pow(v3, 2) + CppAD::pow(v4, 2));
        // 惩罚过大的转角 fg[0] += w_u × (d1² + d2² + d3² + d4²) 让优化器倾向于使用较小转角，也就是倾向于让轮子朝向车体前方
        fg[0] += w_u * (CppAD::pow(d1, 2) + CppAD::pow(d2, 2) + CppAD::pow(d3, 2) + CppAD::pow(d4, 2));
        // v_avg 是四个有符号轮速标量的平均值，不是严格的车体纵向速度
        AD<double> v_avg = 0.25 * (v1 + v2 + v3 + v4);
        // 参考速度跟踪代价 第 i+1 个状态节点的参考速度。之所以使用 i+1，是因为第 i 个控制量负责生成第 i+1 个状态
        // fg[0] += w_v × (参考速度 - 平均轮速)² 让平均轮速靠近参考速度
        fg[0] += w_v * CppAD::pow(traj_ref(3, i + 1) - v_avg, 2);
        // 查找“前一个轮速”当 i=0 时，预测序列中不存在 v1[-1]，因此使用上一周期实际执行的轮速 pv1 = U_prev(0)
        // 当 i=1 时：pv1 = vars[v1_start_ + 0] = v1[0]
        AD<double> pv1 = (i == 0) ? AD<double>(U_prev(0)) : vars[v1_start_ + i - 1];
        AD<double> pv2 = (i == 0) ? AD<double>(U_prev(1)) : vars[v2_start_ + i - 1];
        AD<double> pv3 = (i == 0) ? AD<double>(U_prev(2)) : vars[v3_start_ + i - 1];
        AD<double> pv4 = (i == 0) ? AD<double>(U_prev(3)) : vars[v4_start_ + i - 1];
        // 查找“前一个转角”
        AD<double> pd1 = (i == 0) ? AD<double>(U_prev(4)) : vars[d1_start_ + i - 1];
        AD<double> pd2 = (i == 0) ? AD<double>(U_prev(5)) : vars[d2_start_ + i - 1];
        AD<double> pd3 = (i == 0) ? AD<double>(U_prev(6)) : vars[d3_start_ + i - 1];
        AD<double> pd4 = (i == 0) ? AD<double>(U_prev(7)) : vars[d4_start_ + i - 1];
        // 处理四个轮子的速度变化 v1～v4当前预测步的四轮速度 pv1～pv4用于比较的上一组四轮速度
        fg[0] += w_du * (CppAD::pow(v1 - pv1, 2) + CppAD::pow(v2 - pv2, 2) +
                          CppAD::pow(v3 - pv3, 2) + CppAD::pow(v4 - pv4, 2));
        // 处理四个轮子的转角变化 d1～d4当前预测步的四轮转角 pd1～pd4用于比较的上一组四轮转角
        fg[0] += w_du * (CppAD::pow(d1 - pd1, 2) + CppAD::pow(d2 - pd2, 2) +
                          CppAD::pow(d3 - pd3, 2) + CppAD::pow(d4 - pd4, 2));
        // J_du = w_du × Σ[四轮速度变化² + 四轮转角变化²]
    }

    // ---- 初始状态约束 ----
    // 固定预测轨迹的第一个状态为当前实测状态  根据四轮速度和转角预测下一时刻状态  把预测状态与参考轨迹之间的误差加入目标函数
    // fg[0]     = 目标函数     fg[1 + j] = 第 j 条约束 x_start_   = 0  y_start_   = 10 yaw_start_ = 20
    // 把预测轨迹的起点锁定在车辆当前实测状态
    fg[1 + x_start_]   = vars[x_start_];    // fg[1] = vars[0]= current_x; vars[0] 是预测轨迹的第一个横坐标 x[0]
    fg[1 + y_start_]   = vars[y_start_];    // fg[11] = vars[10] = y[0]= current_y
    fg[1 + yaw_start_] = vars[yaw_start_];  // fg[21] = vars[20] = yaw[0]= current_yaw

    // ---- 动力学约束 + 轨迹跟踪代价 ----
    for (int i = 0; i < NMPC_T - 1; i++) {
        // 读取当前状态
        AD<double> x0   = vars[x_start_ + i];
        AD<double> y0   = vars[y_start_ + i];
        AD<double> yaw0 = vars[yaw_start_ + i];
        // 读取下一状态
        AD<double> x1   = vars[x_start_ + i + 1];
        AD<double> y1   = vars[y_start_ + i + 1];
        AD<double> yaw1 = vars[yaw_start_ + i + 1];
        // 读取当前控制量   第 i 个控制区间内四个轮子的有符号线速度
        AD<double> v1 = vars[v1_start_ + i];
        AD<double> v2 = vars[v2_start_ + i];
        AD<double> v3 = vars[v3_start_ + i];
        AD<double> v4 = vars[v4_start_ + i];
        // 读取对应转角 d1～d4 是车轮滚动方向相对于车体前进方向的夹角
        AD<double> d1 = vars[d1_start_ + i];
        AD<double> d2 = vars[d2_start_ + i];
        AD<double> d3 = vars[d3_start_ + i];
        AD<double> d4 = vars[d4_start_ + i];
        // 计算世界坐标系 x 方向速度
        AD<double> dx = 0.25 * (v1 * CppAD::cos(d1 + yaw0) + v2 * CppAD::cos(d2 + yaw0) +
                                 v3 * CppAD::cos(d3 + yaw0) + v4 * CppAD::cos(d4 + yaw0));
        // 计算世界坐标系 y 方向速度
        AD<double> dy = 0.25 * (v1 * CppAD::sin(d1 + yaw0) + v2 * CppAD::sin(d2 + yaw0) +
                                 v3 * CppAD::sin(d3 + yaw0) + v4 * CppAD::sin(d4 + yaw0));
        // 计算车辆角速度
        AD<double> dtheta = ((-yw[0] * CppAD::cos(d1) + xw[0] * CppAD::sin(d1)) * v1 +
                              (-yw[1] * CppAD::cos(d2) + xw[1] * CppAD::sin(d2)) * v2 +
                              (-yw[2] * CppAD::cos(d3) + xw[2] * CppAD::sin(d3)) * v3 +
                              (-yw[3] * CppAD::cos(d4) + xw[3] * CppAD::sin(d4)) * v4) / denom;
        // 欧拉积分预测下一状态
        AD<double> x_pred   = x0 + dx * NMPC_DT;
        AD<double> y_pred   = y0 + dy * NMPC_DT;
        AD<double> yaw_pred = yaw0 + dtheta * NMPC_DT;
        // 建立动力学等式约束   外部将这些约束的上下界都设置为零，因此要求：x1 - x_pred = 0 x[i+1] = x_pred
        // fg[0] 是目标函数 fg[1] 是 x 初始状态约束 fg[2] 才能开始存放 x 动力学约束
        fg[2 + x_start_ + i]   = x1 - x_pred;
        fg[2 + y_start_ + i]   = y1 - y_pred;
        fg[2 + yaw_start_ + i] = yaw1 - yaw_pred;
        // 位置跟踪代价 traj_ref(0,i+1) 是下一状态节点的参考 x   控制量 u[i] 把状态 i 推进到状态 i+1，所以这里与参考轨迹的第 i+1 列比较
        // w_pos × (参考x - 预测x)²
        fg[0] += w_pos * CppAD::pow(traj_ref(0, i + 1) - x_pred, 2);
        // w_pos × (参考y - 预测y)²
        fg[0] += w_pos * CppAD::pow(traj_ref(1, i + 1) - y_pred, 2);
        // 计算预测航向角与参考航向角的原始差值
        AD<double> dyaw = yaw_pred - traj_ref(2, i + 1);
        // fg[0] += w_yaw * CppAD::pow(wrapped_dyaw, 2);
        fg[0] += w_yaw * CppAD::pow(CppAD::atan2(CppAD::sin(dyaw), CppAD::cos(dyaw)), 2);
        // J += w_pos × 位置误差² + w_yaw × 航向误差²
    }

    // ---- 转向角/轮速变化率硬约束 ----
    // 前 30 条约束已经用于：x 的初始状态和动力学：10 条 y 的初始状态和动力学：10 条 yaw 的初始状态和动力学：10 条
    // 因为 fg[0] 是目标函数,fg[1 + j] = 第 j 条约束，所以新的约束从 fg[31] 开始。
    int idx = 1 + 3 * NMPC_T; 
    for (int i = 0; i < NMPC_T - 1; i++) {
        // fg[31] = expression; idx = 32;
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
    // 约束每一个控制步 (共 T-1 步) 的 4WIS 组合必须符合刚体瞬心相交原理    idx = 103
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
    // 初始状态
    double x   = initial_x(0);
    double y   = initial_x(1);
    double yaw = initial_x(2);
    // 计算参考轨迹和目标索引
    if (target_ind_state < min_index) target_ind_state = min_index;
    M_XREF_DIFF traj_ref = build_horizon_reference(min_index, cx, cy, cyaw, speed,
                                                    reference_speed, target_ind_state);
    // 计算预测时域内的变量总数和约束总数 102
    size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
    
    // 初始/动力学 + 每一控制步的变化率 + 每一控制步的刚体几何约束。
    size_t n_constraints = NMPC_T * 3 + 8 * (NMPC_T - 1) + 5 * (NMPC_T - 1);
    // 读取上一拍实际命令
    Eigen::VectorXd previous_control = Eigen::VectorXd::Zero(NMPC_NU);
    if (applied_control.size() == NMPC_NU && applied_control.allFinite()) {
        previous_control = applied_control;
    }
    // 将所有变量初始化为零，然后将第一个预测状态设为当前状态
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) vars[i] = 0.0;

    vars[x_start_]   = x;
    vars[y_start_]   = y;
    vars[yaw_start_] = yaw;
    // 用上一拍控制命令 U 初始化未来 9 个控制步     根据这些初始控制量向前模拟车辆运动，生成未来状态的初始猜测。
    // 把完整初值交给 IPOPT，帮助它更快收敛 IPOPT 后面仍然会修改这些变量
    // 第一段循环初始化未来控制变量，第二段代码用四轮运动学把这些控制量转换成一条满足动力学关系的状态初值
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
        double denom = 4.0 * (params_.L * params_.L + params_.W * params_.W);   // 分母用于根据四轮速度计算车体角速度
        double sx = x, sy = y, syaw = yaw;  // 状态模拟的起点   初始值就是当前实测状态。随后每个预测步都会更新一次
        for (int i = 0; i < NMPC_T - 1; i++) {
            // 前面已经把每个控制步初始化为相同的 U，所以第一次进入这里时，9 个预测步的控制量相同。
            double v1 = vars[v1_start_ + i], v2 = vars[v2_start_ + i];
            double v3 = vars[v3_start_ + i], v4 = vars[v4_start_ + i];
            double d1 = vars[d1_start_ + i], d2 = vars[d2_start_ + i];
            double d3 = vars[d3_start_ + i], d4 = vars[d4_start_ + i];

            double dx = 0.25 * (v1 * std::cos(d1 + syaw) + v2 * std::cos(d2 + syaw) +
                                 v3 * std::cos(d3 + syaw) + v4 * std::cos(d4 + syaw));
            double dy = 0.25 * (v1 * std::sin(d1 + syaw) + v2 * std::sin(d2 + syaw) +
                                 v3 * std::sin(d3 + syaw) + v4 * std::sin(d4 + syaw));
            // omega = Σ[-yi·qix + xi·qiy] / Σ[xi² + yi²]   dtheta 实际表示车辆角速度 omega，单位是 rad/s
            double dtheta = ((-yw[0]*std::cos(d1)+xw[0]*std::sin(d1))*v1 +
                              (-yw[1]*std::cos(d2)+xw[1]*std::sin(d2))*v2 +
                              (-yw[2]*std::cos(d3)+xw[2]*std::sin(d3))*v3 +
                              (-yw[3]*std::cos(d4)+xw[3]*std::sin(d4))*v4) / denom;
            // 欧拉积分预测未来状态 当前 NMPC_DT=0.1 s，所以使用离散欧拉积分
            sx += dx * NMPC_DT;
            sy += dy * NMPC_DT;
            syaw += dtheta * NMPC_DT;
            // 写入未来状态初值 使用更新后的 syaw 进入下一次循环，因此车辆方向会随预测旋转逐步改变
            // 当前状态已经存放在第 0 个状态 所以预测的第 i+1 个状态存放在 x_start_ + i + 1
            // 最终 vars 中形成一条由恒定控制 U 正向模拟得到的完整初始轨迹
            vars[x_start_ + i + 1]   = sx;
            vars[y_start_ + i + 1]   = sy;
            vars[yaw_start_ + i + 1] = syaw;
        }
    }
    // IPOPT 设置两类边界
    Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);   // 变量边界 变量边界直接限制轮速、转角等决策变量
    for (size_t i = 0; i < n_vars; i++) {
        // 默认设置为近似无限制 -100000000 <= vars[i] <= 100000000
        // x、y、yaw：基本无限制 轮速、转角：暂时也基本无限制
        vars_lowerbound[i] = -1.0e8;
        vars_upperbound[i] =  1.0e8;
    }
    // 局部函数 Lambda，用于给一整组连续控制变量设置相同边界    lo：下界 hi：上界
    auto set_bounds = [&](int start, double lo, double hi) {
        for (int i = start; i < start + NMPC_T - 1; i++) {
            vars_lowerbound[i] = lo;
            vars_upperbound[i] = hi;
        }
    };
    // 轮速和转角绝对限制
    const auto& limits = params_.control_limits;
    set_bounds(v1_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(v2_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(v3_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(v4_start_, -limits.max_wheel_speed, limits.max_wheel_speed);
    set_bounds(d1_start_, -limits.max_steering_angle, limits.max_steering_angle);
    set_bounds(d2_start_, -limits.max_steering_angle, limits.max_steering_angle);
    set_bounds(d3_start_, -limits.max_steering_angle, limits.max_steering_angle);
    set_bounds(d4_start_, -limits.max_steering_angle, limits.max_steering_angle);
    // 约束函数边界 约束边界限制动力学残差、控制变化量和刚体几何残差
    Dvector constraints_lowerbound(n_constraints), constraints_upperbound(n_constraints);
    // 默认把约束设置为等式零
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    // 固定预测初始状态
    constraints_lowerbound[x_start_]   = x;
    constraints_upperbound[x_start_]   = x;
    constraints_lowerbound[y_start_]   = y;
    constraints_upperbound[y_start_]   = y;
    constraints_lowerbound[yaw_start_] = yaw;
    constraints_upperbound[yaw_start_] = yaw;

    {
        int off = NMPC_T * 3;   // off 指向控制约束起点off = 10 × 3 = 30
        // 前 0...29 共 30 个位置已经属于初始状态和动力学约束。因此从第 30 个约束开始写控制变化率边界
        const double max_steering_step = limits.max_steering_rate * params_.dt;
        const double max_speed_step = limits.max_wheel_acceleration * params_.dt;
        // 30...65：   转角变化率
        for (int i = 0; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 4; k++) {
                constraints_lowerbound[off] = -max_steering_step;
                constraints_upperbound[off] =  max_steering_step;
                off++;
            }
        }
        // 66...101：  轮速变化率
        for (int i = 0; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 4; k++) {
                constraints_lowerbound[off] = -max_speed_step;
                constraints_upperbound[off] =  max_speed_step;
                off++;
            }
        }

        // 修改/新增：为新增的 5 * (T-1) 个刚体几何约束设置上下界   102...146：刚体几何约束
        // 既然是严格等式约束，上下界必须强行锁死在 0.0
        for (int i = 0; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 5; k++) {
                constraints_lowerbound[off] = 0.0;
                constraints_upperbound[off] = 0.0;
                off++;
            }
        }
    }
    // 定义目标函数和约束。 调用 IPOPT 求最优预测轨迹。 只取预测时域中的第一组控制量执行。
    // 构造 FG_EVAL_DIFF    创建“目标函数和约束计算器”，还没有开始求解
    // traj_ref：未来 10 个状态节点的参考轨迹，包含 [x, y, yaw, v_ref]  previous_control：上一周期实际下发的 8 维控制量
    // params_.L：车辆中心到前后轮的纵向距离    params_.W：车辆中心到左右轮的横向距离
    FG_EVAL_DIFF fg_eval(traj_ref, previous_control, params_.L, params_.W);

    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        reverse\n";
    options += "Integer max_iter      150\n";
    options += "Numeric max_cpu_time          0.3\n";

    CppAD::ipopt::solve_result<Dvector> solution;
    // 调用求解器   Dvector：普通 double 数值向量类型   FG_EVAL_DIFF：目标函数和约束函数类型
    // options：求解器配置  vars：IPOPT 的初始猜测  vars_lowerbound/upperbound：决策变量的上下界
    // constraints_lowerbound/upperbound：约束函数的上下界  fg_eval：负责计算目标函数和约束 solution：接收最终求解结果
    CppAD::ipopt::solve<Dvector, FG_EVAL_DIFF>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);
    // 判断求解状态 状态精确等于 success 才接受结果
    bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
    if (!ok) {
        std::cout << "[diffMpcController] IPOPT 未能成功收敛, status=" << solution.status
                   << "，本周期沿用上一时刻控制量，避免采用病态解" << std::endl;
        return previous_control;
    }
    // 提取第一组控制量 solution.x 包含完整的 102 维最优解，但 MPC 只执行第一步控制
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
