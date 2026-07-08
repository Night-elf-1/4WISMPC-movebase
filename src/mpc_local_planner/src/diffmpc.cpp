#include "mpc_local_planner/nmpc_core/diffmpc.hpp"

// ===================== 工具函数：保留与线性版本一致的接口 =====================

std::vector<double> diffMpcController::calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed) {
    std::vector<double> referenceSpeeds;
    for (double k : curvatures) {
        // 用倒数曲线根据曲率限速，避免大曲率处速度为负
        double speed = max_speed / (1.0 + 3.0 * std::fabs(k));
        speed = std::max(0.05, std::min(max_speed, speed));
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
                                                         std::vector<double> cx, std::vector<double> cy,
                                                         std::vector<double> cyaw) {
    double mind = std::numeric_limits<double>::max();
    int ind = 0;
    for (int i = 0; i < (int)cx.size(); i++) {
        double idx = current_x - cx[i];
        double idy = current_y - cy[i];
        double d_e = std::sqrt(idx * idx + idy * idy);
        if (d_e < mind) {
            mind = d_e;
            ind = i;
        }
    }
    return std::make_tuple(ind, mind);
}

std::tuple<int, double> diffMpcController::calc_ref_trajectory(double current_x, double current_y,
                                                                 std::vector<double> cx, std::vector<double> cy,
                                                                 std::vector<double> cyaw) {
    auto [ind, d_e] = calc_nearest_index_diff(current_x, current_y, cx, cy, cyaw);
    return std::make_tuple(ind, d_e);
}

// 构建预测时域内的参考轨迹 [x,y,yaw] x T （仿照 NMPC 单车版 calc_ref_trajectory 的弧长步进逻辑）
M_XREF_DIFF diffMpcController::build_horizon_reference(int min_index, std::vector<double>& cx, std::vector<double>& cy,
                                                        std::vector<double>& cyaw, std::vector<double>& speed, double dl,
                                                        double current_v, int &target_ind) {
    M_XREF_DIFF xref = M_XREF_DIFF::Zero();
    int ncourse = (int)cx.size();

    int ind = min_index;
    if (target_ind >= ind) ind = target_ind;

    xref(0, 0) = cx[ind];
    xref(1, 0) = cy[ind];
    xref(2, 0) = cyaw[ind];
    xref(3, 0) = speed[ind];

    double travel = 0.0;
    for (int i = 0; i < NMPC_T; i++) {
        travel += std::abs(current_v) * NMPC_DT;
        int dind = (int)std::round(travel / dl);

        int use_ind = (ind + dind < ncourse) ? (ind + dind) : (ncourse - 1);
        xref(0, i) = cx[use_ind];
        xref(1, i) = cy[use_ind];
        xref(2, i) = cyaw[use_ind];
        xref(3, i) = speed[use_ind];
    }

    target_ind = ind;
    return xref;
}

// ===================== NMPC 代价 / 约束函数 =====================

FG_EVAL_DIFF::FG_EVAL_DIFF(const M_XREF_DIFF &trajRef, const Eigen::VectorXd &uPrev, double L_, double W_)
    : traj_ref(trajRef), U_prev(uPrev), L(L_), W(W_) {}

// void FG_EVAL_DIFF::operator()(FG_EVAL_DIFF::ADvector &fg, const FG_EVAL_DIFF::ADvector &vars) {
//     fg[0] = 0;

//     // 四个轮子在车体坐标系下的位置 (FL, RL, RR, FR)
//     double xw[4] = {L, -L, -L, L};
//     double yw[4] = {W, W, -W, -W};
//     double denom = 4.0 * (L * L + W * W);

//     const double w_pos = 1.0;     // 位置跟踪权重
//     const double w_yaw = 0.5;     // 航向跟踪权重
//     const double w_v   = 2.0;     // 速度跟踪权重（原来完全缺失，导致轮速一直顶满上限）
//     const double w_u   = 0.01;    // 控制量大小权重
//     const double w_du  = 0.5;     // 控制量变化率权重（软惩罚，配合下面的硬约束一起用）

//     // ---- 控制量代价（大小 + 相邻步变化率 + 速度跟踪） ----
//     for (int i = 0; i < NMPC_T - 1; i++) {
//         AD<double> v1 = vars[v1_start_ + i];
//         AD<double> v2 = vars[v2_start_ + i];
//         AD<double> v3 = vars[v3_start_ + i];
//         AD<double> v4 = vars[v4_start_ + i];
//         AD<double> d1 = vars[d1_start_ + i];
//         AD<double> d2 = vars[d2_start_ + i];
//         AD<double> d3 = vars[d3_start_ + i];
//         AD<double> d4 = vars[d4_start_ + i];

//         fg[0] += w_u * (CppAD::pow(v1, 2) + CppAD::pow(v2, 2) + CppAD::pow(v3, 2) + CppAD::pow(v4, 2));
//         fg[0] += w_u * (CppAD::pow(d1, 2) + CppAD::pow(d2, 2) + CppAD::pow(d3, 2) + CppAD::pow(d4, 2));

//         // 四轮平均轮速跟踪参考速度——这是之前版本缺失的关键一项，
//         // 没有它优化器只会发现"轮速越大越快接近参考点"，于是一直顶满 v_max，
//         // 完全不按曲率减速，弯道一来横向误差就爆掉。
//         AD<double> v_avg = 0.25 * (v1 + v2 + v3 + v4);
//         fg[0] += w_v * CppAD::pow(traj_ref(3, i + 1) - v_avg, 2);

//         AD<double> pv1 = (i == 0) ? AD<double>(U_prev(0)) : vars[v1_start_ + i - 1];
//         AD<double> pv2 = (i == 0) ? AD<double>(U_prev(1)) : vars[v2_start_ + i - 1];
//         AD<double> pv3 = (i == 0) ? AD<double>(U_prev(2)) : vars[v3_start_ + i - 1];
//         AD<double> pv4 = (i == 0) ? AD<double>(U_prev(3)) : vars[v4_start_ + i - 1];
//         AD<double> pd1 = (i == 0) ? AD<double>(U_prev(4)) : vars[d1_start_ + i - 1];
//         AD<double> pd2 = (i == 0) ? AD<double>(U_prev(5)) : vars[d2_start_ + i - 1];
//         AD<double> pd3 = (i == 0) ? AD<double>(U_prev(6)) : vars[d3_start_ + i - 1];
//         AD<double> pd4 = (i == 0) ? AD<double>(U_prev(7)) : vars[d4_start_ + i - 1];

//         fg[0] += w_du * (CppAD::pow(v1 - pv1, 2) + CppAD::pow(v2 - pv2, 2) +
//                           CppAD::pow(v3 - pv3, 2) + CppAD::pow(v4 - pv4, 2));
//         fg[0] += w_du * (CppAD::pow(d1 - pd1, 2) + CppAD::pow(d2 - pd2, 2) +
//                           CppAD::pow(d3 - pd3, 2) + CppAD::pow(d4 - pd4, 2));
//     }

//     // ---- 初始状态约束 ----
//     fg[1 + x_start_]   = vars[x_start_];
//     fg[1 + y_start_]   = vars[y_start_];
//     fg[1 + yaw_start_] = vars[yaw_start_];

//     // ---- 动力学约束 + 轨迹跟踪代价 ----
//     for (int i = 0; i < NMPC_T - 1; i++) {
//         AD<double> x0   = vars[x_start_ + i];
//         AD<double> y0   = vars[y_start_ + i];
//         AD<double> yaw0 = vars[yaw_start_ + i];

//         AD<double> x1   = vars[x_start_ + i + 1];
//         AD<double> y1   = vars[y_start_ + i + 1];
//         AD<double> yaw1 = vars[yaw_start_ + i + 1];

//         AD<double> v1 = vars[v1_start_ + i];
//         AD<double> v2 = vars[v2_start_ + i];
//         AD<double> v3 = vars[v3_start_ + i];
//         AD<double> v4 = vars[v4_start_ + i];
//         AD<double> d1 = vars[d1_start_ + i];
//         AD<double> d2 = vars[d2_start_ + i];
//         AD<double> d3 = vars[d3_start_ + i];
//         AD<double> d4 = vars[d4_start_ + i];

//         // 非线性运动学模型（与 KinematicModel_MPC::updatestate 保持一致）
//         AD<double> dx = 0.25 * (v1 * CppAD::cos(d1 + yaw0) + v2 * CppAD::cos(d2 + yaw0) +
//                                  v3 * CppAD::cos(d3 + yaw0) + v4 * CppAD::cos(d4 + yaw0));
//         AD<double> dy = 0.25 * (v1 * CppAD::sin(d1 + yaw0) + v2 * CppAD::sin(d2 + yaw0) +
//                                  v3 * CppAD::sin(d3 + yaw0) + v4 * CppAD::sin(d4 + yaw0));

//         AD<double> dtheta = ((-yw[0] * CppAD::cos(d1) + xw[0] * CppAD::sin(d1)) * v1 +
//                               (-yw[1] * CppAD::cos(d2) + xw[1] * CppAD::sin(d2)) * v2 +
//                               (-yw[2] * CppAD::cos(d3) + xw[2] * CppAD::sin(d3)) * v3 +
//                               (-yw[3] * CppAD::cos(d4) + xw[3] * CppAD::sin(d4)) * v4) / denom;

//         AD<double> x_pred   = x0 + dx * NMPC_DT;
//         AD<double> y_pred   = y0 + dy * NMPC_DT;
//         AD<double> yaw_pred = yaw0 + dtheta * NMPC_DT;

//         fg[2 + x_start_ + i]   = x1 - x_pred;
//         fg[2 + y_start_ + i]   = y1 - y_pred;
//         fg[2 + yaw_start_ + i] = yaw1 - yaw_pred;

//         // 轨迹跟踪代价（直接用模型预测值，避免额外引入等式约束）
//         fg[0] += w_pos * CppAD::pow(traj_ref(0, i + 1) - x_pred, 2);
//         fg[0] += w_pos * CppAD::pow(traj_ref(1, i + 1) - y_pred, 2);
//         AD<double> dyaw = yaw_pred - traj_ref(2, i + 1);
//         fg[0] += w_yaw * CppAD::pow(CppAD::atan2(CppAD::sin(dyaw), CppAD::cos(dyaw)), 2);
//     }

//     // ---- 转向角/轮速变化率硬约束 ----
//     // 之前版本只在代价里软惩罚 Δu，权重不够大时 IPOPT 可能为了减小跟踪误差
//     // 直接让某个转角单步跳变到边界（日志里能看到从 0.2rad 跳到 1.57rad 的病态解）。
//     // 这里改成硬约束，强制相邻两步的差值落在限幅范围内。
//     int idx = 1 + 3 * NMPC_T; // fg[0]=cost, fg[1..3T]=动力学约束，紧接着追加变化率约束
//     for (int i = 1; i < NMPC_T - 1; i++) {
//         fg[idx++] = vars[d1_start_ + i] - vars[d1_start_ + i - 1];
//         fg[idx++] = vars[d2_start_ + i] - vars[d2_start_ + i - 1];
//         fg[idx++] = vars[d3_start_ + i] - vars[d3_start_ + i - 1];
//         fg[idx++] = vars[d4_start_ + i] - vars[d4_start_ + i - 1];
//     }
//     for (int i = 1; i < NMPC_T - 1; i++) {
//         fg[idx++] = vars[v1_start_ + i] - vars[v1_start_ + i - 1];
//         fg[idx++] = vars[v2_start_ + i] - vars[v2_start_ + i - 1];
//         fg[idx++] = vars[v3_start_ + i] - vars[v3_start_ + i - 1];
//         fg[idx++] = vars[v4_start_ + i] - vars[v4_start_ + i - 1];
//     }
// }

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
    for (int i = 1; i < NMPC_T - 1; i++) {
        fg[idx++] = vars[d1_start_ + i] - vars[d1_start_ + i - 1];
        fg[idx++] = vars[d2_start_ + i] - vars[d2_start_ + i - 1];
        fg[idx++] = vars[d3_start_ + i] - vars[d3_start_ + i - 1];
        fg[idx++] = vars[d4_start_ + i] - vars[d4_start_ + i - 1];
    }
    for (int i = 1; i < NMPC_T - 1; i++) {
        fg[idx++] = vars[v1_start_ + i] - vars[v1_start_ + i - 1];
        fg[idx++] = vars[v2_start_ + i] - vars[v2_start_ + i - 1];
        fg[idx++] = vars[v3_start_ + i] - vars[v3_start_ + i - 1];
        fg[idx++] = vars[v4_start_ + i] - vars[v4_start_ + i - 1];
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

// ===================== NMPC 求解 =====================

// Eigen::VectorXd diffMpcController::mpc_solve(std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw,
//                                               std::vector<double>& ck, std::vector<double>& speed, Eigen::Vector3d inital_x,
//                                               int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_) {
//     typedef CPPAD_TESTVECTOR(double) Dvector;

//     double x   = inital_x(0);
//     double y   = inital_x(1);
//     double yaw = inital_x(2);

//     // 构建预测时域内的参考轨迹
//     static int target_ind_state = min_index; // 简单持久化，避免目标点回退
//     if (target_ind_state < min_index) target_ind_state = min_index;
//     M_XREF_DIFF traj_ref = build_horizon_reference(min_index, cx, cy, cyaw, speed, 1.0,
//                                                     agv_model.v, target_ind_state);

//     size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
//     // 3T 个动力学约束 + 8*(T-2) 个变化率硬约束（4个转角 + 4个轮速，各 T-2 步）
//     size_t n_constraints = NMPC_T * 3 + 8 * (NMPC_T - 2);

//     Dvector vars(n_vars);
//     for (size_t i = 0; i < n_vars; i++) vars[i] = 0.0;

//     vars[x_start_]   = x;
//     vars[y_start_]   = y;
//     vars[yaw_start_] = yaw;
//     // 用上一时刻控制量作为控制变量的初始猜测，有助于 IPOPT 收敛
//     for (int i = 0; i < NMPC_T - 1; i++) {
//         vars[v1_start_ + i] = U(0);
//         vars[v2_start_ + i] = U(1);
//         vars[v3_start_ + i] = U(2);
//         vars[v4_start_ + i] = U(3);
//         vars[d1_start_ + i] = U(4);
//         vars[d2_start_ + i] = U(5);
//         vars[d3_start_ + i] = U(6);
//         vars[d4_start_ + i] = U(7);
//     }
//     // 内部状态 (x,y,yaw) 的初值不能留 0——之前版本只给了第一个点真实状态，
//     // 其余 T-1 个点全是 0，离可行域太远，IPOPT 在误差变大的弯道处很容易求解失败。
//     // 这里用上面猜测的控制量沿非线性模型做一次前向仿真，把整条轨迹的初值填满。
//     {
//         double xw[4] = {params_.L, -params_.L, -params_.L, params_.L};
//         double yw[4] = {params_.W, params_.W, -params_.W, -params_.W};
//         double denom = 4.0 * (params_.L * params_.L + params_.W * params_.W);
//         double sx = x, sy = y, syaw = yaw;
//         for (int i = 0; i < NMPC_T - 1; i++) {
//             double v1 = vars[v1_start_ + i], v2 = vars[v2_start_ + i];
//             double v3 = vars[v3_start_ + i], v4 = vars[v4_start_ + i];
//             double d1 = vars[d1_start_ + i], d2 = vars[d2_start_ + i];
//             double d3 = vars[d3_start_ + i], d4 = vars[d4_start_ + i];

//             double dx = 0.25 * (v1 * std::cos(d1 + syaw) + v2 * std::cos(d2 + syaw) +
//                                  v3 * std::cos(d3 + syaw) + v4 * std::cos(d4 + syaw));
//             double dy = 0.25 * (v1 * std::sin(d1 + syaw) + v2 * std::sin(d2 + syaw) +
//                                  v3 * std::sin(d3 + syaw) + v4 * std::sin(d4 + syaw));
//             double dtheta = ((-yw[0]*std::cos(d1)+xw[0]*std::sin(d1))*v1 +
//                               (-yw[1]*std::cos(d2)+xw[1]*std::sin(d2))*v2 +
//                               (-yw[2]*std::cos(d3)+xw[2]*std::sin(d3))*v3 +
//                               (-yw[3]*std::cos(d4)+xw[3]*std::sin(d4))*v4) / denom;

//             sx += dx * NMPC_DT;
//             sy += dy * NMPC_DT;
//             syaw += dtheta * NMPC_DT;

//             vars[x_start_ + i + 1]   = sx;
//             vars[y_start_ + i + 1]   = sy;
//             vars[yaw_start_ + i + 1] = syaw;
//         }
//     }

//     Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);
//     for (size_t i = 0; i < n_vars; i++) {
//         vars_lowerbound[i] = -1.0e8;
//         vars_upperbound[i] =  1.0e8;
//     }
//     auto set_bounds = [&](int start, double lo, double hi) {
//         for (int i = start; i < start + NMPC_T - 1; i++) {
//             vars_lowerbound[i] = lo;
//             vars_upperbound[i] = hi;
//         }
//     };
//     set_bounds(v1_start_, NMPC_V_MIN, NMPC_V_MAX);
//     set_bounds(v2_start_, NMPC_V_MIN, NMPC_V_MAX);
//     set_bounds(v3_start_, NMPC_V_MIN, NMPC_V_MAX);
//     set_bounds(v4_start_, NMPC_V_MIN, NMPC_V_MAX);
//     set_bounds(d1_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);
//     set_bounds(d2_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);
//     set_bounds(d3_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);
//     set_bounds(d4_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);

//     Dvector constraints_lowerbound(n_constraints), constraints_upperbound(n_constraints);
//     for (size_t i = 0; i < n_constraints; i++) {
//         constraints_lowerbound[i] = 0;
//         constraints_upperbound[i] = 0;
//     }
//     constraints_lowerbound[x_start_]   = x;
//     constraints_upperbound[x_start_]   = x;
//     constraints_lowerbound[y_start_]   = y;
//     constraints_upperbound[y_start_]   = y;
//     constraints_lowerbound[yaw_start_] = yaw;
//     constraints_upperbound[yaw_start_] = yaw;

//     // 变化率硬约束的上下界（对应 operator() 里追加的 8*(T-2) 个约束方程，顺序：先4个转角，再4个轮速）
//     {
//         int off = NMPC_T * 3; // 0-indexed: 前面 3T 个是动力学约束
//         for (int i = 1; i < NMPC_T - 1; i++) {
//             for (int k = 0; k < 4; k++) {
//                 constraints_lowerbound[off] = -NMPC_DDELTA_MAX;
//                 constraints_upperbound[off] =  NMPC_DDELTA_MAX;
//                 off++;
//             }
//         }
//         for (int i = 1; i < NMPC_T - 1; i++) {
//             for (int k = 0; k < 4; k++) {
//                 constraints_lowerbound[off] = -NMPC_DV_MAX;
//                 constraints_upperbound[off] =  NMPC_DV_MAX;
//                 off++;
//             }
//         }
//     }

//     FG_EVAL_DIFF fg_eval(traj_ref, U, params_.L, params_.W);

//     std::string options;
//     options += "Integer print_level  0\n";
//     options += "Sparse  true        reverse\n";
//     options += "Integer max_iter      150\n";
//     options += "Numeric max_cpu_time          0.3\n";

//     CppAD::ipopt::solve_result<Dvector> solution;
//     CppAD::ipopt::solve<Dvector, FG_EVAL_DIFF>(
//         options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
//         constraints_upperbound, fg_eval, solution);

//     bool ok = (solution.status == CppAD::ipopt::solve_result<Dvector>::success);
//     if (!ok) {
//         std::cout << "[diffMpcController] IPOPT 未能成功收敛, status=" << solution.status
//                    << "，本周期沿用上一时刻控制量，避免采用病态解" << std::endl;
//         // 求解失败时 solution.x 不可信（之前版本直接用了它，导致出现转角瞬间打满、
//         // 轮速变负这种明显不合理的控制量）。这里直接返回上一时刻的控制量，
//         // 让控制器“原地保持”而不是抽风，下一周期 warm start 也更容易恢复。
//         return U;
//     }

//     // 取第一步控制量作为本周期实际下发的控制
//     Eigen::VectorXd U_out(8);
//     U_out(0) = solution.x[v1_start_];
//     U_out(1) = solution.x[v2_start_];
//     U_out(2) = solution.x[v3_start_];
//     U_out(3) = solution.x[v4_start_];
//     U_out(4) = solution.x[d1_start_];
//     U_out(5) = solution.x[d2_start_];
//     U_out(6) = solution.x[d3_start_];
//     U_out(7) = solution.x[d4_start_];

//     U = U_out; // 记录用于下一周期的控制量平滑/初始猜测
//     std::cout << "U_out: " << U_out.transpose() << std::endl;
//     return U_out;
// }


Eigen::VectorXd diffMpcController::mpc_solve(std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw,
                                              std::vector<double>& ck, std::vector<double>& speed, Eigen::Vector3d inital_x,
                                              int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x   = inital_x(0);
    double y   = inital_x(1);
    double yaw = inital_x(2);

    if (target_ind_state < min_index) target_ind_state = min_index;
    // 参考路径发布器以 0.5m 为间隔采样，dl 取 0.5 保证预测时域按实际弧长步进
    M_XREF_DIFF traj_ref = build_horizon_reference(min_index, cx, cy, cyaw, speed, 0.5,
                                                    agv_model.v, target_ind_state);

    size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
    
    // 修改：原本有 3T + 8*(T-2) 个约束。现在每个预测控制步增加 5 个约束，共增加 5 * (T-1) 个
    size_t n_constraints = NMPC_T * 3 + 8 * (NMPC_T - 2) + 5 * (NMPC_T - 1);

    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; i++) vars[i] = 0.0;

    vars[x_start_]   = x;
    vars[y_start_]   = y;
    vars[yaw_start_] = yaw;

    for (int i = 0; i < NMPC_T - 1; i++) {
        vars[v1_start_ + i] = U(0);
        vars[v2_start_ + i] = U(1);
        vars[v3_start_ + i] = U(2);
        vars[v4_start_ + i] = U(3);
        vars[d1_start_ + i] = U(4);
        vars[d2_start_ + i] = U(5);
        vars[d3_start_ + i] = U(6);
        vars[d4_start_ + i] = U(7);
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
    set_bounds(v1_start_, NMPC_V_MIN, NMPC_V_MAX);
    set_bounds(v2_start_, NMPC_V_MIN, NMPC_V_MAX);
    set_bounds(v3_start_, NMPC_V_MIN, NMPC_V_MAX);
    set_bounds(v4_start_, NMPC_V_MIN, NMPC_V_MAX);
    set_bounds(d1_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);
    set_bounds(d2_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);
    set_bounds(d3_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);
    set_bounds(d4_start_, NMPC_DELTA_MIN, NMPC_DELTA_MAX);

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
        for (int i = 1; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 4; k++) {
                constraints_lowerbound[off] = -NMPC_DDELTA_MAX;
                constraints_upperbound[off] =  NMPC_DDELTA_MAX;
                off++;
            }
        }
        for (int i = 1; i < NMPC_T - 1; i++) {
            for (int k = 0; k < 4; k++) {
                constraints_lowerbound[off] = -NMPC_DV_MAX;
                constraints_upperbound[off] =  NMPC_DV_MAX;
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

    FG_EVAL_DIFF fg_eval(traj_ref, U, params_.L, params_.W);

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
        return U;
    }

    Eigen::VectorXd U_out(8);
    U_out(0) = solution.x[v1_start_];
    U_out(1) = solution.x[v2_start_];
    U_out(2) = solution.x[v3_start_];
    U_out(3) = solution.x[v4_start_];
    U_out(4) = solution.x[d1_start_];
    U_out(5) = solution.x[d2_start_];
    U_out(6) = solution.x[d3_start_];
    U_out(7) = solution.x[d4_start_];

    U = U_out; 
    // std::cout << "U_out: " << U_out.transpose() << std::endl;
    return U_out;
}

// ===================== 车辆运动学更新（与原版一致，未改动） =====================

void KinematicModel_MPC::updatestate(Eigen::VectorXd U_cmd) {
    U = U_cmd;
    double v1 = U(0), v2 = U(1), v3 = U(2), v4 = U(3);
    double d1 = U(4), d2 = U(5), d3 = U(6), d4 = U(7);

    double dx = 0.25 * (v1 * std::cos(d1 + yaw) + v2 * std::cos(d2 + yaw) +
                        v3 * std::cos(d3 + yaw) + v4 * std::cos(d4 + yaw));
    double dy = 0.25 * (v1 * std::sin(d1 + yaw) + v2 * std::sin(d2 + yaw) +
                        v3 * std::sin(d3 + yaw) + v4 * std::sin(d4 + yaw));

    double denom = 4.0 * (L * L + W * W);
    double xw1 = L, yw1 = W;
    double xw2 = -L, yw2 = W;
    double xw3 = -L, yw3 = -W;
    double xw4 = L, yw4 = -W;

    double d_theta = ( (-yw1*std::cos(d1) + xw1*std::sin(d1))*v1 +
                       (-yw2*std::cos(d2) + xw2*std::sin(d2))*v2 +
                       (-yw3*std::cos(d3) + xw3*std::sin(d3))*v3 +
                       (-yw4*std::cos(d4) + xw4*std::sin(d4))*v4 ) / denom;

    x += dx * dt;
    y += dy * dt;
    yaw += d_theta * dt;
    v = 0.25 * (v1 + v2 + v3 + v4);
}

std::tuple<double, double, double, double> KinematicModel_MPC::getstate() {
    return std::make_tuple(x, y, yaw, v);
}