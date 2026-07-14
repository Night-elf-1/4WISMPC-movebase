//
// 非线性MPC (NMPC) 版本 —— 四轮独立转向/驱动 AGV 路径跟踪
// 由线性化(QP/OSQP)版本改写为 CppAD + IPOPT 非线性求解版本，
// 框架对齐 ModelPredictiveControl.h/.cpp (NMPC单车模型) 的写法。
//
#ifndef DIFFMPC_HPP
#define DIFFMPC_HPP

#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <tuple>
#include <vector>

#include "mpc_local_planner/control_limits.hpp"

using CppAD::AD;

// ============== 可调参数 ==============
// 注意：非线性求解比 QP 慢得多，预测步长/控制步长建议比线性版本(NP=30,NC=5)小很多，
// 这里采用 T 个预测步、(T-1) 个控制步（与控制步长合并，不再单独区分 NC）。
#define NMPC_NX 3        // 状态量维度 [x, y, yaw]
#define NMPC_NU 8        // 控制量维度 [v1, v2, v3, v4, d1, d2, d3, d4]
#define NMPC_T 10         // 预测步长（horizon）
#define NMPC_DT 0.1       // 采样时间

struct parameters {
    double L = 0.4615;       // 车辆中心到前后轴的距离 (Half wheelbase)，与 smart.xacro 中 front_tyre_x 一致
    double W = 0.4;          // 车辆中心到左右轮的距离 (Half track width)
    double dt = NMPC_DT;
    mpc_local_planner::ControlLimits control_limits;
};

// 参考轨迹矩阵：行 0=x,1=y,2=yaw,3=v_ref（参考速度），列为预测时域上的各时刻参考点
using M_XREF_DIFF = Eigen::Matrix<double, 4, NMPC_T>;

// 变量在 vars 数组中的起始下标
// 状态: x(T) y(T) yaw(T)
// 控制: v1(T-1) v2(T-1) v3(T-1) v4(T-1) d1(T-1) d2(T-1) d3(T-1) d4(T-1)
static const int x_start_   = 0;
static const int y_start_   = x_start_   + NMPC_T;
static const int yaw_start_ = y_start_   + NMPC_T;
static const int v1_start_  = yaw_start_ + NMPC_T;
static const int v2_start_  = v1_start_  + (NMPC_T - 1);
static const int v3_start_  = v2_start_  + (NMPC_T - 1);
static const int v4_start_  = v3_start_  + (NMPC_T - 1);
static const int d1_start_  = v4_start_  + (NMPC_T - 1);
static const int d2_start_  = d1_start_  + (NMPC_T - 1);
static const int d3_start_  = d2_start_  + (NMPC_T - 1);
static const int d4_start_  = d3_start_  + (NMPC_T - 1);

// ============== NMPC 代价/约束函数对象 (供 CppAD::ipopt::solve 使用) ==============
class FG_EVAL_DIFF {
public:
    M_XREF_DIFF traj_ref;     // 参考轨迹 [x; y; yaw; v_ref] x T
    Eigen::VectorXd U_prev;   // 上一时刻实际控制量(8维)，用于控制量平滑代价
    double L, W;
    // 构造函数声明
    FG_EVAL_DIFF(const M_XREF_DIFF &trajRef, const Eigen::VectorXd &uPrev, double L_, double W_);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    // 这是函数调用运算符重载。它让一个 FG_EVAL_DIFF 对象可以像函数一样调用
    // fg 是输出参数    vars 是输入决策变量
    void operator()(ADvector &fg, const ADvector &vars);
};

class diffMpcController {
public:
    Eigen::VectorXd U;      // 记录上一时刻实际控制量
    int target_ind_state = 0;  // 预测时域目标索引，改为成员变量以便新路径重置

    diffMpcController() : U(Eigen::VectorXd::Zero(NMPC_NU)) {}

    // 重置内部状态，应在收到新路径时调用
    void reset() {
        U.setZero();
        target_ind_state = 0;
    }

    // 计算参考点的速度（与曲率相关，原样保留）
    std::vector<double> calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed);
    // 平缓航向角
    void smooth_yaw(std::vector<double>& cyaw);
    // 计算最近点
    std::tuple<int, double> calc_ref_trajectory(double current_x,
                                                double current_y,
                                                const std::vector<double>& cx,
                                                const std::vector<double>& cy) const;
    // 构建预测时域内的参考轨迹 [x,y,yaw] x T（NMPC专用，按弧长步进）
    M_XREF_DIFF build_horizon_reference(int min_index,
                                         const std::vector<double>& cx,
                                         const std::vector<double>& cy,
                                         const std::vector<double>& cyaw,
                                         const std::vector<double>& speed,
                                         double current_v,
                                         int& target_ind);

    // NMPC 求解器：返回本周期要执行的第一组 8 维控制量。
    Eigen::VectorXd mpc_solve(const std::vector<double>& cx,
                               const std::vector<double>& cy,
                               const std::vector<double>& cyaw,
                               const std::vector<double>& speed,
                               const Eigen::Vector3d& initial_x,
                               int min_index,
                               double reference_speed,
                               const parameters& params_,
                               const Eigen::VectorXd& applied_control);
};

#endif
