//
// 非线性MPC (NMPC) 版本 —— 四轮独立转向/驱动 AGV 路径跟踪
// 由线性化(QP/OSQP)版本改写为 CppAD + IPOPT 非线性求解版本，
// 框架对齐 ModelPredictiveControl.h/.cpp (NMPC单车模型) 的写法。
//
#ifndef DIFFMPC_HPP
#define DIFFMPC_HPP

#include <iostream>
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "cubic_spline.hpp"

using CppAD::AD;
using namespace std;

// ============== 可调参数 ==============
// 注意：非线性求解比 QP 慢得多，预测步长/控制步长建议比线性版本(NP=30,NC=5)小很多，
// 这里采用 T 个预测步、(T-1) 个控制步（与控制步长合并，不再单独区分 NC）。
#define NMPC_NX 3        // 状态量维度 [x, y, yaw]
#define NMPC_NU 8        // 控制量维度 [v1, v2, v3, v4, d1, d2, d3, d4]
#define NMPC_T 10         // 预测步长（horizon）
#define NMPC_DT 0.1       // 采样时间

#define NMPC_V_MAX 5.0
#define NMPC_V_MIN -5.0
// 转向角范围收紧：±90° 在数值上太宽，容易让 IPOPT 找到"原地打转"式的病态解，
// 改成 ±50° 左右，足够覆盖正常转弯需求，又能避免极端解。
#define NMPC_DELTA_MAX (50.0/180.0*M_PI)
#define NMPC_DELTA_MIN (-50.0/180.0*M_PI)
#define NMPC_DV_MAX 1.0       // 单步轮速最大变化（硬约束）
#define NMPC_DDELTA_MAX 0.15  // 单步转角最大变化（硬约束，弧度）

#define N_IND_SEARCH 10

static inline bool finish = true;

struct parameters {
    double L = 0.4615;       // 车辆中心到前后轴的距离 (Half wheelbase)，与 smart.xacro 中 front_tyre_x 一致
    double W = 0.4;          // 车辆中心到左右轮的距离 (Half track width)
    int NX = NMPC_NX;
    int NU = NMPC_NU;
    int NP = NMPC_T;         // 预测步长（兼容旧字段名）
    int NC = NMPC_T - 1;     // 控制步长（兼容旧字段名）
    double dt = NMPC_DT;
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

class KinematicModel_MPC {
public:
    double x, y, yaw, v, L, W, dt;
    Eigen::VectorXd U;      // 上一时刻实际控制量(8维)

    KinematicModel_MPC(double x, double y, double yaw, double v, double L, double W, double dt)
        : x(x), y(y), yaw(yaw), v(v), L(L), W(W), dt(dt) {
        U = Eigen::VectorXd::Zero(8);
    };
    ~KinematicModel_MPC() {};

    void updatestate(Eigen::VectorXd U_cmd);
    std::tuple<double, double, double, double> getstate();
};

// ============== NMPC 代价/约束函数对象 (供 CppAD::ipopt::solve 使用) ==============
class FG_EVAL_DIFF {
public:
    M_XREF_DIFF traj_ref;     // 参考轨迹 [x;y;yaw] x T
    Eigen::VectorXd U_prev;   // 上一时刻实际控制量(8维)，用于控制量平滑代价
    double L, W;

    FG_EVAL_DIFF(const M_XREF_DIFF &trajRef, const Eigen::VectorXd &uPrev, double L_, double W_);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars);
};

class diffMpcController {
public:
    int NX, NU, NP, NC;
    Eigen::VectorXd U;      // 记录上一时刻实际控制量

    diffMpcController(int nx, int nu, int np, int nc) : NX(nx), NU(nu), NP(np), NC(nc) {
        U = Eigen::VectorXd::Zero(nu);
    };
    ~diffMpcController() {};

    // 计算参考点的速度（与曲率相关，原样保留）
    std::vector<double> calculateReferenceSpeeds(const std::vector<double>& curvatures, const double& max_speed);
    // 平缓航向角
    void smooth_yaw(std::vector<double>& cyaw);
    // 计算最近点
    std::tuple<int, double> calc_ref_trajectory(double current_x, double current_y, std::vector<double> cx, std::vector<double> cy, std::vector<double> cyaw);
    // 构建预测时域内的参考轨迹 [x,y,yaw] x T（NMPC专用，按弧长步进）
    M_XREF_DIFF build_horizon_reference(int min_index, std::vector<double>& cx, std::vector<double>& cy,
                                         std::vector<double>& cyaw, std::vector<double>& speed, double dl,
                                         double current_v, int &target_ind);

    // NMPC 求解器：返回未来 T-1 步的全部控制量（拼接成一个向量），调用方取第一组 8 维控制量执行
    Eigen::VectorXd mpc_solve(std::vector<double>& cx, std::vector<double>& cy, std::vector<double>& cyaw,
                               std::vector<double>& ck, std::vector<double>& speed, Eigen::Vector3d inital_x,
                               int min_index, double min_errors, KinematicModel_MPC agv_model, parameters params_);
};

#endif