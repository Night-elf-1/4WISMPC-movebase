#include "nmpc_4wisnew2/diffmpc.hpp"
#include <chrono>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

int main(int argc, char const *argv[])
{
    // 生成参考路线
    vector<double> wx({10.0, 60.0, 125.0,  50.0,   60.0,  35.0,  -10.0});
    vector<double> wy({0.0,  0.0,  50.0,  65.0,   45.0,  50.0,  -20.0});

    Spline2D csp_obj(wx, wy);
    vector<double> r_x;
    vector<double> r_y;
    vector<double> ryaw;
    vector<double> rcurvature;
    vector<double> rs;

    for (double i = 0; i < csp_obj.s.back(); i += 1.0) {
        vector<double> point_ = csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }
    double target_speed = 3.5;

    // 初始化参数
    parameters param;

    // NMPC 控制器：内部使用 CppAD + IPOPT 求解非线性优化问题
    diffMpcController mpc(param.NX, param.NU, param.NP, param.NC);
    vector<double> speed_profile = mpc.calculateReferenceSpeeds(rcurvature, target_speed);
    mpc.smooth_yaw(ryaw);

    // 初始化agv初始状态 x, y, yaw
    Eigen::Vector3d initial_x;
    initial_x << 10.0, 5.0, 0.1;

    KinematicModel_MPC agv(initial_x(0), initial_x(1), initial_x(2), target_speed, param.L, param.W, param.dt);

    std::vector<double> x_history, y_history;
    plt::figure_size(800, 600);

    while (finish)
    {
        // 计算最近点
        auto [min_index, min_e] = mpc.calc_ref_trajectory(initial_x(0), initial_x(1), r_x, r_y, ryaw);
        cout << "最近点索引: " << min_index << ", 最近点误差: " << min_e << endl;

        // NMPC 求解，直接返回非线性优化得到的本周期实际控制量 [v1,v2,v3,v4,d1,d2,d3,d4]
        Eigen::VectorXd U_cmd = mpc.mpc_solve(r_x, r_y, ryaw, rcurvature, speed_profile, initial_x, min_index, min_e, agv, param);

        agv.updatestate(U_cmd);
        auto [current_x, current_y, current_yaw, current_v] = agv.getstate();
        std::cout << "当前速度: " << current_v << ", 参考速度: " << speed_profile[min_index] << std::endl;
        initial_x << current_x, current_y, current_yaw;

        x_history.push_back(current_x);
        y_history.push_back(current_y);

        // 绘图
        plt::clf();
        plt::plot(r_x, r_y, "b--");
        plt::plot(x_history, y_history, "r-");
        plt::scatter(std::vector<double>{initial_x(0)}, std::vector<double>{initial_x(1)}, 20.0, {{"color", "green"}});
        plt::pause(0.01);

        if (min_index >= (int)r_x.size() - 15)
        {
            finish = false;
            std::cout << "仿真结束!" << std::endl;
            std::cout << "航向角误差：" << current_yaw - ryaw[min_index] << std::endl;
            std::cout << "横向误差：" << min_e << std::endl;
        }
    }

    plt::show();
    return 0;
}