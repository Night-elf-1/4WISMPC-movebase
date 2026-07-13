#ifndef MPC_LOCAL_PLANNER_CONTROL_LIMITS_HPP
#define MPC_LOCAL_PLANNER_CONTROL_LIMITS_HPP

#include <Eigen/Core>

namespace mpc_local_planner
{

struct ControlLimits
{
    double max_wheel_speed = 0.8;          // wheel linear speed [m/s]
    double max_wheel_acceleration = 1.0;   // wheel linear acceleration [m/s^2]
    double max_steering_angle = 0.8726646259971648; // 50 deg [rad]
    double max_steering_rate = 0.5;        // steering rate [rad/s]
};

bool areControlLimitsValid(const ControlLimits& limits);

// Applies absolute and per-cycle slew limits in place. Both vectors use
// [v_fl, v_rl, v_rr, v_fr, steer_fl, steer_rl, steer_rr, steer_fr].
bool applyControlLimits(Eigen::VectorXd& command,
                        const Eigen::VectorXd& previous_command,
                        double elapsed_seconds,
                        const ControlLimits& limits);

} // namespace mpc_local_planner

#endif // MPC_LOCAL_PLANNER_CONTROL_LIMITS_HPP
