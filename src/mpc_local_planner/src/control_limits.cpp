#include "mpc_local_planner/control_limits.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_local_planner
{

namespace
{
constexpr Eigen::Index kControlSize = 8;
constexpr Eigen::Index kWheelCount = 4;
}

bool areControlLimitsValid(const ControlLimits& limits)
{
    return std::isfinite(limits.max_wheel_speed) && limits.max_wheel_speed > 0.0 &&
           std::isfinite(limits.max_wheel_acceleration) && limits.max_wheel_acceleration > 0.0 &&
           std::isfinite(limits.max_steering_angle) && limits.max_steering_angle > 0.0 &&
           std::isfinite(limits.max_steering_rate) && limits.max_steering_rate > 0.0;
}

bool applyControlLimits(Eigen::VectorXd& command,
                        const Eigen::VectorXd& previous_command,
                        double elapsed_seconds,
                        const ControlLimits& limits)
{
    if (command.size() != kControlSize || previous_command.size() != kControlSize ||
        !std::isfinite(elapsed_seconds) || elapsed_seconds <= 0.0 ||
        !areControlLimitsValid(limits))
    {
        return false;
    }

    for (Eigen::Index i = 0; i < kControlSize; ++i)
    {
        if (!std::isfinite(command(i)) || !std::isfinite(previous_command(i)))
        {
            return false;
        }
    }

    const double max_speed_step = limits.max_wheel_acceleration * elapsed_seconds;
    const double max_steering_step = limits.max_steering_rate * elapsed_seconds;

    for (Eigen::Index i = 0; i < kWheelCount; ++i)
    {
        const double previous = std::clamp(previous_command(i),
                                           -limits.max_wheel_speed,
                                           limits.max_wheel_speed);
        command(i) = std::clamp(command(i),
                                -limits.max_wheel_speed,
                                limits.max_wheel_speed);
        command(i) = std::clamp(command(i),
                                previous - max_speed_step,
                                previous + max_speed_step);
        command(i) = std::clamp(command(i),
                                -limits.max_wheel_speed,
                                limits.max_wheel_speed);
    }

    for (Eigen::Index i = kWheelCount; i < kControlSize; ++i)
    {
        const double previous = std::clamp(previous_command(i),
                                           -limits.max_steering_angle,
                                           limits.max_steering_angle);
        command(i) = std::clamp(command(i),
                                -limits.max_steering_angle,
                                limits.max_steering_angle);
        command(i) = std::clamp(command(i),
                                previous - max_steering_step,
                                previous + max_steering_step);
        command(i) = std::clamp(command(i),
                                -limits.max_steering_angle,
                                limits.max_steering_angle);
    }

    return true;
}

} // namespace mpc_local_planner
