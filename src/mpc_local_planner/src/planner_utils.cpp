#include "mpc_local_planner/planner_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mpc_local_planner
{

GoalEvaluation evaluateGoal(double current_x,
                            double current_y,
                            double current_yaw,
                            const GoalPose2D& goal,
                            double xy_tolerance,
                            double yaw_tolerance)
{
    GoalEvaluation result;
    if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
        !std::isfinite(current_yaw) || !std::isfinite(goal.x) ||
        !std::isfinite(goal.y) || !std::isfinite(goal.yaw) ||
        !std::isfinite(xy_tolerance) || xy_tolerance < 0.0 ||
        !std::isfinite(yaw_tolerance) || yaw_tolerance < 0.0)
    {
        result.distance = std::numeric_limits<double>::infinity();
        result.yaw_error = std::numeric_limits<double>::infinity();
        return result;
    }

    result.distance = std::hypot(current_x - goal.x, current_y - goal.y);
    result.yaw_error = std::atan2(std::sin(goal.yaw - current_yaw),
                                  std::cos(goal.yaw - current_yaw));
    result.valid = true;
    result.position_reached = result.distance <= xy_tolerance;
    result.yaw_reached = std::fabs(result.yaw_error) <= yaw_tolerance;
    return result;
}

bool canReportGoalReached(const GoalEvaluation& goal, bool steering_recentered)
{
    return goal.reached() && steering_recentered;
}

double stepTowardZero(double value, double max_step)
{
    if (!std::isfinite(value) || !std::isfinite(max_step) || max_step <= 0.0)
    {
        return value;
    }
    if (value > max_step)
    {
        return value - max_step;
    }
    if (value < -max_step)
    {
        return value + max_step;
    }
    return 0.0;
}

std::vector<double> movingAverage(const std::vector<double>& data, int window)
{
    std::vector<double> smoothed = data;
    if (window <= 1 || data.empty())
    {
        return smoothed;
    }

    // A centered average needs an odd sample count. Preserve the existing
    // behavior by expanding even values to the next odd width (2 -> 3, etc.).
    const int half = window / 2;
    for (size_t i = 0; i < data.size(); ++i)
    {
        double sum = 0.0;
        int count = 0;
        for (int offset = -half; offset <= half; ++offset)
        {
            const int index = static_cast<int>(i) + offset;
            if (index >= 0 && index < static_cast<int>(data.size()))
            {
                sum += data[index];
                ++count;
            }
        }
        if (count > 0)
        {
            smoothed[i] = sum / static_cast<double>(count);
        }
    }
    return smoothed;
}

double calculateTerminalPathYaw(const std::vector<double>& path_x,
                                const std::vector<double>& path_y,
                                double lookback_distance,
                                double fallback_yaw)
{
    constexpr double kMinChordLength = 1e-6;
    const size_t point_count = std::min(path_x.size(), path_y.size());
    double candidate = std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
    if (point_count < 2)
    {
        return candidate;
    }

    const size_t last = point_count - 1;
    const double goal_x = path_x[last];
    const double goal_y = path_y[last];
    if (!std::isfinite(goal_x) || !std::isfinite(goal_y))
    {
        return candidate;
    }

    const double required_chord =
        std::isfinite(lookback_distance)
            ? std::max(lookback_distance, kMinChordLength)
            : kMinChordLength;
    for (size_t index = last; index-- > 0;)
    {
        if (!std::isfinite(path_x[index]) || !std::isfinite(path_y[index]))
        {
            continue;
        }

        const double dx = goal_x - path_x[index];
        const double dy = goal_y - path_y[index];
        const double chord = std::hypot(dx, dy);
        if (chord <= kMinChordLength)
        {
            continue;
        }

        candidate = std::atan2(dy, dx);
        if (chord >= required_chord)
        {
            break;
        }
    }
    return candidate;
}

} // namespace mpc_local_planner
