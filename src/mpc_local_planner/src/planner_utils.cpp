#include "mpc_local_planner/planner_utils.hpp"

#include <algorithm>
#include <cctype>
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

bool parseGoalYawMode(const std::string& value, GoalYawMode& mode)
{
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char character) {
                       return static_cast<char>(std::tolower(character));
                   });

    if (normalized == "auto")
    {
        mode = GoalYawMode::AUTO;
        return true;
    }
    if (normalized == "pose" || normalized == "pose_orientation")
    {
        mode = GoalYawMode::POSE_ORIENTATION;
        return true;
    }
    if (normalized == "path" || normalized == "path_tangent")
    {
        mode = GoalYawMode::PATH_TANGENT;
        return true;
    }
    return false;
}

const char* goalYawModeName(GoalYawMode mode)
{
    switch (mode)
    {
        case GoalYawMode::AUTO:
            return "auto";
        case GoalYawMode::POSE_ORIENTATION:
            return "pose_orientation";
        case GoalYawMode::PATH_TANGENT:
            return "path_tangent";
    }
    return "auto";
}

GoalYawSelection selectGoalYaw(GoalYawMode mode,
                               double pose_yaw,
                               bool terminal_path_yaw_valid,
                               double terminal_path_yaw)
{
    const bool pose_yaw_valid = std::isfinite(pose_yaw);
    if (mode != GoalYawMode::PATH_TANGENT && pose_yaw_valid)
    {
        return GoalYawSelection{pose_yaw, true, true};
    }
    if (mode == GoalYawMode::POSE_ORIENTATION)
    {
        return GoalYawSelection();
    }
    if (terminal_path_yaw_valid && std::isfinite(terminal_path_yaw))
    {
        return GoalYawSelection{terminal_path_yaw, true, false};
    }
    return GoalYawSelection();
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
    double yaw = 0.0;
    if (tryCalculateTerminalPathYaw(path_x, path_y, lookback_distance, yaw))
    {
        return yaw;
    }
    return std::isfinite(fallback_yaw) ? fallback_yaw : 0.0;
}

bool tryCalculateTerminalPathYaw(const std::vector<double>& path_x,
                                 const std::vector<double>& path_y,
                                 double lookback_distance,
                                 double& yaw)
{
    constexpr double kMinChordLength = 1e-6;
    const size_t point_count = std::min(path_x.size(), path_y.size());
    if (point_count < 2)
    {
        return false;
    }

    const size_t last = point_count - 1;
    const double goal_x = path_x[last];
    const double goal_y = path_y[last];
    if (!std::isfinite(goal_x) || !std::isfinite(goal_y))
    {
        return false;
    }

    const double required_chord =
        std::isfinite(lookback_distance)
            ? std::max(lookback_distance, kMinChordLength)
            : kMinChordLength;
    bool found_direction = false;
    double candidate = 0.0;
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
        found_direction = true;
        if (chord >= required_chord)
        {
            break;
        }
    }
    if (!found_direction)
    {
        return false;
    }
    yaw = candidate;
    return true;
}

} // namespace mpc_local_planner
