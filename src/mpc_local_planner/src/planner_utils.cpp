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

std::vector<double> calculatePathCurvatures(const std::vector<double>& path_x,
                                            const std::vector<double>& path_y,
                                            const std::vector<double>& path_yaw)
{
    constexpr double kMinArcLength = 1e-6;
    const size_t point_count = std::min({path_x.size(), path_y.size(), path_yaw.size()});
    std::vector<double> curvature(point_count, 0.0);
    if (point_count < 2)
    {
        return curvature;
    }

    auto segmentLength = [&](size_t first, size_t second) {
        return std::hypot(path_x[second] - path_x[first],
                          path_y[second] - path_y[first]);
    };
    auto yawDifference = [&](size_t first, size_t second) {
        const double difference = path_yaw[second] - path_yaw[first];
        return std::atan2(std::sin(difference), std::cos(difference));
    };

    if (point_count == 2)
    {
        const double arc_length = segmentLength(0, 1);
        if (std::isfinite(arc_length) && arc_length > kMinArcLength)
        {
            const double value = yawDifference(0, 1) / arc_length;
            if (std::isfinite(value))
            {
                curvature[0] = value;
                curvature[1] = value;
            }
        }
        return curvature;
    }

    for (size_t i = 1; i + 1 < point_count; ++i)
    {
        const double arc_length = segmentLength(i - 1, i) + segmentLength(i, i + 1);
        if (!std::isfinite(arc_length) || arc_length <= kMinArcLength)
        {
            continue;
        }

        const double value = yawDifference(i - 1, i + 1) / arc_length;
        if (std::isfinite(value))
        {
            curvature[i] = value;
        }
    }

    // A one-sided yaw derivative is much noisier. Extending the nearest
    // centered estimate also avoids artificial zero-curvature speed spikes.
    curvature.front() = curvature[1];
    curvature.back() = curvature[point_count - 2];
    return curvature;
}

bool enforceSpeedProfileAccelerationLimits(const std::vector<double>& path_x,
                                           const std::vector<double>& path_y,
                                           double max_acceleration,
                                           double max_deceleration,
                                           std::vector<double>& speed_profile)
{
    if (path_x.size() != path_y.size() || path_x.size() != speed_profile.size() ||
        !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
        !std::isfinite(max_deceleration) || max_deceleration <= 0.0)
    {
        return false;
    }

    for (size_t i = 0; i < speed_profile.size(); ++i)
    {
        if (!std::isfinite(path_x[i]) || !std::isfinite(path_y[i]) ||
            !std::isfinite(speed_profile[i]) || speed_profile[i] < 0.0)
        {
            return false;
        }
    }
    if (speed_profile.size() < 2)
    {
        return true;
    }

    for (size_t i = 1; i < speed_profile.size(); ++i)
    {
        const double distance = std::hypot(path_x[i] - path_x[i - 1],
                                           path_y[i] - path_y[i - 1]);
        const double reachable_speed = std::sqrt(
            speed_profile[i - 1] * speed_profile[i - 1] +
            2.0 * max_acceleration * distance);
        speed_profile[i] = std::min(speed_profile[i], reachable_speed);
    }

    for (size_t i = speed_profile.size(); i-- > 1;)
    {
        const double distance = std::hypot(path_x[i] - path_x[i - 1],
                                           path_y[i] - path_y[i - 1]);
        const double reachable_speed = std::sqrt(
            speed_profile[i] * speed_profile[i] +
            2.0 * max_deceleration * distance);
        speed_profile[i - 1] = std::min(speed_profile[i - 1], reachable_speed);
    }

    return true;
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

// 估计“路径终点处的行驶方向”   lookback_distance希望从终点回看的直线距离
bool tryCalculateTerminalPathYaw(const std::vector<double>& path_x,
                                 const std::vector<double>& path_y,
                                 double lookback_distance,
                                 double& yaw)
{
    constexpr double kMinChordLength = 1e-6;    // 直线弦长的最小值，避免除以零或过小的数值
    // 只使用两个数组都存在的公共部分，避免越界
    const size_t point_count = std::min(path_x.size(), path_y.size());
    if (point_count < 2)
    {
        return false;
    }
    // 把路径最后一个有效索引当作终点。终点坐标如果是 NaN 或无穷大，直接失败
    const size_t last = point_count - 1;
    const double goal_x = path_x[last];
    const double goal_y = path_y[last];
    if (!std::isfinite(goal_x) || !std::isfinite(goal_y))
    {
        return false;
    }
    // 计算实际回看阈值
    const double required_chord =
        std::isfinite(lookback_distance)
            ? std::max(lookback_distance, kMinChordLength)
            : kMinChordLength;
    bool found_direction = false;
    double candidate = 0.0;
    // 反向遍历路径
    for (size_t index = last; index-- > 0;)
    {
        // 如果路径点无效，直接跳过
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
        // 计算候选方向 atan2(dy, dx) 得到从候选点指向终点的方向，范围通常为 [-π, π]
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
