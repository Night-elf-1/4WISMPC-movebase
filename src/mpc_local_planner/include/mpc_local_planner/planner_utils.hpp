#ifndef MPC_LOCAL_PLANNER_PLANNER_UTILS_HPP
#define MPC_LOCAL_PLANNER_PLANNER_UTILS_HPP

#include <string>
#include <vector>

namespace mpc_local_planner
{

struct GoalPose2D
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct GoalEvaluation
{
    double distance = 0.0;
    double yaw_error = 0.0;
    bool valid = false;
    bool position_reached = false;
    bool yaw_reached = false;

    bool reached() const
    {
        return valid && position_reached && yaw_reached;
    }
};

enum class GoalYawMode
{
    AUTO,
    POSE_ORIENTATION,
    PATH_TANGENT
};

struct GoalYawSelection
{
    double yaw = 0.0;
    bool valid = false;
    bool used_pose_orientation = false;
};

GoalEvaluation evaluateGoal(double current_x,
                            double current_y,
                            double current_yaw,
                            const GoalPose2D& goal,
                            double xy_tolerance,
                            double yaw_tolerance);

bool canReportGoalReached(const GoalEvaluation& goal, bool steering_recentered);

double stepTowardZero(double value, double max_step);

bool parseGoalYawMode(const std::string& value, GoalYawMode& mode);

const char* goalYawModeName(GoalYawMode mode);

GoalYawSelection selectGoalYaw(GoalYawMode mode,
                               double pose_yaw,
                               bool terminal_path_yaw_valid,
                               double terminal_path_yaw);

std::vector<double> movingAverage(const std::vector<double>& data, int window);

std::vector<double> calculatePathCurvatures(const std::vector<double>& path_x,
                                            const std::vector<double>& path_y,
                                            const std::vector<double>& path_yaw);

bool enforceSpeedProfileAccelerationLimits(const std::vector<double>& path_x,
                                           const std::vector<double>& path_y,
                                           double max_acceleration,
                                           double max_deceleration,
                                           std::vector<double>& speed_profile);

bool tryCalculateTerminalPathYaw(const std::vector<double>& path_x,
                                 const std::vector<double>& path_y,
                                 double lookback_distance,
                                 double& yaw);

double calculateTerminalPathYaw(const std::vector<double>& path_x,
                                const std::vector<double>& path_y,
                                double lookback_distance,
                                double fallback_yaw);

} // namespace mpc_local_planner

#endif // MPC_LOCAL_PLANNER_PLANNER_UTILS_HPP
