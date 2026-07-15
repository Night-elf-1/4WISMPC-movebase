#ifndef MPC_LOCAL_PLANNER_PLANNER_UTILS_HPP
#define MPC_LOCAL_PLANNER_PLANNER_UTILS_HPP

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

GoalEvaluation evaluateGoal(double current_x,
                            double current_y,
                            double current_yaw,
                            const GoalPose2D& goal,
                            double xy_tolerance,
                            double yaw_tolerance);

bool canReportGoalReached(const GoalEvaluation& goal, bool steering_recentered);

double stepTowardZero(double value, double max_step);

std::vector<double> movingAverage(const std::vector<double>& data, int window);

double calculateTerminalPathYaw(const std::vector<double>& path_x,
                                const std::vector<double>& path_y,
                                double lookback_distance,
                                double fallback_yaw);

} // namespace mpc_local_planner

#endif // MPC_LOCAL_PLANNER_PLANNER_UTILS_HPP
