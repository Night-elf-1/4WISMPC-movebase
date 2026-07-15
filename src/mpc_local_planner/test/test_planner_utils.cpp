#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mpc_local_planner/planner_utils.hpp"

namespace
{

using mpc_local_planner::GoalPose2D;
using mpc_local_planner::canReportGoalReached;
using mpc_local_planner::calculateTerminalPathYaw;
using mpc_local_planner::evaluateGoal;
using mpc_local_planner::movingAverage;
using mpc_local_planner::stepTowardZero;

TEST(PlannerGoalLogic, ReachedUsesTrueGoalPoseAcrossAllSmoothingWindows)
{
    const std::vector<double> raw_x{0.0, 0.5, 1.0, 1.5, 2.0, 2.01, 2.0};
    const std::vector<double> raw_y(raw_x.size(), 0.0);
    const GoalPose2D goal{raw_x.back(), raw_y.back(), 0.0};

    for (int window = 1; window <= 9; ++window)
    {
        std::vector<double> path_x = movingAverage(raw_x, window);
        std::vector<double> path_y = movingAverage(raw_y, window);
        path_x.front() = raw_x.front();
        path_y.front() = raw_y.front();
        path_x.back() = raw_x.back();
        path_y.back() = raw_y.back();

        const double terminal_path_yaw =
            calculateTerminalPathYaw(path_x, path_y, 0.2, M_PI);
        EXPECT_NEAR(terminal_path_yaw, 0.0, 1e-12) << "window=" << window;

        const auto evaluation =
            evaluateGoal(goal.x, goal.y, goal.yaw, goal, 0.1, 0.2);
        EXPECT_TRUE(evaluation.reached()) << "window=" << window;
    }
}

TEST(PlannerGoalLogic, DuplicateEndpointDoesNotReplaceTrueGoalYaw)
{
    const std::vector<double> path_x{0.0, 1.0, 2.0, 2.0};
    const std::vector<double> path_y(path_x.size(), 0.0);
    const GoalPose2D goal{2.0, 0.0, M_PI_2};

    EXPECT_NEAR(calculateTerminalPathYaw(path_x, path_y, 0.2, M_PI), 0.0, 1e-12);
    EXPECT_TRUE(evaluateGoal(2.0, 0.0, M_PI_2, goal, 0.1, 0.2).reached());

    const auto needs_alignment = evaluateGoal(2.0, 0.0, 0.0, goal, 0.1, 0.2);
    EXPECT_TRUE(needs_alignment.position_reached);
    EXPECT_FALSE(needs_alignment.yaw_reached);
    EXPECT_FALSE(needs_alignment.reached());
}

TEST(PlannerGoalLogic, UsesShortestYawErrorAcrossWrapBoundary)
{
    const GoalPose2D goal{0.0, 0.0, -M_PI + 0.02};
    const auto evaluation =
        evaluateGoal(0.0, 0.0, M_PI - 0.02, goal, 0.1, 0.05);

    EXPECT_NEAR(evaluation.yaw_error, 0.04, 1e-12);
    EXPECT_TRUE(evaluation.reached());
}

TEST(PlannerGoalLogic, PositionAndYawTolerancesRemainIndependent)
{
    const GoalPose2D goal{1.0, 2.0, 0.5};

    const auto outside_position = evaluateGoal(1.11, 2.0, 0.5, goal, 0.1, 0.2);
    EXPECT_FALSE(outside_position.position_reached);
    EXPECT_TRUE(outside_position.yaw_reached);
    EXPECT_FALSE(outside_position.reached());

    const auto outside_yaw = evaluateGoal(1.0, 2.0, 0.71, goal, 0.1, 0.2);
    EXPECT_TRUE(outside_yaw.position_reached);
    EXPECT_FALSE(outside_yaw.yaw_reached);
    EXPECT_FALSE(outside_yaw.reached());
}

TEST(PlannerGoalLogic, GoalCompletionWaitsForSteeringRecenter)
{
    const GoalPose2D goal{1.0, 2.0, 0.5};
    const auto reached = evaluateGoal(1.0, 2.0, 0.5, goal, 0.1, 0.2);

    EXPECT_FALSE(canReportGoalReached(reached, false));
    EXPECT_TRUE(canReportGoalReached(reached, true));

    const auto outside_position = evaluateGoal(1.2, 2.0, 0.5, goal, 0.1, 0.2);
    EXPECT_FALSE(canReportGoalReached(outside_position, true));
}

TEST(PlannerGoalLogic, SteeringRecenterIsBoundedAndEndsAtExactZero)
{
    std::vector<double> steering{0.856, -0.868, 0.023, -0.004};
    constexpr double step = 0.05;

    for (int cycle = 0; cycle < 20; ++cycle)
    {
        for (double& angle : steering)
        {
            const double previous = angle;
            angle = stepTowardZero(angle, step);
            EXPECT_LE(std::fabs(angle), std::fabs(previous));
            EXPECT_LE(std::fabs(angle - previous), step + 1e-12);
            EXPECT_GE(angle * previous, 0.0);
        }
    }

    for (double angle : steering)
    {
        EXPECT_DOUBLE_EQ(angle, 0.0);
    }
}

} // namespace
