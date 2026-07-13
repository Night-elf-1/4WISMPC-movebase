#include <gtest/gtest.h>

#include <limits>

#include "mpc_local_planner/control_limits.hpp"

namespace
{

TEST(ControlLimits, AppliesAbsoluteAndSlewLimits)
{
    mpc_local_planner::ControlLimits limits;
    limits.max_wheel_speed = 0.8;
    limits.max_wheel_acceleration = 1.0;
    limits.max_steering_angle = 0.8;
    limits.max_steering_rate = 0.5;

    Eigen::VectorXd previous = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd command(8);
    command << 2.0, -2.0, 0.5, -0.5, 1.2, -1.2, 0.4, -0.4;

    ASSERT_TRUE(mpc_local_planner::applyControlLimits(command, previous, 0.1, limits));
    EXPECT_DOUBLE_EQ(command(0), 0.1);
    EXPECT_DOUBLE_EQ(command(1), -0.1);
    EXPECT_DOUBLE_EQ(command(2), 0.1);
    EXPECT_DOUBLE_EQ(command(3), -0.1);
    EXPECT_DOUBLE_EQ(command(4), 0.05);
    EXPECT_DOUBLE_EQ(command(5), -0.05);
    EXPECT_DOUBLE_EQ(command(6), 0.05);
    EXPECT_DOUBLE_EQ(command(7), -0.05);
}

TEST(ControlLimits, NeverExceedsAbsoluteLimitNearBoundary)
{
    mpc_local_planner::ControlLimits limits;
    Eigen::VectorXd previous = Eigen::VectorXd::Zero(8);
    previous.head<4>().setConstant(0.79);
    previous.tail<4>().setConstant(0.86);
    Eigen::VectorXd command = Eigen::VectorXd::Constant(8, 10.0);

    ASSERT_TRUE(mpc_local_planner::applyControlLimits(command, previous, 0.1, limits));
    EXPECT_LE(command.head<4>().maxCoeff(), limits.max_wheel_speed);
    EXPECT_LE(command.tail<4>().maxCoeff(), limits.max_steering_angle);
}

TEST(ControlLimits, RejectsInvalidCommand)
{
    mpc_local_planner::ControlLimits limits;
    Eigen::VectorXd previous = Eigen::VectorXd::Zero(8);
    Eigen::VectorXd command = Eigen::VectorXd::Zero(8);
    command(0) = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(mpc_local_planner::applyControlLimits(command, previous, 0.1, limits));
    EXPECT_FALSE(mpc_local_planner::applyControlLimits(command, previous, 0.0, limits));
}

} // namespace
