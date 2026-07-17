#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "mpc_local_planner/nmpc_core/diffmpc.hpp"

namespace
{

TEST(DiffMpcReference, CurvatureSpeedGainIsConfigurable)
{
    diffMpcController controller;
    const auto speed = controller.calculateReferenceSpeeds({0.0, 0.2, 5.0}, 0.8, 2.0, 0.2);

    ASSERT_EQ(speed.size(), 3U);
    EXPECT_DOUBLE_EQ(speed[0], 0.8);
    EXPECT_NEAR(speed[1], 0.8 / 1.4, 1e-12);
    EXPECT_DOUBLE_EQ(speed[2], 0.2);
}

TEST(DiffMpcCost, ConfiguredPositionWeightChangesObjective)
{
    constexpr size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
    constexpr size_t n_constraints =
        NMPC_T * 3 + 8 * (NMPC_T - 1) + 5 * (NMPC_T - 1);
    FG_EVAL_DIFF::ADvector vars(n_vars);
    FG_EVAL_DIFF::ADvector fg(1 + n_constraints);
    for (size_t i = 0; i < n_vars; ++i)
    {
        vars[i] = 0.0;
    }

    M_XREF_DIFF reference = M_XREF_DIFF::Zero();
    reference.row(0).setOnes();
    MpcCostWeights weights;
    weights.position = 3.0;
    weights.yaw = 0.0;
    weights.speed = 0.0;
    weights.wheel_speed = 0.0;
    weights.steering = 0.0;
    weights.wheel_speed_change = 0.0;
    weights.steering_change = 0.0;
    FG_EVAL_DIFF evaluator(
        reference, Eigen::VectorXd::Zero(NMPC_NU), 0.4615, 0.4, weights);

    evaluator(fg, vars);

    EXPECT_NEAR(CppAD::Value(fg[0]), 3.0 * (NMPC_T - 1), 1e-12);
}

TEST(DiffMpcCost, WheelAndSteeringWeightsAreIndependent)
{
    constexpr size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
    constexpr size_t n_constraints =
        NMPC_T * 3 + 8 * (NMPC_T - 1) + 5 * (NMPC_T - 1);
    FG_EVAL_DIFF::ADvector vars(n_vars);
    FG_EVAL_DIFF::ADvector fg(1 + n_constraints);
    for (size_t i = 0; i < n_vars; ++i)
    {
        vars[i] = 0.0;
    }
    vars[v1_start_] = 1.0;
    vars[d1_start_] = 1.0;

    MpcCostWeights weights;
    weights.position = 0.0;
    weights.yaw = 0.0;
    weights.speed = 0.0;
    weights.wheel_speed = 2.0;
    weights.steering = 3.0;
    weights.wheel_speed_change = 0.0;
    weights.steering_change = 0.0;
    FG_EVAL_DIFF evaluator(
        M_XREF_DIFF::Zero(), Eigen::VectorXd::Zero(NMPC_NU), 0.4615, 0.4, weights);

    evaluator(fg, vars);

    EXPECT_NEAR(CppAD::Value(fg[0]), 5.0, 1e-12);
}

TEST(DiffMpcReference, InterpolatesUsingActualArcLength)
{
    diffMpcController controller;
    const std::vector<double> x{0.0, 0.05, 0.20, 0.55, 1.0};
    const std::vector<double> y(x.size(), 0.0);
    const std::vector<double> yaw(x.size(), 0.0);
    const std::vector<double> speed(x.size(), 0.5);
    int target_index = 0;

    const M_XREF_DIFF reference = controller.build_horizon_reference(
        0, x, y, yaw, speed, 0.5, target_index);

    for (int i = 0; i < NMPC_T; ++i)
    {
        EXPECT_NEAR(reference(0, i), 0.05 * i, 1e-9);
        EXPECT_NEAR(reference(1, i), 0.0, 1e-9);
    }
}

TEST(DiffMpcReference, HandlesDuplicatePointsAndPathEnd)
{
    diffMpcController controller;
    const std::vector<double> x{0.0, 0.0, 0.1};
    const std::vector<double> y(x.size(), 0.0);
    const std::vector<double> yaw(x.size(), 0.0);
    const std::vector<double> speed(x.size(), 1.0);
    int target_index = 0;

    const M_XREF_DIFF reference = controller.build_horizon_reference(
        0, x, y, yaw, speed, 1.0, target_index);

    EXPECT_NEAR(reference(0, 0), 0.0, 1e-9);
    EXPECT_NEAR(reference(0, 1), 0.1, 1e-9);
    EXPECT_NEAR(reference(0, NMPC_T - 1), 0.1, 1e-9);
}

TEST(DiffMpcReference, InterpolatesYawAcrossWrapBoundary)
{
    diffMpcController controller;
    const std::vector<double> x{0.0, 0.1};
    const std::vector<double> y(x.size(), 0.0);
    const std::vector<double> yaw{3.1, -3.1};
    const std::vector<double> speed(x.size(), 0.5);
    int target_index = 0;

    const M_XREF_DIFF reference = controller.build_horizon_reference(
        0, x, y, yaw, speed, 0.5, target_index);

    EXPECT_NEAR(std::fabs(reference(2, 1)), M_PI, 1e-6);
}

TEST(DiffMpcController, FindsNearestPointWithoutCopyOnlyInputs)
{
    diffMpcController controller;
    const std::vector<double> x{0.0, 1.0, 2.0};
    const std::vector<double> y{0.0, 0.0, 0.0};

    const auto [index, distance] = controller.calc_ref_trajectory(1.2, 0.5, x, y);

    EXPECT_EQ(index, 1);
    EXPECT_NEAR(distance, std::hypot(0.2, 0.5), 1e-12);
}

TEST(DiffMpcController, ResetClearsRuntimeState)
{
    diffMpcController controller;
    controller.U.setOnes();
    controller.target_ind_state = 7;

    controller.reset();

    EXPECT_TRUE(controller.U.isZero());
    EXPECT_EQ(controller.target_ind_state, 0);
}

TEST(DiffMpcController, HandlesEmptyNearestPointInput)
{
    diffMpcController controller;
    const std::vector<double> empty;

    const auto [index, distance] = controller.calc_ref_trajectory(0.0, 0.0, empty, empty);

    EXPECT_EQ(index, 0);
    EXPECT_EQ(distance, std::numeric_limits<double>::max());
}

TEST(DiffMpcConstraints, FirstControlStepUsesAppliedCommand)
{
    constexpr size_t n_vars = NMPC_T * 3 + (NMPC_T - 1) * 8;
    constexpr size_t n_constraints =
        NMPC_T * 3 + 8 * (NMPC_T - 1) + 5 * (NMPC_T - 1);
    FG_EVAL_DIFF::ADvector vars(n_vars);
    FG_EVAL_DIFF::ADvector fg(1 + n_constraints);
    for (size_t i = 0; i < n_vars; ++i)
    {
        vars[i] = 0.0;
    }
    for (size_t i = 0; i <= n_constraints; ++i)
    {
        fg[i] = 123.0;
    }

    Eigen::VectorXd previous = Eigen::VectorXd::Zero(8);
    previous(0) = 0.2;
    previous(4) = 0.4;
    FG_EVAL_DIFF evaluator(M_XREF_DIFF::Zero(), previous, 0.4615, 0.4);
    evaluator(fg, vars);

    constexpr size_t first_steering_rate_constraint = 1 + 3 * NMPC_T;
    constexpr size_t first_speed_rate_constraint =
        first_steering_rate_constraint + 4 * (NMPC_T - 1);
    EXPECT_NEAR(CppAD::Value(fg[first_steering_rate_constraint]), -0.4, 1e-12);
    EXPECT_NEAR(CppAD::Value(fg[first_speed_rate_constraint]), -0.2, 1e-12);
    EXPECT_NEAR(CppAD::Value(fg[n_constraints]), 0.0, 1e-12);
}

TEST(DiffMpcSolver, ProducesFiniteFirstCommandWithinSlewLimits)
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> speed;
    for (int i = 0; i <= 40; ++i)
    {
        x.push_back(0.05 * i);
        y.push_back(0.0);
        yaw.push_back(0.0);
        speed.push_back(0.5);
    }

    parameters params;
    params.control_limits.max_wheel_speed = 0.8;
    params.control_limits.max_wheel_acceleration = 1.0;
    params.control_limits.max_steering_angle = 0.8727;
    params.control_limits.max_steering_rate = 0.5;

    diffMpcController controller;
    const Eigen::VectorXd applied = Eigen::VectorXd::Zero(NMPC_NU);
    const Eigen::VectorXd command = controller.mpc_solve(
        x, y, yaw, speed, Eigen::Vector3d::Zero(), 0, 0.5, params, applied);

    ASSERT_EQ(command.size(), NMPC_NU);
    ASSERT_TRUE(command.allFinite());
    EXPECT_LE(command.head(4).cwiseAbs().maxCoeff(),
              params.control_limits.max_wheel_acceleration * params.dt + 1e-6);
    EXPECT_LE(command.tail(4).cwiseAbs().maxCoeff(),
              params.control_limits.max_steering_rate * params.dt + 1e-6);
}

} // namespace
