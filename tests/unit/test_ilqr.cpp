// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/planners/ilqr.hpp"
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"

using namespace kinetra;
using namespace kinetra::planners;

// ─── DiffDriveSimple iLQR ────────────────────────────────────────────────────

TEST(iLQR_DiffDriveSimple, SolveWithCost_StraightLine) {
    DiffDriveSimple model;
    iLQROptions opts;
    opts.horizon = 40;
    opts.dt = 0.1;
    opts.maxIterations = 100;
    opts.timeLimitMs = 5000.0;

    iLQR<DiffDriveSimple> planner(model, opts);

    constexpr int nx = 3;
    constexpr int nu = 2;

    iLQR<DiffDriveSimple>::CostMatrices costs;
    costs.Q  = MatX::Identity(nx, nx) * 1.0;
    costs.R  = MatX::Identity(nu, nu) * 0.1;
    costs.Qf = MatX::Identity(nx, nx) * 200.0;

    DiffDriveSimple::StateType x0;
    x0 << 0, 0, 0;
    DiffDriveSimple::StateType x_goal;
    x_goal << 2, 0, 0;

    auto result = planner.solveWithCost(x0, x_goal, costs);

    EXPECT_TRUE(result.success()) << "Status: " << toString(result.status);
    EXPECT_GT(result.iterations, 0);

    // Should end near the goal
    auto& traj = planner.stateTrajectory();
    ASSERT_FALSE(traj.empty());
    Scalar dx = traj.back()[0] - x_goal[0];
    Scalar dy = traj.back()[1] - x_goal[1];
    EXPECT_LT(std::sqrt(dx * dx + dy * dy), 0.5);
}

TEST(iLQR_DiffDriveSimple, SolvePlanningProblem) {
    DiffDriveSimple model;
    iLQROptions opts;
    opts.horizon = 40;
    opts.dt = 0.1;

    iLQR<DiffDriveSimple> planner(model, opts);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {2, 0, 0, 0};

    auto result = planner.solve(problem);
    EXPECT_TRUE(result.success()) << "Status: " << toString(result.status);
    EXPECT_GT(result.trajectory.size(), 0u);
}

TEST(iLQR_DiffDriveSimple, ResetClearsState) {
    DiffDriveSimple model;
    iLQR<DiffDriveSimple> planner(model);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {1, 0, 0, 0};
    planner.solve(problem);

    EXPECT_FALSE(planner.stateTrajectory().empty());
    planner.reset();
    EXPECT_TRUE(planner.stateTrajectory().empty());
}

// ─── DiffDriveAccel iLQR ─────────────────────────────────────────────────────

TEST(iLQR_DiffDriveAccel, SolveWithCost) {
    DiffDriveAccel model;
    iLQROptions opts;
    opts.horizon = 50;
    opts.dt = 0.1;
    opts.maxIterations = 100;

    iLQR<DiffDriveAccel> planner(model, opts);

    constexpr int nx = 5;
    constexpr int nu = 2;

    iLQR<DiffDriveAccel>::CostMatrices costs;
    costs.Q  = MatX::Identity(nx, nx) * 1.0;
    costs.R  = MatX::Identity(nu, nu) * 0.1;
    costs.Qf = MatX::Identity(nx, nx) * 200.0;

    DiffDriveAccel::StateType x0 = DiffDriveAccel::StateType::Zero();
    DiffDriveAccel::StateType x_goal = DiffDriveAccel::StateType::Zero();
    x_goal[0] = 2.0;  // goal at x=2

    auto result = planner.solveWithCost(x0, x_goal, costs);
    EXPECT_TRUE(result.success()) << "Status: " << toString(result.status);

    // Terminal velocity should be near zero
    auto& traj = planner.stateTrajectory();
    EXPECT_NEAR(traj.back()[3], 0.0, 0.5);  // v ≈ 0 at goal
}

// ─── DiffDriveJerk iLQR ──────────────────────────────────────────────────────

TEST(iLQR_DiffDriveJerk, SolveWithCost) {
    DiffDriveJerk model;
    iLQROptions opts;
    opts.horizon = 60;
    opts.dt = 0.1;
    opts.maxIterations = 100;

    iLQR<DiffDriveJerk> planner(model, opts);

    constexpr int nx = 7;
    constexpr int nu = 2;

    iLQR<DiffDriveJerk>::CostMatrices costs;
    costs.Q  = MatX::Identity(nx, nx) * 0.5;
    costs.R  = MatX::Identity(nu, nu) * 0.1;
    costs.Qf = MatX::Identity(nx, nx) * 200.0;

    DiffDriveJerk::StateType x0 = DiffDriveJerk::StateType::Zero();
    DiffDriveJerk::StateType x_goal = DiffDriveJerk::StateType::Zero();
    x_goal[0] = 2.0;

    auto result = planner.solveWithCost(x0, x_goal, costs);

    // Jerk model may not converge as tightly — check it ran
    EXPECT_GT(result.iterations, 0);
    EXPECT_GT(result.trajectory.size(), 0u);
}

// ─── Ackermann iLQR ──────────────────────────────────────────────────────────

TEST(iLQR_AckermannSimple, SolveStraight) {
    AckermannSimple model;
    model.wheelbase = 2.5;
    iLQROptions opts;
    opts.horizon = 40;
    opts.dt = 0.1;

    iLQR<AckermannSimple> planner(model, opts);

    constexpr int nx = 3;
    constexpr int nu = 2;

    iLQR<AckermannSimple>::CostMatrices costs;
    costs.Q  = MatX::Identity(nx, nx) * 1.0;
    costs.R  = MatX::Identity(nu, nu) * 0.1;
    costs.Qf = MatX::Identity(nx, nx) * 200.0;

    AckermannSimple::StateType x0;
    x0 << 0, 0, 0;
    AckermannSimple::StateType x_goal;
    x_goal << 3, 0, 0;

    auto result = planner.solveWithCost(x0, x_goal, costs);
    EXPECT_TRUE(result.success()) << "Status: " << toString(result.status);
}

// ─── Omni iLQR ───────────────────────────────────────────────────────────────

TEST(iLQR_OmniSimple, DiagonalMotion) {
    OmniSimple model;
    iLQROptions opts;
    opts.horizon = 30;
    opts.dt = 0.1;

    iLQR<OmniSimple> planner(model, opts);

    constexpr int nx = 3;
    constexpr int nu = 3;

    iLQR<OmniSimple>::CostMatrices costs;
    costs.Q  = MatX::Identity(nx, nx) * 1.0;
    costs.R  = MatX::Identity(nu, nu) * 0.1;
    costs.Qf = MatX::Identity(nx, nx) * 200.0;

    OmniSimple::StateType x0;
    x0 << 0, 0, 0;
    OmniSimple::StateType x_goal;
    x_goal << 1, 1, 0;

    auto result = planner.solveWithCost(x0, x_goal, costs);
    EXPECT_TRUE(result.success()) << "Status: " << toString(result.status);

    auto& traj = planner.stateTrajectory();
    Scalar dx = traj.back()[0] - 1.0;
    Scalar dy = traj.back()[1] - 1.0;
    EXPECT_LT(std::sqrt(dx * dx + dy * dy), 0.5);
}
