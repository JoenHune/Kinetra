// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/planners/stomp.hpp"
#include "kinetra/collision/occupancy_grid.hpp"

using namespace kinetra;
using namespace kinetra::planners;

TEST(STOMP, SolveStraightLine) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {5, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);

    STOMPOptions opts;
    opts.numTimesteps = 30;
    opts.maxIterations = 50;
    opts.numRollouts = 15;

    STOMP planner(opts);
    auto result = planner.solve(problem);

    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.trajectory.size(), 0u);
    EXPECT_NEAR(result.trajectory.back().x, 5.0, 0.5);
}

TEST(STOMP, SolveDiagonalWithCostFn) {
    PlanningProblem problem;
    problem.start = {-5, -5, 0, 0};
    problem.goal  = { 5,  5, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);

    STOMPOptions opts;
    opts.numTimesteps = 40;
    opts.maxIterations = 30;
    opts.numRollouts = 10;

    STOMP planner(opts);
    planner.setCostFunction([](const MatX& traj) -> VecX {
        return VecX::Zero(traj.rows());  // No obstacle cost
    });

    auto result = planner.solve(problem);
    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.pathLength, 0);
}

TEST(STOMP, SolveWithObstacleCost) {
    collision::OccupancyGrid2D grid(-10, -10, 10, 10, 0.2);
    grid.addCircleObstacle(Vec2(2.5, 0), 1.5);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {5, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);

    STOMPOptions opts;
    opts.numTimesteps = 40;
    opts.maxIterations = 80;
    opts.numRollouts = 20;

    STOMP planner(opts);
    planner.setCostFunction([&](const MatX& traj) -> VecX {
        VecX costs = VecX::Zero(traj.rows());
        for (int i = 0; i < traj.rows(); ++i) {
            Scalar sd = grid.signedDistance(Vec2(traj(i, 0), traj(i, 1)));
            if (sd < 0.5) costs[i] = (0.5 - sd) * 100.0;
        }
        return costs;
    });

    auto result = planner.solve(problem);
    EXPECT_EQ(result.status, SolveStatus::kSuccess);
}

TEST(STOMP, ResetHasNoSideEffect) {
    STOMP planner;
    planner.reset();  // Should not crash
    SUCCEED();
}

TEST(STOMP, NameIsSTOMP) {
    STOMP planner;
    EXPECT_EQ(planner.name(), "STOMP");
}
