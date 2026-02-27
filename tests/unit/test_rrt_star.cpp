// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/collision/occupancy_grid.hpp"

using namespace kinetra;
using namespace kinetra::planners;

TEST(RRTStar, EmptyEnvironment) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {5, 5, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);
    problem.options.goalTolerance = 0.5;

    RRTStarOptions opts;
    opts.maxIterations = 2000;
    opts.stepSize = 1.0;
    opts.timeLimitMs = 3000;

    RRTStar planner(opts);
    auto result = planner.solve(problem);

    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.trajectory.size(), 0u);
    EXPECT_GT(result.pathLength, 0);
}

TEST(RRTStar, WithObstacles) {
    PlanningProblem problem;
    problem.start = {-5, 0, 0, 0};
    problem.goal = {5, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);
    problem.options.goalTolerance = 0.5;

    // Create grid with obstacle in the middle
    collision::OccupancyGrid2D grid(-10, -10, 10, 10, 0.2);
    grid.addRectObstacle(Vec2(-0.5, -5), Vec2(0.5, 5));

    RRTStarOptions opts;
    opts.maxIterations = 5000;
    opts.stepSize = 0.5;
    opts.timeLimitMs = 5000;

    RRTStar planner(opts);
    planner.setCollisionChecker(
        [&](const Vec2& p) { return grid.isFree(p); },
        [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
    );

    auto result = planner.solve(problem);
    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    // Path should go around the wall
    EXPECT_GT(result.pathLength, 10.0);
}

TEST(RRTStar, ResetClearsTree) {
    RRTStar planner;
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {1, 1, 0, 0};
    problem.environment.bounds_min = Vec2(-5, -5);
    problem.environment.bounds_max = Vec2(5, 5);
    problem.options.goalTolerance = 0.5;

    planner.solve(problem);
    EXPECT_GT(planner.treeSize(), 0u);

    planner.reset();
    EXPECT_EQ(planner.treeSize(), 0u);
}
