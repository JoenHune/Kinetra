// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/planners/lattice.hpp"
#include "kinetra/collision/occupancy_grid.hpp"

using namespace kinetra;
using namespace kinetra::planners;

TEST(LatticePlanner, SolveStraightLine) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {5, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);
    problem.options.goalTolerance = 1.0;

    LatticeOptions opts;
    opts.xyResolution = 0.5;
    opts.numAngles = 16;
    opts.maxExpansions = 50000;

    LatticePlanner planner(opts);
    auto result = planner.solve(problem);

    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.trajectory.size(), 0u);
}

TEST(LatticePlanner, SolveWithObstacles) {
    collision::OccupancyGrid2D grid(-15, -15, 15, 15, 0.5);
    grid.addCircleObstacle(Vec2(3, 0), 1.5);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {6, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-15, -15);
    problem.environment.bounds_max = Vec2(15, 15);
    problem.options.goalTolerance = 2.0;

    LatticeOptions opts;
    opts.xyResolution = 1.0;
    opts.numAngles = 8;
    opts.primitiveLength = 1.5;
    opts.maxExpansions = 100000;
    opts.timeLimitMs = 10000;

    LatticePlanner planner(opts);
    planner.setCollisionChecker(
        [&](const Vec2& p) { return grid.isFree(p); },
        [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
    );
    auto result = planner.solve(problem);

    // Should find a path around the obstacle
    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.pathLength, 0);
}

TEST(LatticePlanner, PrimitivesGenerated) {
    LatticeOptions opts;
    opts.numAngles = 8;
    opts.primitivesPerAngle = 3;

    LatticePlanner planner(opts);
    // Should have numAngles * primitivesPerAngle primitives
    EXPECT_EQ(planner.primitives().size(), 8u * 3u);
}

TEST(LatticePlanner, NameIsLattice) {
    LatticePlanner planner;
    EXPECT_EQ(planner.name(), "Lattice");
}

TEST(LatticePlanner, ResetDoesNotCrash) {
    LatticePlanner planner;
    planner.reset();
    SUCCEED();
}
