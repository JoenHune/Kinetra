// SPDX-License-Identifier: BSD-3-Clause
// Integration tests: run full planning pipelines on standard scenarios.
#include <gtest/gtest.h>
#include "kinetra/kinetra.hpp"

using namespace kinetra;

class PlanningScenarioTest : public ::testing::Test {
protected:
    PlanningProblem makeNarrowPassage() {
        PlanningProblem p;
        p.start = {-8, 0, 0, 0};
        p.goal = {8, 0, 0, 0};
        p.environment.bounds_min = Vec2(-10, -10);
        p.environment.bounds_max = Vec2(10, 10);
        p.options.goalTolerance = 1.0;
        // Two walls with a narrow gap
        p.environment.obstacles.push_back(RectangleObstacle{Vec2(-0.5, -10), Vec2(0.5, -1)});
        p.environment.obstacles.push_back(RectangleObstacle{Vec2(-0.5, 1), Vec2(0.5, 10)});
        return p;
    }

    PlanningProblem makeUTurn() {
        PlanningProblem p;
        p.start = {0, 0, 0, 0};
        p.goal = {0, 0, constants::kPi, 0};  // Turn around
        p.environment.bounds_min = Vec2(-5, -5);
        p.environment.bounds_max = Vec2(5, 5);
        p.options.goalTolerance = 0.5;
        p.options.goalAngleTolerance = 0.3;
        return p;
    }
};

TEST_F(PlanningScenarioTest, NarrowPassageRRT) {
    auto problem = makeNarrowPassage();

    collision::OccupancyGrid2D grid(-10, -10, 10, 10, 0.2);
    grid.addRectObstacle(Vec2(-0.5, -10), Vec2(0.5, -1));
    grid.addRectObstacle(Vec2(-0.5, 1), Vec2(0.5, 10));

    planners::RRTStarOptions opts;
    opts.maxIterations = 10000;
    opts.stepSize = 0.5;
    opts.timeLimitMs = 10000;

    planners::RRTStar planner(opts);
    planner.setCollisionChecker(
        [&](const Vec2& p) { return grid.isFree(p); },
        [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
    );

    auto result = planner.solve(problem);
    EXPECT_TRUE(result.success()) << "RRT* failed on narrow passage: "
                                   << toString(result.status);
    if (result.success()) {
        EXPECT_GT(result.trajectory.size(), 2u);
        std::cout << "[NarrowPassage] RRT* solved in " << result.solveTimeMs
                  << "ms, path length=" << result.pathLength << std::endl;
    }
}

TEST_F(PlanningScenarioTest, STOMPEmptyEnvironment) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {10, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-15, -15);
    problem.environment.bounds_max = Vec2(15, 15);

    planners::STOMPOptions opts;
    opts.numTimesteps = 30;
    opts.maxIterations = 50;
    opts.numRollouts = 20;

    planners::STOMP planner(opts);
    auto result = planner.solve(problem);

    EXPECT_TRUE(result.success());
    EXPECT_GT(result.trajectory.size(), 0u);
    // In empty environment, path should be approximately straight
    EXPECT_LT(result.pathLength, 15.0);  // less than 1.5x optimal
    std::cout << "[EmptyEnv] STOMP solved in " << result.solveTimeMs
              << "ms, path length=" << result.pathLength << std::endl;
}
