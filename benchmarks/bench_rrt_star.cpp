// SPDX-License-Identifier: BSD-3-Clause
#include <benchmark/benchmark.h>
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/collision/occupancy_grid.hpp"

using namespace kinetra;

static void BM_RRTStar_EmptyEnv(benchmark::State& state) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {static_cast<Scalar>(state.range(0)), 0, 0, 0};
    problem.environment.bounds_min = Vec2(-20, -20);
    problem.environment.bounds_max = Vec2(20, 20);
    problem.options.goalTolerance = 1.0;

    planners::RRTStarOptions opts;
    opts.maxIterations = 3000;
    opts.stepSize = 1.0;
    opts.timeLimitMs = 5000;

    for (auto _ : state) {
        planners::RRTStar planner(opts);
        auto result = planner.solve(problem);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RRTStar_EmptyEnv)->Arg(5)->Arg(10)->Arg(15);

static void BM_RRTStar_WithObstacles(benchmark::State& state) {
    collision::OccupancyGrid2D grid(-15, -15, 15, 15, 0.2);
    // Add random obstacles
    grid.addCircleObstacle(Vec2(3, 0), 1.5);
    grid.addCircleObstacle(Vec2(-3, 2), 1.0);
    grid.addRectObstacle(Vec2(0, -4), Vec2(2, -2));

    PlanningProblem problem;
    problem.start = {-10, 0, 0, 0};
    problem.goal = {10, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-15, -15);
    problem.environment.bounds_max = Vec2(15, 15);
    problem.options.goalTolerance = 1.0;

    planners::RRTStarOptions opts;
    opts.maxIterations = 5000;
    opts.stepSize = 0.5;

    for (auto _ : state) {
        planners::RRTStar planner(opts);
        planner.setCollisionChecker(
            [&](const Vec2& p) { return grid.isFree(p); },
            [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
        );
        auto result = planner.solve(problem);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RRTStar_WithObstacles);
