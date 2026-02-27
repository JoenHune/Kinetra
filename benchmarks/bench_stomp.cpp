// SPDX-License-Identifier: BSD-3-Clause
#include <benchmark/benchmark.h>
#include "kinetra/planners/stomp.hpp"

using namespace kinetra;

static void BM_STOMP_StraightLine(benchmark::State& state) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {10, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-15, -15);
    problem.environment.bounds_max = Vec2(15, 15);

    planners::STOMPOptions opts;
    opts.numTimesteps = static_cast<int>(state.range(0));
    opts.maxIterations = 30;
    opts.numRollouts = 15;

    for (auto _ : state) {
        planners::STOMP planner(opts);
        auto result = planner.solve(problem);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_STOMP_StraightLine)->Arg(20)->Arg(40)->Arg(80);
