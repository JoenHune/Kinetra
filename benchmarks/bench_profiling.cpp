// SPDX-License-Identifier: BSD-3-Clause
// Comprehensive profiling benchmark — exercises every planner + solver
// for timing analysis on ARM and x86. Outputs JSON for CI dashboards.
#include <benchmark/benchmark.h>

#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/planners/ilqr.hpp"
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/planners/stomp.hpp"
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/solvers/qp_admm.hpp"
#include "kinetra/spaces/dubins.hpp"

using namespace kinetra;

// ─── Shared test environment ─────────────────────────────────────────────────
static collision::OccupancyGrid2D makeObstacleEnv() {
    collision::OccupancyGrid2D grid(-15, -15, 15, 15, 0.2);
    grid.addCircleObstacle(Vec2(3, 1), 1.5);
    grid.addCircleObstacle(Vec2(-2, -3), 1.0);
    grid.addCircleObstacle(Vec2(6, -2), 2.0);
    grid.addRectObstacle(Vec2(-1, 3), Vec2(2, 5));
    return grid;
}

static PlanningProblem makeStandardProblem() {
    PlanningProblem p;
    p.start = {-10, 0, 0, 0};
    p.goal  = { 10, 0, 0, 0};
    p.environment.bounds_min = Vec2(-15, -15);
    p.environment.bounds_max = Vec2(15, 15);
    p.options.goalTolerance = 1.0;
    return p;
}

// ─── RRT* profiling ──────────────────────────────────────────────────────────
static void BM_Profile_RRTStar(benchmark::State& state) {
    auto grid = makeObstacleEnv();
    auto problem = makeStandardProblem();

    planners::RRTStarOptions opts;
    opts.maxIterations = static_cast<int>(state.range(0));
    opts.stepSize = 0.5;
    opts.timeLimitMs = 10000;

    for (auto _ : state) {
        planners::RRTStar planner(opts);
        planner.setCollisionChecker(
            [&](const Vec2& p) { return grid.isFree(p); },
            [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
        );
        auto result = planner.solve(problem);
        benchmark::DoNotOptimize(result);
        state.counters["nodes"] = static_cast<double>(planner.treeSize());
    }
}
BENCHMARK(BM_Profile_RRTStar)->Arg(1000)->Arg(3000)->Arg(5000)
    ->Unit(benchmark::kMillisecond);

// ─── STOMP profiling ─────────────────────────────────────────────────────────
static void BM_Profile_STOMP(benchmark::State& state) {
    auto grid = makeObstacleEnv();
    auto problem = makeStandardProblem();

    planners::STOMPOptions opts;
    opts.numTimesteps = static_cast<int>(state.range(0));
    opts.maxIterations = 50;
    opts.numRollouts = 20;
    opts.timeLimitMs = 10000;

    for (auto _ : state) {
        planners::STOMP planner(opts);
        planner.setCostFunction([&](const MatX& traj) -> VecX {
            VecX costs = VecX::Zero(traj.rows());
            for (int i = 0; i < traj.rows(); ++i) {
                Scalar sd = grid.signedDistance(Vec2(traj(i, 0), traj(i, 1)));
                if (sd < 1.0) costs[i] = (1.0 - sd) * 10.0;
            }
            return costs;
        });
        auto result = planner.solve(problem);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Profile_STOMP)->Arg(30)->Arg(60)->Arg(100)
    ->Unit(benchmark::kMillisecond);

// ─── iLQR profiling (DiffDriveAccel) ────────────────────────────────────────
static void BM_Profile_iLQR(benchmark::State& state) {
    DiffDriveAccel model;
    planners::iLQROptions opts;
    opts.horizon = static_cast<int>(state.range(0));
    opts.dt = 0.05;
    opts.maxIterations = 50;
    opts.timeLimitMs = 10000;

    DiffDriveAccel::StateType x0 = DiffDriveAccel::StateType::Zero();
    DiffDriveAccel::StateType xg;
    xg << 10, 0, 0, 0, 0;

    planners::iLQR<DiffDriveAccel>::CostMatrices costs;
    constexpr int nx = 5, nu = 2;
    costs.Q  = MatX::Identity(nx, nx) * 1.0;
    costs.Qf = MatX::Identity(nx, nx) * 100.0;
    costs.R  = MatX::Identity(nu, nu) * 0.1;

    for (auto _ : state) {
        planners::iLQR<DiffDriveAccel> planner(model, opts);
        auto result = planner.solveWithCost(x0, xg, costs);
        benchmark::DoNotOptimize(result);
        state.counters["iters"] = result.iterations;
    }
}
BENCHMARK(BM_Profile_iLQR)->Arg(30)->Arg(50)->Arg(100)
    ->Unit(benchmark::kMillisecond);

// ─── Dubins path profiling ──────────────────────────────────────────────────
static void BM_Profile_Dubins(benchmark::State& state) {
    DubinsSpace dubins(1.0, -20, 20, -20, 20);  // min turning radius = 1m
    SE2State start{0, 0, 0};

    for (auto _ : state) {
        for (int i = 0; i < 100; ++i) {
            SE2State goal{static_cast<Scalar>(i) * Scalar(0.1), Scalar(3),
                          constants::kPi * Scalar(0.5)};
            auto d = dubins.distance(start, goal);
            benchmark::DoNotOptimize(d);
        }
    }
    state.counters["queries"] = 100;
}
BENCHMARK(BM_Profile_Dubins)->Unit(benchmark::kMicrosecond);

// ─── QP solver profiling ────────────────────────────────────────────────────
static void BM_Profile_QP(benchmark::State& state) {
    int n = static_cast<int>(state.range(0));
    MatX Q = MatX::Identity(n, n) * 2.0;
    VecX c = VecX::LinSpaced(n, -1, 1);
    MatX A = MatX::Identity(n, n);
    VecX lb = VecX::Constant(n, -1.0);
    VecX ub = VecX::Constant(n, 1.0);

    for (auto _ : state) {
        solvers::QPSolverADMM solver;
        solver.setup(Q, c, A, lb, ub);
        auto result = solver.solve();
        benchmark::DoNotOptimize(result);
        state.counters["iters"] = result.iterations;
    }
}
BENCHMARK(BM_Profile_QP)->Arg(10)->Arg(50)->Arg(100)
    ->Unit(benchmark::kMicrosecond);

// ─── SDF computation profiling ──────────────────────────────────────────────
static void BM_Profile_SDF(benchmark::State& state) {
    int grid_size = static_cast<int>(state.range(0));
    Scalar extent = static_cast<Scalar>(grid_size) * 0.1;

    for (auto _ : state) {
        collision::OccupancyGrid2D grid(-extent, -extent, extent, extent, 0.1);
        grid.addCircleObstacle(Vec2(0, 0), extent * 0.3);
        grid.addRectObstacle(Vec2(-extent * 0.5, -extent * 0.5),
                             Vec2(-extent * 0.2, -extent * 0.2));
        grid.recomputeDistanceField();
        auto sd = grid.signedDistance(Vec2(extent * 0.4, extent * 0.4));
        benchmark::DoNotOptimize(sd);
    }
    state.counters["cells"] = grid_size * grid_size;
}
BENCHMARK(BM_Profile_SDF)->Arg(50)->Arg(100)->Arg(200)
    ->Unit(benchmark::kMillisecond);
