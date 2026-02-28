// SPDX-License-Identifier: BSD-3-Clause
// Tests for the Riccati OCP QP solver and GN-Riccati MPCC pathway.
#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "kinetra/solvers/riccati_solver.hpp"
#include "kinetra/planners/mpcc.hpp"
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"
#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/core/reference_path.hpp"

using namespace kinetra;
using namespace kinetra::solvers;
using namespace kinetra::planners;

// ═════════════════════════════════════════════════════════════════════════════
// Riccati Solver — unit tests
// ═════════════════════════════════════════════════════════════════════════════

TEST(RiccatiSolver, TrivialOneStage) {
    // 1-stage, 1D: min 0.5·du² + 0.5·(du + c)²
    // with c = -1 (defect), Q=R=1, A=B=1
    // Optimal du = 0.5, dx₁ = -0.5, cost = 0.25 - 0.5·0 + ...
    OCPProblem ocp;
    ocp.nx = 1;
    ocp.nu = 1;
    ocp.N  = 1;
    ocp.dx_0 = VecX::Zero(1);

    OCPStage stg;
    stg.A = MatX::Identity(1, 1);
    stg.B = MatX::Identity(1, 1);
    stg.c = VecX(1);  stg.c[0] = -1;
    stg.Q = MatX::Identity(1, 1);
    stg.q = VecX::Zero(1);
    stg.R = MatX::Identity(1, 1);
    stg.r = VecX::Zero(1);
    stg.u_lb = VecX::Constant(1, -1e6);
    stg.u_ub = VecX::Constant(1, 1e6);
    ocp.stages = {stg};

    ocp.Q_N = MatX::Identity(1, 1);
    ocp.q_N = VecX::Zero(1);

    RiccatiSolver solver;
    auto sol = solver.solve(ocp);

    ASSERT_EQ(sol.du.size(), 1u);
    ASSERT_EQ(sol.dx.size(), 2u);
    EXPECT_NEAR(sol.du[0][0], 0.5, 1e-10);
    EXPECT_NEAR(sol.dx[1][0], -0.5, 1e-10);
}

TEST(RiccatiSolver, MultiStageLQR) {
    // 2-state (pos, vel), 1-control (force), N=10
    // Regulation: drive state to zero
    const int nx = 2, nu = 1, N = 10;
    Scalar dt = 0.1;

    OCPProblem ocp;
    ocp.nx = nx;
    ocp.nu = nu;
    ocp.N  = N;
    ocp.dx_0 = VecX(2);
    ocp.dx_0 << 1.0, 0.0;  // regulate from pos=1, vel=0

    MatX A(2, 2);
    A << 1, dt, 0, 1;
    MatX B(2, 1);
    B << 0.5 * dt * dt, dt;
    MatX Q = MatX::Identity(2, 2);
    Q(1, 1) = 0.1;
    MatX R = 0.1 * MatX::Identity(1, 1);

    ocp.stages.resize(static_cast<std::size_t>(N));
    for (int k = 0; k < N; ++k) {
        auto& s = ocp.stages[static_cast<std::size_t>(k)];
        s.A = A;
        s.B = B;
        s.c = VecX::Zero(2);
        s.Q = Q;
        s.q = VecX::Zero(2);
        s.R = R;
        s.r = VecX::Zero(1);
        s.u_lb = VecX::Constant(1, -1e6);
        s.u_ub = VecX::Constant(1, 1e6);
    }
    ocp.Q_N = Q;
    ocp.q_N = VecX::Zero(2);

    RiccatiSolver solver;
    auto sol = solver.solve(ocp);

    // Dynamics should be satisfied
    for (int k = 0; k < N; ++k) {
        VecX expected = A * sol.dx[static_cast<std::size_t>(k)]
                      + B * sol.du[static_cast<std::size_t>(k)];
        EXPECT_NEAR((sol.dx[static_cast<std::size_t>(k + 1)] - expected).norm(),
                    0.0, 1e-10);
    }

    // State should converge towards zero
    EXPECT_LT(sol.dx[static_cast<std::size_t>(N)].norm(), 1.0);

    // Cost should be finite and positive
    EXPECT_GT(sol.cost, 0);
    EXPECT_LT(sol.cost, 1e6);
}

TEST(RiccatiSolver, BoxConstraints) {
    // 1D system with tight box constraint on control
    OCPProblem ocp;
    ocp.nx = 1;
    ocp.nu = 1;
    ocp.N  = 1;
    ocp.dx_0 = VecX(1);  ocp.dx_0[0] = 0;

    OCPStage stg;
    stg.A = MatX::Identity(1, 1);
    stg.B = MatX::Identity(1, 1);
    stg.c = VecX(1);  stg.c[0] = -10;  // large defect
    stg.Q = MatX::Identity(1, 1);
    stg.q = VecX::Zero(1);
    stg.R = 0.001 * MatX::Identity(1, 1);  // cheap control
    stg.r = VecX::Zero(1);
    stg.u_lb = VecX::Constant(1, -1.0);  // tight box
    stg.u_ub = VecX::Constant(1,  1.0);
    ocp.stages = {stg};

    ocp.Q_N = MatX::Identity(1, 1);
    ocp.q_N = VecX::Zero(1);

    RiccatiSolver solver;
    auto sol = solver.solve(ocp);

    // Control should be clipped to max
    EXPECT_NEAR(sol.du[0][0], 1.0, 1e-10);
    // dx₁ = 0 + 1·1 + (-10) = -9
    EXPECT_NEAR(sol.dx[1][0], -9.0, 1e-10);
}

// ═════════════════════════════════════════════════════════════════════════════
// MPCC Riccati — integration tests
// ═════════════════════════════════════════════════════════════════════════════

namespace {

ReferencePath makeStraightPath(Scalar length = 5.0, int nPts = 20) {
    std::vector<Waypoint2D> wps;
    for (int i = 0; i < nPts; ++i) {
        Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(nPts - 1);
        wps.push_back({t * length, 0, 0, t});
    }
    return ReferencePath(wps);
}

ReferencePath makeCurvedPath() {
    std::vector<Waypoint2D> wps;
    int N = 20;
    for (int i = 0; i <= N; ++i) {
        Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(N);
        Scalar angle = t * constants::kHalfPi;
        wps.push_back({std::sin(angle),
                        Scalar(1) - std::cos(angle), angle, t});
    }
    return ReferencePath(wps);
}

PlanningProblem makeSimpleProblem() {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {5, 0, 0, 0};
    problem.environment.bounds_min = {-1, -3};
    problem.environment.bounds_max = {7, 3};
    return problem;
}

}  // namespace

TEST(MPCC_Riccati, SolveStraightLine_DiffDriveSimple) {
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.useRiccati = true;
    opts.sqpSettings.maxIterations = 30;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();
    problem.goal = {3, 0, 0, 0};

    auto result = mpcc.solve(problem);

    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
    EXPECT_EQ(result.planningResult.plannerName, "MPCC");

    // Progress should be monotonically non-decreasing
    for (std::size_t i = 1; i < result.progressValues.size(); ++i) {
        EXPECT_GE(result.progressValues[i],
                  result.progressValues[i - 1] - 1e-6);
    }

    // Should have controls
    EXPECT_EQ(result.controls.size(), static_cast<std::size_t>(opts.horizon));

    // Error vectors correct size
    EXPECT_EQ(result.contourErrors.size(),
              static_cast<std::size_t>(opts.horizon + 1));
}

TEST(MPCC_Riccati, SolveCurvedPath_DiffDriveSimple) {
    DiffDriveSimple model;
    auto path = makeCurvedPath();

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.useRiccati = true;
    opts.sqpSettings.maxIterations = 30;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    auto result = mpcc.solve(makeSimpleProblem());

    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
    EXPECT_GT(result.progressValues.back(), result.progressValues.front());
}

TEST(MPCC_Riccati, SolveWithObstacles_DiffDriveSimple) {
    DiffDriveSimple model;
    auto path = makeStraightPath(4.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.useRiccati = true;
    opts.wObstacle = 200.0;
    opts.safeDistance = 0.2;
    opts.sqpSettings.maxIterations = 30;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {4, 0, 0, 0};
    problem.environment.bounds_min = {-1, -3};
    problem.environment.bounds_max = {5, 3};
    problem.environment.obstacles.push_back(CircleObstacle{{2.0, 0.0}, 0.5});

    auto result = mpcc.solve(problem);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

TEST(MPCC_Riccati, SolveDiffDriveAccel) {
    DiffDriveAccel model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.useRiccati = true;
    opts.sqpSettings.maxIterations = 20;

    MPCC<DiffDriveAccel> mpcc(model, path, opts);
    auto result = mpcc.solve(makeSimpleProblem());
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

TEST(MPCC_Riccati, SolveAckermann) {
    AckermannSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.useRiccati = true;
    opts.sqpSettings.maxIterations = 20;

    MPCC<AckermannSimple> mpcc(model, path, opts);
    auto result = mpcc.solve(makeSimpleProblem());
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

TEST(MPCC_Riccati, SolveOmni) {
    OmniSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.useRiccati = true;
    opts.sqpSettings.maxIterations = 20;

    MPCC<OmniSimple> mpcc(model, path, opts);
    auto result = mpcc.solve(makeSimpleProblem());
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

TEST(MPCC_Riccati, PerformanceBenchmark) {
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.useRiccati = true;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();
    problem.goal = {3, 0, 0, 0};

    // Warm up
    mpcc.solve(problem);

    // Benchmark
    using SteadyClock = std::chrono::steady_clock;
    double best = 1e9;
    for (int run = 0; run < 5; ++run) {
        auto t0 = SteadyClock::now();
        auto result = mpcc.solve(problem);
        auto t1 = SteadyClock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        best = std::min(best, ms);
    }

    auto result = mpcc.solve(problem);
    std::cout << "\n=== GN-Riccati MPCC Benchmark (best of 5) ===\n"
              << "  Time:      " << best << " ms\n"
              << "  SQP iters: " << result.sqpIterations << "\n"
              << "  Converged: " << (result.planningResult.status == SolveStatus::kSuccess) << "\n"
              << "  Cost:      " << result.finalCost << "\n"
              << "  Violation: " << result.constraintViolation << "\n";
}

// ═════════════════════════════════════════════════════════════════════════════
// RRT* Dubins — integration test
// ═════════════════════════════════════════════════════════════════════════════

TEST(RRTStarDubins, EmptyEnvironment) {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal  = {5, 5, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);
    problem.options.goalTolerance = 1.0;

    RRTStarOptions opts;
    opts.maxIterations = 5000;
    opts.stepSize = 1.0;
    opts.timeLimitMs = 5000;
    opts.useDubinsSteering = true;
    opts.turningRadius = 1.0;
    opts.dubinsCollisionStep = 0.2;

    RRTStar planner(opts);
    auto result = planner.solve(problem);

    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.trajectory.size(), 0u);
    EXPECT_GT(result.pathLength, 0);
}

TEST(RRTStarDubins, WithObstacles) {
    PlanningProblem problem;
    problem.start = {-5, 0, 0, 0};
    problem.goal  = {5, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);
    problem.options.goalTolerance = 1.5;

    collision::OccupancyGrid2D grid(-10, -10, 10, 10, 0.2);
    grid.addRectObstacle(Vec2(-0.5, -5), Vec2(0.5, 5));

    RRTStarOptions opts;
    opts.maxIterations = 5000;
    opts.stepSize = 0.5;
    opts.timeLimitMs = 8000;
    opts.useDubinsSteering = true;
    opts.turningRadius = 0.8;

    RRTStar planner(opts);
    planner.setCollisionChecker(
        [&](const Vec2& p) { return grid.isFree(p); },
        [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
    );

    auto result = planner.solve(problem);
    EXPECT_EQ(result.status, SolveStatus::kSuccess);
    EXPECT_GT(result.pathLength, 10.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// SDF Gradient — unit test
// ═════════════════════════════════════════════════════════════════════════════

TEST(OccupancyGrid, SDFGradient) {
    collision::OccupancyGrid2D grid(-5, -5, 5, 5, 0.1);
    grid.addCircleObstacle(Vec2(0, 0), 1.0);

    // Point to the right of the obstacle — gradient should point ≈ (+1, 0)
    Vec2 grad = grid.sdfGradient(Vec2(2, 0));
    EXPECT_GT(grad.x(), 0.5);   // pointing away from obstacle
    EXPECT_NEAR(grad.y(), 0.0, 0.3);

    // Point above the obstacle
    Vec2 grad2 = grid.sdfGradient(Vec2(0, 2));
    EXPECT_NEAR(grad2.x(), 0.0, 0.3);
    EXPECT_GT(grad2.y(), 0.5);
}
