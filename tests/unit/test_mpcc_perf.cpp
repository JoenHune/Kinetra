// SPDX-License-Identifier: BSD-3-Clause
// Temporary profiling test for MPCC performance analysis
#include <gtest/gtest.h>
#include <chrono>
#include <iostream>
#include "kinetra/planners/mpcc.hpp"
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/core/reference_path.hpp"
#include "kinetra/optimization/nlp_problem.hpp"
#include "kinetra/solvers/sqp.hpp"
#include "kinetra/solvers/qp_admm.hpp"

using namespace kinetra;
using namespace kinetra::planners;
using Clock = std::chrono::steady_clock;

static ReferencePath makeStraightPath(Scalar length = 5.0, int n = 20) {
    std::vector<Waypoint2D> wps;
    for (int i = 0; i < n; ++i) {
        Scalar t = Scalar(i) / Scalar(n - 1);
        wps.push_back({t * length, 0, 0, t});
    }
    return ReferencePath(wps);
}

TEST(MPCCPerf, ProfileSolve) {
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 15;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {3, 0, 0, 0};
    problem.environment.bounds_min = {-1, -3};
    problem.environment.bounds_max = {5, 3};

    // Run 5 times, report best
    double best = 1e9;
    for (int run = 0; run < 5; ++run) {
        auto t0 = Clock::now();
        auto result = mpcc.solve(problem);
        auto t1 = Clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        best = std::min(best, ms);
    }

    // Final run for reporting
    auto result = mpcc.solve(problem);

    std::cout << "\n=== MPCC Profile (best of 5) ===\n"
              << "  Total:     " << best << " ms\n"
              << "  SQP iters: " << result.sqpIterations << "\n"
              << "  QP iters:  " << result.totalQPIterations
              << " (avg " << (result.sqpIterations > 0 ? result.totalQPIterations / result.sqpIterations : 0) << "/QP)\n"
              << "  Converged: " << (result.planningResult.status == SolveStatus::kSuccess) << "\n"
              << "  Cost:      " << result.finalCost << "\n"
              << "  Violation: " << result.constraintViolation << "\n"
              << "  n=" << ((opts.horizon+1)*3 + opts.horizon*2 + opts.horizon+1)
              << " m=" << (opts.horizon*3 + 3 + opts.horizon)
              << "\n";
}

TEST(MPCCPerf, ProfileQPSolverAlone) {
    // Benchmark QP ADMM solver with realistic MPCC dimensions
    int n = 124;  // states + controls + progress
    int m = 207;  // constraints + variable bounds

    // Create a simple QP problem
    MatX Q = MatX::Identity(n, n);
    VecX c = VecX::Random(n);
    MatX A = MatX::Random(m, n) * 0.1;
    VecX lb = VecX::Constant(m, -10.0);
    VecX ub = VecX::Constant(m, 10.0);

    double best_setup = 1e9, best_solve = 1e9;
    for (int run = 0; run < 10; ++run) {
        auto t0 = Clock::now();
        solvers::QPSolverADMM qp;
        qp.settings().maxIterations = 500;
        qp.setup(Q, c, A, lb, ub);
        auto t1 = Clock::now();
        auto res = qp.solve();
        auto t2 = Clock::now();
        double setup_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        double solve_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        best_setup = std::min(best_setup, setup_ms);
        best_solve = std::min(best_solve, solve_ms);
    }

    std::cout << "\n=== QP ADMM Profile (n=" << n << " m=" << m << ") ===\n"
              << "  Setup: " << best_setup << " ms\n"
              << "  Solve: " << best_solve << " ms\n";
}

TEST(MPCCPerf, ProfileNLPEvaluation) {
    // Profile NLP evaluation components independently
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {3, 0, 0, 0};
    problem.environment.bounds_min = {-1, -3};
    problem.environment.bounds_max = {5, 3};

    collision::OccupancyGrid2D grid(-1, -3, 5, 3, 0.1);

    // Build NLP manually to profile individual components
    using StateType = DiffDriveSimple::StateType;
    StateType x0 = StateType::Zero();

    // Full solve for getting the built NLP (via the public API)  
    auto result = mpcc.solve(problem, grid);
    std::cout << "\n=== NLP Solve timing: " << result.planningResult.solveTimeMs << " ms ===\n";

    // Now time individual NLP operations by rebuilding a simple NLP
    int N = opts.horizon;
    int nx = 3, nu = 2;
    int n_total = (N+1)*nx + N*nu + (N+1);  // 124
    int m_total = N*nx + nx + N;              // 83

    // Create dummy NLP and profile
    optimization::NLPProblem nlp;
    VecX slo(3), shi(3), clo(2), chi(2);
    slo << -100, -100, -M_PI; shi << 100, 100, M_PI;
    clo << -5, -5; chi << 5, 5;

    auto stateVars = std::make_shared<mpcc_nlp::MPCCStateVars>(N, nx, slo, shi);
    auto controlVars = std::make_shared<mpcc_nlp::MPCCControlVars>(N, nu, clo, chi);
    auto progressVars = std::make_shared<mpcc_nlp::MPCCProgressVars>(N, 0, path.totalLength());
    nlp.addVariableSet(stateVars);
    nlp.addVariableSet(controlVars);
    nlp.addVariableSet(progressVars);

    auto dynCon = std::make_shared<mpcc_nlp::DynamicsConstraint<DiffDriveSimple>>(N, model, opts.dt);
    dynCon->linkVariables({stateVars, controlVars});
    nlp.addConstraintSet(dynCon);

    auto initCon = std::make_shared<mpcc_nlp::InitialStateConstraint>(nx, VecX::Zero(nx));
    initCon->linkVariables({stateVars});
    nlp.addConstraintSet(initCon);

    auto progCon = std::make_shared<mpcc_nlp::ProgressConstraint>(N, opts.maxProgressRate, opts.dt);
    progCon->linkVariables({progressVars});
    nlp.addConstraintSet(progCon);

    auto contCost = std::make_shared<mpcc_nlp::ContouringCost>(N, nx, path, 50, 20, 30, 5);
    contCost->linkVariables({stateVars, progressVars});
    nlp.addCostSet(contCost);

    auto ctrlCost = std::make_shared<mpcc_nlp::ControlCost>(N, nu, 1.0);
    ctrlCost->linkVariables({controlVars});
    nlp.addCostSet(ctrlCost);

    auto obsCost = std::make_shared<mpcc_nlp::ObstacleCost>(N, nx, grid, 0.3, 100);
    obsCost->linkVariables({stateVars});
    nlp.addCostSet(obsCost);

    // Initialize along path
    VecX init = VecX::Zero(n_total);
    for (int k = 0; k <= N; ++k) {
        Scalar s = path.totalLength() * k / N;
        Vec3 ref = path.evaluate(s);
        init[k*nx] = ref[0]; init[k*nx+1] = ref[1]; init[k*nx+2] = ref[2];
        init[(N+1)*nx + N*nu + k] = s;
    }
    nlp.setVariableValues(init);

    // Profile 100 evaluations of each operation
    constexpr int REPS = 100;
    auto t0 = Clock::now();
    for (int i = 0; i < REPS; ++i) nlp.totalCost();
    auto t1 = Clock::now();
    for (int i = 0; i < REPS; ++i) nlp.costGradient();
    auto t2 = Clock::now();
    for (int i = 0; i < REPS; ++i) nlp.constraintValues();
    auto t3 = Clock::now();
    for (int i = 0; i < REPS; ++i) nlp.constraintJacobian();
    auto t4 = Clock::now();

    double totalCost_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / REPS;
    double costGrad_us = std::chrono::duration<double, std::micro>(t2 - t1).count() / REPS;
    double consVal_us = std::chrono::duration<double, std::micro>(t3 - t2).count() / REPS;
    double consJac_us = std::chrono::duration<double, std::micro>(t4 - t3).count() / REPS;

    std::cout << "  totalCost:         " << totalCost_us << " μs\n"
              << "  costGradient:      " << costGrad_us << " μs\n"
              << "  constraintValues:  " << consVal_us << " μs\n"
              << "  constraintJacobian:" << consJac_us << " μs\n"
              << "  SUM per iter:      " << (totalCost_us + costGrad_us + consVal_us + consJac_us) << " μs\n"
              << "  × 15 SQP iters:    " << (totalCost_us + costGrad_us + consVal_us + consJac_us) * 15 / 1000 << " ms\n";
}
