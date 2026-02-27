// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Sequential Quadratic Programming (SQP) solver for nonlinear programs.
// Builds on the composable NLPProblem formulation and uses QPSolverADMM
// as the QP sub-problem solver — fully standalone, no external deps.
//
// Solves:   min   f(x)
//           s.t.  cl ≤ g(x) ≤ cu
//                 xl ≤  x   ≤ xu
//
// Algorithm: at each iterate x_k:
//   1. Evaluate f, ∇f, g, J = ∂g/∂x
//   2. Form QP sub-problem:
//        min  ∇f'·d + 0.5·d'·H·d
//        s.t. cl - g ≤ J·d ≤ cu - g
//             xl - x ≤  d  ≤ xu - x
//      where H is a BFGS approximation of the Lagrangian Hessian.
//   3. Solve QP via ADMM.
//   4. Backtracking line search on L1 merit function:
//        φ(x) = f(x) + μ · ‖ max(0, cl - g(x), g(x) - cu) ‖₁
//   5. BFGS Hessian update.
//   6. Convergence test: ‖d‖ < tol AND constraint violation < tol.

#pragma once

#include <string>

#include "kinetra/core/types.hpp"
#include "kinetra/optimization/nlp_problem.hpp"

namespace kinetra::solvers {

struct SQPSettings {
    int maxIterations{100};
    Scalar tolerance{static_cast<Scalar>(1e-6)};          // KKT tolerance
    Scalar constraintTolerance{static_cast<Scalar>(1e-6)}; // Constraint violation tol
    Scalar initialMeritPenalty{static_cast<Scalar>(10.0)}; // μ in L1 merit
    Scalar meritPenaltyGrowth{static_cast<Scalar>(2.0)};   // μ scaling on failure
    Scalar maxMeritPenalty{static_cast<Scalar>(1e6)};
    Scalar lineSearchAlpha{static_cast<Scalar>(1e-4)};     // Armijo constant
    Scalar lineSearchBeta{static_cast<Scalar>(0.5)};       // Step shrink factor
    int lineSearchMaxTrials{20};
    bool verbose{false};
    double timeLimitMs{30000.0};
};

struct SQPResult {
    VecX x;                   // Optimal variables
    VecX lambda;              // Lagrange multipliers (QP dual)
    Scalar cost{0};           // Final objective
    Scalar constraintViolation{0};
    int iterations{0};
    int totalQPIterations{0}; // Sum of ADMM iterations across all QP solves
    bool converged{false};
    double solveTimeMs{0};
    std::string exitMessage;
};

/// SQP solver — solves an NLPProblem using Sequential Quadratic Programming.
///
/// Usage:
///   optimization::NLPProblem problem;
///   // ... add variable sets, constraints, cost terms ...
///   solvers::SQPSolver solver(settings);
///   auto result = solver.solve(problem);
class SQPSolver {
public:
    SQPSolver() = default;
    explicit SQPSolver(SQPSettings settings) : settings_(std::move(settings)) {}

    /// Solve the NLP. The variable values in `problem` are used as the
    /// initial guess and will be updated to the solution on return.
    [[nodiscard]] SQPResult solve(optimization::NLPProblem& problem);

    [[nodiscard]] const SQPSettings& settings() const noexcept { return settings_; }
    SQPSettings& settings() noexcept { return settings_; }

    /// Total QP ADMM iterations across all SQP iterations (for profiling)
    [[nodiscard]] int totalQPIterations() const noexcept { return total_qp_iters_; }

private:
    SQPSettings settings_;
    int total_qp_iters_{0};

    /// Compute L1 constraint violation: ‖max(0, cl - g, g - cu)‖₁
    [[nodiscard]] static Scalar constraintViolation(
        const VecX& g, const VecX& cl, const VecX& cu);

    /// L1 merit function: φ(x) = f + μ · violation
    [[nodiscard]] static Scalar meritFunction(
        Scalar f, Scalar violation, Scalar mu);

    /// BFGS update of Hessian approximation H.
    /// s = x_{k+1} - x_k, y = ∇L_{k+1} - ∇L_k
    static void bfgsUpdate(MatX& H, const VecX& s, const VecX& y);
};

}  // namespace kinetra::solvers
