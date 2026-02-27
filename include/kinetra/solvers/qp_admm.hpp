// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// ADMM-based Quadratic Program solver — standalone, no external dependencies.
// Inspired by OSQP architecture: single matrix factorization in setup,
// all subsequent operations are cheap. Suitable for embedded deployment.
//
// Solves:  min  0.5 * x' Q x + c' x
//          s.t. lb <= A x <= ub

#pragma once

#include <optional>

#include "kinetra/core/types.hpp"

namespace kinetra::solvers {

struct QPSettings {
    int maxIterations{4000};
    Scalar absTolerance{static_cast<Scalar>(1e-4)};
    Scalar relTolerance{static_cast<Scalar>(1e-4)};
    Scalar rho{static_cast<Scalar>(0.1)};          // ADMM penalty parameter
    Scalar sigma{static_cast<Scalar>(1e-6)};        // Regularization
    Scalar alpha{static_cast<Scalar>(1.6)};          // Over-relaxation parameter
    bool adaptiveRho{true};
    bool verbose{false};
    bool warmStart{true};
};

struct QPResult {
    VecX x;               // Primal solution
    VecX y;               // Dual solution (Lagrange multipliers)
    int iterations{0};
    Scalar primalResidual{0};
    Scalar dualResidual{0};
    Scalar objectiveValue{0};
    bool converged{false};
    double solveTimeMs{0};
};

/// ADMM-based QP solver.
/// The algorithm:
///   1. Setup: factorize (Q + sigma*I + rho*A'A)
///   2. Iterate:
///      a. x-update:  solve KKT system
///      b. z-update:  project onto [lb, ub]
///      c. y-update:  dual variable update
///      d. Check convergence (primal + dual residual)
class QPSolverADMM {
public:
    QPSolverADMM() = default;
    explicit QPSolverADMM(QPSettings settings) : settings_(std::move(settings)) {}

    /// Setup the QP problem. Performs matrix factorization.
    /// Returns false if the problem is malformed.
    bool setup(const MatX& Q, const VecX& c, const MatX& A,
               const VecX& lb, const VecX& ub);

    /// Solve the QP. Call setup() first.
    [[nodiscard]] QPResult solve();

    /// Warm-start with previous solution.
    void warmStart(const VecX& x0, const VecX& y0);

    /// Update linear cost term without re-factorizing.
    void updateLinearCost(const VecX& c);

    /// Update constraint bounds without re-factorizing.
    void updateBounds(const VecX& lb, const VecX& ub);

    [[nodiscard]] const QPSettings& settings() const noexcept { return settings_; }
    QPSettings& settings() noexcept { return settings_; }

private:
    QPSettings settings_;

    // Problem data
    MatX Q_, A_;
    VecX c_, lb_, ub_;
    int n_{0};  // number of variables
    int m_{0};  // number of constraints

    // ADMM state
    VecX x_, z_, y_;        // primal, auxiliary, dual
    VecX z_prev_;

    // Factorized KKT matrix (stored for efficient re-solves)
    MatX kkt_factor_;  // Will use Eigen::LDLT in implementation
    bool is_setup_{false};

    // Internal helpers
    void projectOntoBox(VecX& z, const VecX& lb, const VecX& ub) const;
    void updateRho(Scalar primal_res, Scalar dual_res);
};

}  // namespace kinetra::solvers
