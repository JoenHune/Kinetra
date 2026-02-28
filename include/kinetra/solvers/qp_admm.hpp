// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// ADMM-based Quadratic Program solver — standalone, no external dependencies.
// Inspired by OSQP architecture: single matrix factorization in setup,
// all subsequent operations are cheap. Suitable for embedded deployment.
//
// Solves:  min  0.5 * x' Q x + c' x
//          s.t. lb <= A x <= ub
//
// Supports sparse A matrix for efficient matrix-vector products when the
// constraint matrix has low density (typical for trajectory optimisation).

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
///
/// Two setup overloads:
///   - Dense A: setup(Q, c, A_dense, lb, ub)
///   - Sparse A: setup(Q, c, A_sparse, lb, ub)  — faster for low-density A
class QPSolverADMM {
public:
    QPSolverADMM() = default;
    explicit QPSolverADMM(QPSettings settings) : settings_(std::move(settings)) {}

    /// Setup with dense constraint matrix.
    bool setup(const MatX& Q, const VecX& c, const MatX& A,
               const VecX& lb, const VecX& ub);

    /// Setup with sparse constraint matrix (preferred for structured problems).
    bool setup(const MatX& Q, const VecX& c, const SpMatX& A,
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
    MatX Q_;
    SpMatX A_sp_;      // Sparse A — always stored as sparse
    SpMatX AT_sp_;     // A transposed (sparse)
    VecX c_, lb_, ub_;
    int n_{0};  // number of variables
    int m_{0};  // number of constraints

    // ADMM state
    VecX x_, z_, y_;        // primal, auxiliary, dual
    VecX z_prev_;

    // Cached matrices for efficient ADMM
    MatX ATA_;          // A' * A — stored dense (added to dense Q anyway)
    MatX kkt_factor_;   // Q + sigma*I + rho*ATA — for LDLT
    Scalar cached_rho_{0}; // rho used in last factorization
    bool is_setup_{false};

    // Internal helpers
    bool setupInternal();  // Common setup after A_sp_ is populated
    void factorizeKKT();
    void projectOntoBox(VecX& z, const VecX& lb, const VecX& ub) const;
    bool updateRho(Scalar primal_res, Scalar dual_res);
};

}  // namespace kinetra::solvers
