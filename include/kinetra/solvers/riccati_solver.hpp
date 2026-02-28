// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Riccati recursion solver for OCP-structured QP sub-problems.
// Exploits the banded dynamics structure to solve in O(N·nx³) instead
// of O((N·nx)³), enabling real-time trajectory optimisation on embedded
// and WebAssembly targets.
//
// Algorithm:
//   Backward pass (k = N-1 → 0):
//     Compute feedback gains K_k and feedforward k_k via Riccati recursion
//     on the value function V_k(δx) = ½ δx' P_k δx + p_k' δx.
//
//   Forward pass (k = 0 → N-1):
//     Roll out optimal controls δu_k = −K_k δx_k − k_k
//     with projection onto box constraints [u_lb, u_ub].

#pragma once

#include <Eigen/Dense>
#include <cassert>

#include "kinetra/solvers/ocp_types.hpp"

namespace kinetra::solvers {

struct RiccatiSettings {
    Scalar regularization{static_cast<Scalar>(1e-8)};  // min diagonal of M
};

/// Solves an OCP QP using Riccati recursion.
///
/// Complexity per solve: O(N · max(nx³, nx²·nu, nu³)).
/// For the MPCC case (nx=4, nu=3, N=50): ~7 000 FLOPs.
///
/// Compared to dense ADMM on the same problem (n=304, m=507):
///   – ADMM: O(n³) factorization + O(n²) per iteration × 200 iters ≈ 28 M FLOPs
///   – Riccati: ~7 K FLOPs → **~4 000× faster**
class RiccatiSolver {
public:
    RiccatiSolver() = default;
    explicit RiccatiSolver(RiccatiSettings settings)
        : settings_(std::move(settings)) {}

    /// Solve the OCP QP.  Returns state and control corrections.
    [[nodiscard]] OCPSolution solve(const OCPProblem& ocp) const {
        assert(ocp.N > 0);
        assert(ocp.nx > 0);
        assert(ocp.nu > 0);
        assert(static_cast<int>(ocp.stages.size()) == ocp.N);

        const int N  = ocp.N;
        const int nx = ocp.nx;
        const int nu = ocp.nu;

        // ── Backward pass ──────────────────────────────────────────────
        // Value function: V_k(δx) = ½ δx' P_k δx + p_k' δx
        std::vector<MatX> K(static_cast<std::size_t>(N));    // nu × nx
        std::vector<VecX> kff(static_cast<std::size_t>(N));  // nu

        MatX P = ocp.Q_N;   // nx × nx  (terminal cost-to-go Hessian)
        VecX p = ocp.q_N;   // nx        (terminal cost-to-go gradient)

        for (int i = N - 1; i >= 0; --i) {
            const auto ui = static_cast<std::size_t>(i);
            const auto& s = ocp.stages[ui];

            // b = p + P c  (shifted gradient incorporating affine dynamics)
            VecX b = p + P * s.c;              // nx

            // M = R + B' P B    (nu × nu, should be SPD)
            MatX BtP = s.B.transpose() * P;    // nu × nx
            MatX M   = s.R + BtP * s.B;        // nu × nu

            // Regularise M to ensure positive definiteness
            Scalar min_diag = M.diagonal().minCoeff();
            if (min_diag < settings_.regularization) {
                M.diagonal().array() +=
                    settings_.regularization - min_diag;
            }

            // G = B' P A   (nu × nx)
            MatX G = BtP * s.A;

            // Solve  M K = G  and  M kff = r + B' b
            Eigen::LDLT<MatX> M_ldlt(M);
            K[ui]   = M_ldlt.solve(G);                        // nu × nx
            kff[ui] = M_ldlt.solve(s.r + s.B.transpose() * b); // nu

            // P_new = Q + A' P A − G' K
            P = s.Q + s.A.transpose() * P * s.A
                - G.transpose() * K[ui];

            // p_new = q + A' b − G' kff
            p = s.q + s.A.transpose() * b
                - G.transpose() * kff[ui];
        }

        // ── Forward pass ───────────────────────────────────────────────
        OCPSolution sol;
        sol.dx.resize(static_cast<std::size_t>(N + 1));
        sol.du.resize(static_cast<std::size_t>(N));
        sol.dx[0] = ocp.dx_0;

        for (int i = 0; i < N; ++i) {
            const auto ui = static_cast<std::size_t>(i);
            const auto& s = ocp.stages[ui];

            // Optimal control (unconstrained)
            sol.du[ui] = -K[ui] * sol.dx[ui] - kff[ui];

            // Project onto box constraints
            sol.du[ui] = sol.du[ui].cwiseMax(s.u_lb).cwiseMin(s.u_ub);

            // Propagate state: δx_{k+1} = A δx_k + B δu_k + c
            sol.dx[ui + 1] = s.A * sol.dx[ui] + s.B * sol.du[ui] + s.c;
        }

        // ── QP cost (for diagnostics / line search) ────────────────────
        sol.cost = 0;
        for (int i = 0; i < N; ++i) {
            const auto ui = static_cast<std::size_t>(i);
            const auto& s  = ocp.stages[ui];
            const auto& dx = sol.dx[ui];
            const auto& du = sol.du[ui];
            sol.cost += Scalar(0.5) * dx.dot(s.Q * dx) + s.q.dot(dx)
                      + Scalar(0.5) * du.dot(s.R * du) + s.r.dot(du);
        }
        const auto& dxN = sol.dx[static_cast<std::size_t>(N)];
        sol.cost += Scalar(0.5) * dxN.dot(ocp.Q_N * dxN) + ocp.q_N.dot(dxN);

        return sol;
    }

    [[nodiscard]] const RiccatiSettings& settings() const noexcept {
        return settings_;
    }
    RiccatiSettings& settings() noexcept { return settings_; }

private:
    RiccatiSettings settings_;
};

}  // namespace kinetra::solvers
