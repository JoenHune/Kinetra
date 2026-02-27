// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Riccati solver for the finite-horizon LQR problem.
// Used as the core backward pass of iLQR/DDP.

#pragma once

#include <vector>

#include "kinetra/core/types.hpp"

namespace kinetra::solvers {

/// Result of the LQR backward pass — feedback gains + feedforward terms.
struct LQRGains {
    std::vector<MatX> K;     // Feedback gains: u = K[t] * (x - x_ref) + k[t]
    std::vector<VecX> k;     // Feedforward terms
    std::vector<MatX> V;     // Value function Hessian at each time step
    std::vector<VecX> v;     // Value function gradient at each time step
    Scalar expectedImprovement{0};
};

/// Solve the finite-horizon discrete-time LQR problem via Riccati recursion.
///
/// Dynamics:  x_{k+1} = A_k x_k + B_k u_k
/// Cost:      sum_k [ 0.5 x_k' Q_k x_k + x_k' q_k + 0.5 u_k' R_k u_k + u_k' r_k ]
///            + 0.5 x_N' Q_N x_N + x_N' q_N
///
/// @param A  State transition matrices (T elements)
/// @param B  Control matrices (T elements)
/// @param Q  State cost matrices (T+1 elements, last is terminal)
/// @param q  State cost vectors (T+1 elements)
/// @param R  Control cost matrices (T elements)
/// @param r  Control cost vectors (T elements)
/// @param reg Regularization parameter (Levenberg-Marquardt style)
/// @return Feedback gains and value functions
[[nodiscard]] LQRGains solveLQR(
    const std::vector<MatX>& A,
    const std::vector<MatX>& B,
    const std::vector<MatX>& Q,
    const std::vector<VecX>& q,
    const std::vector<MatX>& R,
    const std::vector<VecX>& r,
    Scalar reg = static_cast<Scalar>(1e-6));

}  // namespace kinetra::solvers
