// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Optimal Control Problem (OCP) data structures for structured QP solvers.
//
// Stage-wise QP formulation used by GN-SQP + Riccati:
//
//   min  Σ_{k=0}^{N-1} [½ δx_k' Q_k δx_k + q_k' δx_k
//                       + ½ δu_k' R_k δu_k + r_k' δu_k]
//        + ½ δx_N' Q_N δx_N + q_N' δx_N
//
//   s.t. δx_{k+1} = A_k δx_k + B_k δu_k + c_k    (linearized dynamics)
//        u_lb_k  ≤ δu_k ≤ u_ub_k                  (control box constraints)
//        δx_0 = 0                                   (initial state fixed)

#pragma once

#include <vector>

#include "kinetra/core/types.hpp"

namespace kinetra::solvers {

/// Single stage of a discrete-time OCP QP sub-problem.
struct OCPStage {
    // Linearized dynamics: δx_{k+1} = A · δx_k + B · δu_k + c
    MatX A;   // nx × nx   (state Jacobian)
    MatX B;   // nx × nu   (control Jacobian)
    VecX c;   // nx         (dynamics defect / affine term)

    // Stage cost: ½ δx' Q δx + q' δx + ½ δu' R δu + r' δu
    MatX Q;   // nx × nx   (PSD — Gauss-Newton Hessian of state cost)
    VecX q;   // nx         (cost gradient w.r.t. state correction)
    MatX R;   // nu × nu   (PD — control cost Hessian)
    VecX r;   // nu         (cost gradient w.r.t. control correction)

    // Box constraints on control corrections
    VecX u_lb;  // nu (lower bound on δu)
    VecX u_ub;  // nu (upper bound on δu)
};

/// Full OCP QP problem: N stages + terminal cost.
struct OCPProblem {
    int nx{0};             // augmented state dimension
    int nu{0};             // augmented control dimension
    int N{0};              // number of stages (= number of control intervals)

    std::vector<OCPStage> stages;  // stages[0 .. N-1]

    // Terminal cost: ½ δx_N' Q_N δx_N + q_N' δx_N
    MatX Q_N;  // nx × nx
    VecX q_N;  // nx

    // Initial state correction (zero for fixed initial state)
    VecX dx_0;  // nx
};

/// Solution of an OCP QP.
struct OCPSolution {
    std::vector<VecX> dx;  // state corrections: dx[0 .. N], each ∈ R^nx
    std::vector<VecX> du;  // control corrections: du[0 .. N-1], each ∈ R^nu
    Scalar cost{0};        // optimal QP cost
};

}  // namespace kinetra::solvers
