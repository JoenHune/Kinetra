// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// STOMP — Stochastic Trajectory Optimization for Motion Planning.
// Gradient-free optimization: generates noisy rollouts, weights by cost,
// updates trajectory as probability-weighted combination.

#pragma once

#include <functional>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra::planners {

struct STOMPOptions {
    int numTimesteps{60};           // Number of trajectory waypoints
    int numRollouts{30};            // Noisy trajectories per iteration
    int maxIterations{100};
    Scalar dt{static_cast<Scalar>(0.1)};
    Scalar controlCostWeight{static_cast<Scalar>(0.1)};
    Scalar noiseSigma{static_cast<Scalar>(0.5)};      // Noise standard deviation
    Scalar costSensitivity{static_cast<Scalar>(10.0)}; // Exponentiated cost sensitivity
    double timeLimitMs{5000.0};
};

/// Cost function type: given a trajectory (as N×3 matrix of [x,y,theta]),
/// returns per-waypoint costs as a vector.
using TrajectoryCostFn = std::function<VecX(const MatX& trajectory)>;

/// STOMP planner for trajectory optimization.
/// The algorithm:
///   1. Start with straight-line initial trajectory
///   2. Generate K noisy rollouts around current trajectory
///   3. Evaluate cost of each rollout
///   4. Compute probability-weighted combination of rollouts
///   5. Update trajectory using weighted combination
///   6. Repeat until convergence or max iterations
class STOMP {
public:
    STOMP() = default;
    explicit STOMP(STOMPOptions options) : options_(std::move(options)) {}

    /// Solve trajectory optimization.
    [[nodiscard]] PlanningResult solve(const PlanningProblem& problem);

    /// Set custom trajectory cost function
    void setCostFunction(TrajectoryCostFn cost_fn) { cost_fn_ = std::move(cost_fn); }

    [[nodiscard]] std::string_view name() const noexcept { return "STOMP"; }
    void reset();

    [[nodiscard]] const STOMPOptions& options() const noexcept { return options_; }
    STOMPOptions& options() noexcept { return options_; }

private:
    STOMPOptions options_;
    TrajectoryCostFn cost_fn_;

    // Internal: generate noise matrix for a rollout
    [[nodiscard]] MatX generateNoise(int rows, int cols) const;

    // Internal: compute smoothness cost matrix (finite-difference acceleration)
    [[nodiscard]] MatX smoothnessMatrix(int n) const;

    // Internal: compute probability weights from costs
    [[nodiscard]] VecX computeWeights(const VecX& costs) const;

    // Internal: create straight-line initial trajectory
    [[nodiscard]] MatX initializeTrajectory(const Waypoint2D& start,
                                             const Waypoint2D& goal) const;
};

}  // namespace kinetra::planners
