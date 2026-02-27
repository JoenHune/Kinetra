// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// iLQR (iterative Linear Quadratic Regulator) planner.
// Works with any model satisfying the LinearizableModel concept.

#pragma once

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/types.hpp"
#include "kinetra/solvers/lqr.hpp"

namespace kinetra::planners {

struct iLQROptions {
    int maxIterations{100};
    Scalar tolerance{static_cast<Scalar>(1e-6)};
    Scalar initialRegularization{static_cast<Scalar>(1.0)};
    Scalar regFactor{static_cast<Scalar>(10.0)};   // Multiply/divide reg by this
    Scalar regMin{static_cast<Scalar>(1e-8)};
    Scalar regMax{static_cast<Scalar>(1e6)};
    int horizon{50};                                 // Number of time steps
    Scalar dt{static_cast<Scalar>(0.05)};
    double timeLimitMs{5000.0};

    // Line search parameters
    Scalar alphaMin{static_cast<Scalar>(1e-4)};
    int lineSearchMaxIter{10};
    Scalar lineSearchBeta{static_cast<Scalar>(0.5)};  // Backtracking factor
};

/// iLQR planner — works with any LinearizableModel.
/// The algorithm:
///   1. Forward rollout with current control sequence
///   2. Backward pass: compute LQR gains via Riccati recursion
///   3. Forward pass with line search: apply gains to improve trajectory
///   4. Repeat until convergence
///
/// Template parameter Model must satisfy LinearizableModel concept.
template <typename Model>
    requires LinearizableModel<Model>
class iLQR {
public:
    using StateType = typename Model::StateType;
    using ControlType = typename Model::ControlType;

    iLQR(Model model, iLQROptions options = {})
        : model_(std::move(model)), options_(std::move(options)) {}

    /// Solve trajectory optimization from start to goal.
    [[nodiscard]] PlanningResult solve(const PlanningProblem& problem);

    /// Solve with explicit state/control cost matrices.
    /// Q: state cost (per step), R: control cost (per step),
    /// Qf: terminal state cost, x_ref: reference state trajectory.
    struct CostMatrices {
        MatX Q;    // State cost weight
        MatX R;    // Control cost weight
        MatX Qf;   // Terminal state cost weight
        std::vector<StateType> x_ref;  // Reference trajectory (optional)
    };

    [[nodiscard]] PlanningResult solveWithCost(
        const StateType& x0, const StateType& x_goal,
        const CostMatrices& costs);

    [[nodiscard]] std::string_view name() const noexcept { return "iLQR"; }
    void reset();

    // Access optimized trajectory and controls
    [[nodiscard]] const std::vector<StateType>& stateTrajectory() const { return x_traj_; }
    [[nodiscard]] const std::vector<ControlType>& controlTrajectory() const { return u_traj_; }

    [[nodiscard]] const iLQROptions& options() const noexcept { return options_; }
    iLQROptions& options() noexcept { return options_; }

private:
    Model model_;
    iLQROptions options_;

    // Trajectories
    std::vector<StateType> x_traj_;
    std::vector<ControlType> u_traj_;

    // Internal: forward rollout with current controls
    void forwardRollout(const StateType& x0);

    // Internal: compute quadratic cost approximation around current trajectory
    void computeCostExpansion(const CostMatrices& costs, const StateType& x_goal,
                              std::vector<MatX>& Q_xx, std::vector<VecX>& q_x,
                              std::vector<MatX>& R_uu, std::vector<VecX>& r_u);

    // Internal: forward pass with line search
    Scalar forwardPassWithLineSearch(const solvers::LQRGains& gains,
                                     const StateType& x0,
                                     const CostMatrices& costs,
                                     const StateType& x_goal);
};

}  // namespace kinetra::planners

// Template implementation
#include "kinetra/planners/ilqr_impl.hpp"
