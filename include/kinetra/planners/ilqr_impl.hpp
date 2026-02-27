// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// iLQR template implementation — included from ilqr.hpp.
// Kept separate for readability; this file should NOT be included directly.

#pragma once

#include "kinetra/planners/ilqr.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>

namespace kinetra::planners {

// ─── reset ──────────────────────────────────────────────────────────────────

template <typename Model>
    requires LinearizableModel<Model>
void iLQR<Model>::reset() {
    x_traj_.clear();
    u_traj_.clear();
}

// ─── solve (PlanningProblem → PlanningResult) ───────────────────────────────

template <typename Model>
    requires LinearizableModel<Model>
PlanningResult iLQR<Model>::solve(const PlanningProblem& problem) {
    constexpr int nx = static_cast<int>(Model::kStateDim);
    constexpr int nu = static_cast<int>(Model::kControlDim);

    // Build default cost matrices: Q = I, R = 0.01*I, Qf = 100*I
    CostMatrices costs;
    costs.Q  = MatX::Identity(nx, nx);
    costs.R  = static_cast<Scalar>(0.01) * MatX::Identity(nu, nu);
    costs.Qf = static_cast<Scalar>(100.0) * MatX::Identity(nx, nx);

    // Map start/goal from Waypoint2D → model StateType
    StateType x0 = StateType::Zero();
    x0[0] = problem.start.x;
    x0[1] = problem.start.y;
    if constexpr (nx >= 3) {
        x0[2] = problem.start.theta;
    }

    StateType x_goal = StateType::Zero();
    x_goal[0] = problem.goal.x;
    x_goal[1] = problem.goal.y;
    if constexpr (nx >= 3) {
        x_goal[2] = problem.goal.theta;
    }

    return solveWithCost(x0, x_goal, costs);
}

// ─── solveWithCost (the real algorithm) ──────────────────────────────────────

template <typename Model>
    requires LinearizableModel<Model>
PlanningResult iLQR<Model>::solveWithCost(
    const StateType& x0, const StateType& x_goal,
    const CostMatrices& costs)
{
    namespace ch = std::chrono;
    auto t_start = ch::steady_clock::now();

    constexpr int nx = static_cast<int>(Model::kStateDim);
    constexpr int nu = static_cast<int>(Model::kControlDim);
    int T = options_.horizon;

    PlanningResult result;
    result.plannerName = "iLQR";

    // ── 1. Initialize control trajectory (zeros) ─────────────────────────
    u_traj_.assign(static_cast<std::size_t>(T), ControlType::Zero());
    forwardRollout(x0);

    // ── 2. Compute initial cost ──────────────────────────────────────────
    auto computeTotalCost = [&](const std::vector<StateType>& xs,
                                const std::vector<ControlType>& us) -> Scalar {
        Scalar total = 0;
        for (int t = 0; t < T; ++t) {
            auto ti = static_cast<std::size_t>(t);
            VecX dx = xs[ti] - x_goal;
            VecX du = us[ti];
            total += static_cast<Scalar>(0.5) * dx.dot(costs.Q * dx);
            total += static_cast<Scalar>(0.5) * du.dot(costs.R * du);
        }
        VecX dx_final = xs[static_cast<std::size_t>(T)] - x_goal;
        total += static_cast<Scalar>(0.5) * dx_final.dot(costs.Qf * dx_final);
        return total;
    };

    Scalar prev_cost = computeTotalCost(x_traj_, u_traj_);
    Scalar reg = options_.initialRegularization;

    // ── 3. Main iteration loop ───────────────────────────────────────────
    int iter = 0;
    for (; iter < options_.maxIterations; ++iter) {
        // Check time limit
        auto now = ch::steady_clock::now();
        double elapsed_ms = ch::duration<double, std::milli>(now - t_start).count();
        if (elapsed_ms > options_.timeLimitMs) {
            result.status = SolveStatus::kTimeout;
            break;
        }

        // ── 3a. Backward pass: linearize + LQR ──────────────────────────
        std::vector<MatX> A_vec(static_cast<std::size_t>(T));
        std::vector<MatX> B_vec(static_cast<std::size_t>(T));
        std::vector<MatX> Q_vec(static_cast<std::size_t>(T + 1));
        std::vector<VecX> q_vec(static_cast<std::size_t>(T + 1));
        std::vector<MatX> R_vec(static_cast<std::size_t>(T));
        std::vector<VecX> r_vec(static_cast<std::size_t>(T));

        for (int t = 0; t < T; ++t) {
            auto ti = static_cast<std::size_t>(t);
            A_vec[ti] = model_.jacobianState(x_traj_[ti], u_traj_[ti], options_.dt);
            B_vec[ti] = model_.jacobianControl(x_traj_[ti], u_traj_[ti], options_.dt);

            // Quadratic cost expansion around (x_ref, 0 control)
            VecX dx = x_traj_[ti] - x_goal;
            Q_vec[ti] = costs.Q;
            q_vec[ti] = costs.Q * dx;
            R_vec[ti] = costs.R;
            r_vec[ti] = costs.R * u_traj_[ti];
        }
        // Terminal cost
        VecX dx_final = x_traj_[static_cast<std::size_t>(T)] - x_goal;
        Q_vec[static_cast<std::size_t>(T)] = costs.Qf;
        q_vec[static_cast<std::size_t>(T)] = costs.Qf * dx_final;

        solvers::LQRGains gains = solvers::solveLQR(
            A_vec, B_vec, Q_vec, q_vec, R_vec, r_vec, reg);

        // ── 3b. Forward pass with line search ────────────────────────────
        Scalar alpha = static_cast<Scalar>(1.0);
        bool improved = false;

        for (int ls = 0; ls < options_.lineSearchMaxIter; ++ls) {
            std::vector<StateType> x_new(static_cast<std::size_t>(T + 1));
            std::vector<ControlType> u_new(static_cast<std::size_t>(T));
            x_new[0] = x0;

            for (int t = 0; t < T; ++t) {
                auto ti = static_cast<std::size_t>(t);
                VecX dx = x_new[ti] - x_traj_[ti];
                ControlType du = gains.K[ti] * dx + alpha * gains.k[ti];
                u_new[ti] = u_traj_[ti] + du;

                // Clamp controls
                ControlType u_lb = model_.controlLowerBound();
                ControlType u_ub = model_.controlUpperBound();
                for (int i = 0; i < nu; ++i) {
                    u_new[ti][i] = std::clamp(u_new[ti][i], u_lb[i], u_ub[i]);
                }

                x_new[ti + 1] = model_.dynamics(x_new[ti], u_new[ti], options_.dt);
            }

            Scalar new_cost = computeTotalCost(x_new, u_new);

            if (new_cost < prev_cost) {
                x_traj_ = std::move(x_new);
                u_traj_ = std::move(u_new);
                prev_cost = new_cost;
                improved = true;

                // Decrease regularization on success
                reg = std::max(reg / options_.regFactor, options_.regMin);
                break;
            }

            alpha *= options_.lineSearchBeta;
        }

        if (!improved) {
            // Increase regularization
            reg = std::min(reg * options_.regFactor, options_.regMax);
            if (reg >= options_.regMax) {
                // Converged (can't improve further)
                break;
            }
        }

        // ── 3c. Convergence check ───────────────────────────────────────
        if (improved && std::abs(gains.expectedImprovement) < options_.tolerance) {
            result.status = SolveStatus::kSuccess;
            break;
        }
    }

    // ── 4. Finalize result ───────────────────────────────────────────────
    auto t_end = ch::steady_clock::now();
    result.solveTimeMs = ch::duration<double, std::milli>(t_end - t_start).count();
    result.iterations = iter;
    result.cost = prev_cost;

    if (result.status == SolveStatus::kNotSolved) {
        // Check if we actually reached the goal
        VecX dx_goal = x_traj_.back() - x_goal;
        Scalar pos_err = std::sqrt(dx_goal[0] * dx_goal[0] + dx_goal[1] * dx_goal[1]);
        if (pos_err < static_cast<Scalar>(0.5)) {
            result.status = SolveStatus::kSuccess;
        } else {
            result.status = SolveStatus::kMaxIterations;
        }
    }

    // Build Trajectory2D from state trajectory
    result.trajectory.clear();
    result.trajectory.reserve(x_traj_.size());
    for (std::size_t i = 0; i < x_traj_.size(); ++i) {
        Waypoint2D wp;
        wp.x = x_traj_[i][0];
        wp.y = x_traj_[i][1];
        if constexpr (nx >= 3) {
            wp.theta = x_traj_[i][2];
        }
        wp.t = static_cast<Scalar>(i) * options_.dt;
        result.trajectory.append(wp);
    }
    result.pathLength = result.trajectory.pathLength();
    result.maxCurvature = result.trajectory.maxCurvature();
    result.smoothness = result.trajectory.smoothness();

    return result;
}

// ─── forwardRollout ─────────────────────────────────────────────────────────

template <typename Model>
    requires LinearizableModel<Model>
void iLQR<Model>::forwardRollout(const StateType& x0) {
    int T = options_.horizon;
    x_traj_.resize(static_cast<std::size_t>(T + 1));
    x_traj_[0] = x0;

    for (int t = 0; t < T; ++t) {
        auto ti = static_cast<std::size_t>(t);
        x_traj_[ti + 1] = model_.dynamics(x_traj_[ti], u_traj_[ti], options_.dt);
    }
}

// ─── computeCostExpansion ───────────────────────────────────────────────────

template <typename Model>
    requires LinearizableModel<Model>
void iLQR<Model>::computeCostExpansion(
    const CostMatrices& costs, const StateType& x_goal,
    std::vector<MatX>& Q_xx, std::vector<VecX>& q_x,
    std::vector<MatX>& R_uu, std::vector<VecX>& r_u)
{
    int T = options_.horizon;
    Q_xx.resize(static_cast<std::size_t>(T + 1));
    q_x.resize(static_cast<std::size_t>(T + 1));
    R_uu.resize(static_cast<std::size_t>(T));
    r_u.resize(static_cast<std::size_t>(T));

    for (int t = 0; t < T; ++t) {
        auto ti = static_cast<std::size_t>(t);
        VecX dx = x_traj_[ti] - x_goal;
        Q_xx[ti] = costs.Q;
        q_x[ti]  = costs.Q * dx;
        R_uu[ti] = costs.R;
        r_u[ti]  = costs.R * u_traj_[ti];
    }
    VecX dx_final = x_traj_[static_cast<std::size_t>(T)] - x_goal;
    Q_xx[static_cast<std::size_t>(T)] = costs.Qf;
    q_x[static_cast<std::size_t>(T)]  = costs.Qf * dx_final;
}

// ─── forwardPassWithLineSearch ──────────────────────────────────────────────

template <typename Model>
    requires LinearizableModel<Model>
Scalar iLQR<Model>::forwardPassWithLineSearch(
    const solvers::LQRGains& gains,
    const StateType& x0,
    const CostMatrices& costs,
    const StateType& x_goal)
{
    constexpr int nu = static_cast<int>(Model::kControlDim);
    int T = options_.horizon;
    Scalar alpha = static_cast<Scalar>(1.0);

    auto computeCost = [&](const std::vector<StateType>& xs,
                           const std::vector<ControlType>& us) -> Scalar {
        Scalar total = 0;
        for (int t = 0; t < T; ++t) {
            auto ti = static_cast<std::size_t>(t);
            VecX dx = xs[ti] - x_goal;
            VecX du = us[ti];
            total += static_cast<Scalar>(0.5) * dx.dot(costs.Q * dx);
            total += static_cast<Scalar>(0.5) * du.dot(costs.R * du);
        }
        VecX dx_f = xs[static_cast<std::size_t>(T)] - x_goal;
        total += static_cast<Scalar>(0.5) * dx_f.dot(costs.Qf * dx_f);
        return total;
    };

    Scalar best_cost = computeCost(x_traj_, u_traj_);

    for (int ls = 0; ls < options_.lineSearchMaxIter; ++ls) {
        std::vector<StateType> x_new(static_cast<std::size_t>(T + 1));
        std::vector<ControlType> u_new(static_cast<std::size_t>(T));
        x_new[0] = x0;

        for (int t = 0; t < T; ++t) {
            auto ti = static_cast<std::size_t>(t);
            VecX dx = x_new[ti] - x_traj_[ti];
            ControlType du = gains.K[ti] * dx + alpha * gains.k[ti];
            u_new[ti] = u_traj_[ti] + du;

            ControlType u_lb = model_.controlLowerBound();
            ControlType u_ub = model_.controlUpperBound();
            for (int i = 0; i < nu; ++i) {
                u_new[ti][i] = std::clamp(u_new[ti][i], u_lb[i], u_ub[i]);
            }
            x_new[ti + 1] = model_.dynamics(x_new[ti], u_new[ti], options_.dt);
        }

        Scalar new_cost = computeCost(x_new, u_new);
        if (new_cost < best_cost) {
            x_traj_ = std::move(x_new);
            u_traj_ = std::move(u_new);
            return new_cost;
        }
        alpha *= options_.lineSearchBeta;
    }
    return best_cost;
}

}  // namespace kinetra::planners
