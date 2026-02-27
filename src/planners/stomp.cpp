// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/planners/stomp.hpp"

#include <Eigen/Cholesky>
#include <chrono>
#include <cmath>
#include <random>

namespace kinetra::planners {

PlanningResult STOMP::solve(const PlanningProblem& problem) {
    PlanningResult result;
    result.plannerName = "STOMP";
    auto start_time = Clock::now();

    int N = options_.numTimesteps;
    int K = options_.numRollouts;

    // Initialize straight-line trajectory (N x 3 matrix: [x, y, theta])
    MatX trajectory = initializeTrajectory(problem.start, problem.goal);

    // Smoothness inverse R^{-1} and its Cholesky for correlated noise generation.
    // Sampling ε = σ * L * z (z ~ N(0,I)) produces smooth noise with
    // covariance σ² R^{-1}, matching the STOMP prior distribution.
    MatX R_inv = smoothnessMatrix(N);
    Eigen::LLT<MatX> llt(R_inv);
    MatX L = llt.matrixL();  // R^{-1} = L L^T

    // Default cost function: obstacle distance
    auto cost_fn = cost_fn_ ? cost_fn_ :
        [](const MatX& traj) -> VecX { return VecX::Zero(traj.rows()); };

    Scalar best_cost = cost_fn(trajectory).sum();
    MatX best_trajectory = trajectory;

    for (int iter = 0; iter < options_.maxIterations; ++iter) {
        auto elapsed = std::chrono::duration<double, std::milli>(
            Clock::now() - start_time).count();
        if (elapsed > options_.timeLimitMs) {
            result.status = SolveStatus::kTimeout;
            break;
        }

        // Generate K noisy rollouts and evaluate costs
        VecX rollout_costs(K);
        std::vector<MatX> noises(static_cast<std::size_t>(K));

        for (int k = 0; k < K; ++k) {
            auto ki = static_cast<std::size_t>(k);
            if (k == 0) {
                // Include current trajectory (zero noise) as a baseline rollout.
                // This anchors the weighted average, preventing drift when
                // state costs are flat (e.g. empty environments).
                noises[ki] = MatX::Zero(N, 3);
            } else {
                // Generate smooth (correlated) noise via Cholesky of R^{-1}
                MatX z = generateNoise(N, 3);
                noises[ki] = options_.noiseSigma * (L * z);
                noises[ki].row(0).setZero();
                noises[ki].row(N - 1).setZero();
            }

            // Apply noise to get rollout
            MatX rollout = trajectory + noises[ki];

            // Evaluate cost: state cost + control regularization
            VecX state_costs = cost_fn(rollout);
            Scalar control_cost = options_.controlCostWeight *
                noises[ki].squaredNorm();
            rollout_costs[k] = state_costs.sum() + control_cost;
        }

        // Compute probability weights
        VecX weights = computeWeights(rollout_costs);

        // Compute weighted combination of noises.
        // No additional R^{-1} smoothing needed — noise is already smooth.
        MatX delta = MatX::Zero(N, 3);
        for (int k = 0; k < K; ++k) {
            delta += weights[k] * noises[static_cast<std::size_t>(k)];
        }

        // Update trajectory
        trajectory += delta;

        // Pin start and goal
        trajectory.row(0) << problem.start.x, problem.start.y, problem.start.theta;
        trajectory.row(N - 1) << problem.goal.x, problem.goal.y, problem.goal.theta;

        // Track best
        VecX current_costs = cost_fn(trajectory);
        Scalar total_cost = current_costs.sum();
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_trajectory = trajectory;
        }

        // Convergence: average per-element squared change is small
        if (delta.squaredNorm() / static_cast<Scalar>(N * 3)
            < static_cast<Scalar>(1e-6)) {
            break;
        }
    }

    // Convert to Trajectory2D
    Trajectory2D traj;
    traj.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
        traj.append({
            best_trajectory(i, 0),
            best_trajectory(i, 1),
            best_trajectory(i, 2),
            static_cast<Scalar>(i) * options_.dt
        });
    }

    auto end_time = Clock::now();
    result.solveTimeMs = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    result.trajectory = traj;
    result.cost = best_cost;
    result.pathLength = traj.pathLength();

    if (result.status != SolveStatus::kTimeout) {
        result.status = SolveStatus::kSuccess;
    }
    return result;
}

void STOMP::reset() {
    // No persistent state to reset
}

MatX STOMP::generateNoise(int rows, int cols) const {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<Scalar> dist(0, options_.noiseSigma);
    MatX noise(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            noise(i, j) = dist(gen);
        }
    }
    return noise;
}

MatX STOMP::smoothnessMatrix(int n) const {
    // Finite-difference matrix for acceleration (second derivative)
    // A = tridiagonal(1, -2, 1), R = A' * A
    // Return R^{-1} for smoothing updates
    MatX A = MatX::Zero(n - 2, n);
    for (int i = 0; i < n - 2; ++i) {
        A(i, i)     =  1;
        A(i, i + 1) = -2;
        A(i, i + 2) =  1;
    }
    Scalar scale = static_cast<Scalar>(1.0) / (options_.dt * options_.dt);
    MatX R = scale * A.transpose() * A + static_cast<Scalar>(1e-6) * MatX::Identity(n, n);

    // Return inverse for smoothing
    return R.inverse();
}

VecX STOMP::computeWeights(const VecX& costs) const {
    Scalar min_cost = costs.minCoeff();
    Scalar max_cost = costs.maxCoeff();
    Scalar range = max_cost - min_cost;
    if (range < constants::kEpsilon) range = 1;

    // Exponential weighting
    VecX exp_costs = (-options_.costSensitivity * (costs.array() - min_cost) / range).exp();
    Scalar sum = exp_costs.sum();
    if (sum < constants::kEpsilon) {
        return VecX::Constant(costs.size(), static_cast<Scalar>(1.0) / static_cast<Scalar>(costs.size()));
    }
    return exp_costs / sum;
}

MatX STOMP::initializeTrajectory(const Waypoint2D& start, const Waypoint2D& goal) const {
    int N = options_.numTimesteps;
    MatX trajectory(N, 3);
    for (int i = 0; i < N; ++i) {
        Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(N - 1);
        trajectory(i, 0) = lerp(start.x, goal.x, t);
        trajectory(i, 1) = lerp(start.y, goal.y, t);
        trajectory(i, 2) = start.theta + t * angularDistance(start.theta, goal.theta);
    }
    return trajectory;
}

}  // namespace kinetra::planners
