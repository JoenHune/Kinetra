// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/solvers/qp_admm.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

namespace kinetra::solvers {

bool QPSolverADMM::setup(const MatX& Q, const VecX& c, const MatX& A,
                          const VecX& lb, const VecX& ub) {
    n_ = static_cast<int>(Q.rows());
    m_ = static_cast<int>(A.rows());

    if (Q.cols() != n_ || c.size() != n_ || A.cols() != n_ ||
        lb.size() != m_ || ub.size() != m_) {
        return false;
    }

    Q_ = Q; c_ = c; A_ = A; lb_ = lb; ub_ = ub;

    // Initialize ADMM variables
    x_ = VecX::Zero(n_);
    z_ = VecX::Zero(m_);
    y_ = VecX::Zero(m_);
    z_prev_ = z_;

    // Factorize KKT matrix: (Q + sigma*I + rho * A' * A)
    MatX M = Q_ + settings_.sigma * MatX::Identity(n_, n_)
             + settings_.rho * A_.transpose() * A_;
    kkt_factor_ = M;  // TODO: Use LDLT factorization for efficiency

    is_setup_ = true;
    return true;
}

QPResult QPSolverADMM::solve() {
    QPResult result;
    if (!is_setup_) {
        result.converged = false;
        return result;
    }

    auto start = Clock::now();

    // LDLT factorization of KKT matrix
    Eigen::LDLT<MatX> ldlt(kkt_factor_);

    for (int iter = 0; iter < settings_.maxIterations; ++iter) {
        // ── x-update: solve (Q + sigma*I + rho*A'A) x = -c + A'(rho*z - y) + sigma*x
        VecX rhs = -c_ + A_.transpose() * (settings_.rho * z_ - y_) + settings_.sigma * x_;
        x_ = ldlt.solve(rhs);

        // ── z-update with relaxation
        z_prev_ = z_;
        VecX ax = A_ * x_;
        VecX z_hat = settings_.alpha * ax + (1 - settings_.alpha) * z_prev_
                     + y_ / settings_.rho;
        z_ = z_hat;
        projectOntoBox(z_, lb_, ub_);

        // ── y-update (dual)
        y_ = y_ + settings_.rho * (settings_.alpha * ax + (1 - settings_.alpha) * z_prev_ - z_);

        // ── Check convergence
        VecX primal_res = ax - z_;
        VecX dual_res = settings_.rho * A_.transpose() * (z_ - z_prev_);

        Scalar primal_norm = primal_res.norm();
        Scalar dual_norm = dual_res.norm();

        Scalar eps_primal = settings_.absTolerance * std::sqrt(static_cast<Scalar>(m_))
                            + settings_.relTolerance * std::max(ax.norm(), z_.norm());
        Scalar eps_dual = settings_.absTolerance * std::sqrt(static_cast<Scalar>(n_))
                          + settings_.relTolerance * (A_.transpose() * y_).norm();

        if (primal_norm <= eps_primal && dual_norm <= eps_dual) {
            result.converged = true;
            result.iterations = iter + 1;
            result.primalResidual = primal_norm;
            result.dualResidual = dual_norm;
            break;
        }

        // ── Adaptive rho
        if (settings_.adaptiveRho) {
            updateRho(primal_norm, dual_norm);
            // Re-factorize if rho changed significantly
            MatX M = Q_ + settings_.sigma * MatX::Identity(n_, n_)
                     + settings_.rho * A_.transpose() * A_;
            ldlt.compute(M);
        }

        result.iterations = iter + 1;
        result.primalResidual = primal_norm;
        result.dualResidual = dual_norm;
    }

    result.x = x_;
    result.y = y_;
    result.objectiveValue = static_cast<Scalar>(0.5) * x_.dot(Q_ * x_) + c_.dot(x_);

    auto end = Clock::now();
    result.solveTimeMs = std::chrono::duration<double, std::milli>(end - start).count();

    return result;
}

void QPSolverADMM::warmStart(const VecX& x0, const VecX& y0) {
    if (x0.size() == n_) x_ = x0;
    if (y0.size() == m_) y_ = y0;
}

void QPSolverADMM::updateLinearCost(const VecX& c) {
    if (c.size() == n_) c_ = c;
}

void QPSolverADMM::updateBounds(const VecX& lb, const VecX& ub) {
    if (lb.size() == m_) lb_ = lb;
    if (ub.size() == m_) ub_ = ub;
}

void QPSolverADMM::projectOntoBox(VecX& z, const VecX& lb, const VecX& ub) const {
    z = z.cwiseMax(lb).cwiseMin(ub);
}

void QPSolverADMM::updateRho(Scalar primal_res, Scalar dual_res) {
    constexpr Scalar kMu = 10;
    constexpr Scalar kTauIncr = 2;
    constexpr Scalar kTauDecr = 2;

    if (primal_res > kMu * dual_res) {
        settings_.rho *= kTauIncr;
        y_ /= kTauIncr;
    } else if (dual_res > kMu * primal_res) {
        settings_.rho /= kTauDecr;
        y_ *= kTauDecr;
    }
}

}  // namespace kinetra::solvers
