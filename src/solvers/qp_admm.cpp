// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/solvers/qp_admm.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <cmath>

namespace kinetra::solvers {

// ─── Dense A overload: convert to sparse and delegate ────────────────────────

bool QPSolverADMM::setup(const MatX& Q, const VecX& c, const MatX& A,
                          const VecX& lb, const VecX& ub) {
    n_ = static_cast<int>(Q.rows());
    m_ = static_cast<int>(A.rows());

    if (Q.cols() != n_ || c.size() != n_ || A.cols() != n_ ||
        lb.size() != m_ || ub.size() != m_) {
        return false;
    }

    Q_ = Q; c_ = c; lb_ = lb; ub_ = ub;
    A_sp_ = A.sparseView();
    A_sp_.makeCompressed();
    return setupInternal();
}

// ─── Sparse A overload ──────────────────────────────────────────────────────

bool QPSolverADMM::setup(const MatX& Q, const VecX& c, const SpMatX& A,
                          const VecX& lb, const VecX& ub) {
    n_ = static_cast<int>(Q.rows());
    m_ = static_cast<int>(A.rows());

    if (Q.cols() != n_ || c.size() != n_ || A.cols() != n_ ||
        lb.size() != m_ || ub.size() != m_) {
        return false;
    }

    Q_ = Q; c_ = c; lb_ = lb; ub_ = ub;
    A_sp_ = A;
    A_sp_.makeCompressed();
    return setupInternal();
}

// ─── Common setup ───────────────────────────────────────────────────────────

bool QPSolverADMM::setupInternal() {
    // Initialize ADMM variables
    x_ = VecX::Zero(n_);
    z_ = VecX::Zero(m_);
    y_ = VecX::Zero(m_);
    z_prev_ = z_;

    // Precompute transpose (sparse)
    AT_sp_ = SpMatX(A_sp_.transpose());

    // Precompute A'A as dense (it's added to the dense Q matrix anyway).
    // For the MPCC problem (n=124, m=207, ~2% density in A), this is still
    // fast because the sparse × sparse product exploits structure.
    ATA_ = MatX(AT_sp_ * A_sp_);

    // Auto-tune initial rho based on problem scaling
    if (settings_.rho <= Scalar(0.1) + Scalar(1e-12)) {
        Scalar trQ = Q_.trace();
        Scalar trATA = ATA_.trace();
        if (trATA > Scalar(1e-12) && trQ > Scalar(1e-12)) {
            settings_.rho = std::sqrt(trQ / Scalar(n_)) / std::sqrt(trATA / Scalar(m_));
            settings_.rho = clamp(settings_.rho, Scalar(1e-4), Scalar(1e4));
        }
    }

    cached_rho_ = 0;  // force initial factorization
    factorizeKKT();

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

    // LDLT factorization of KKT matrix (pre-factorized in setup/factorizeKKT)
    Eigen::LDLT<MatX> ldlt(kkt_factor_);

    for (int iter = 0; iter < settings_.maxIterations; ++iter) {
        // ── x-update: solve (Q + sigma*I + rho*A'A) x = -c + A'(rho*z - y) + sigma*x
        VecX rhs = -c_ + AT_sp_ * (settings_.rho * z_ - y_) + settings_.sigma * x_;
        x_ = ldlt.solve(rhs);

        // ── z-update with relaxation
        z_prev_ = z_;
        VecX ax = A_sp_ * x_;
        VecX z_hat = settings_.alpha * ax + (1 - settings_.alpha) * z_prev_
                     + y_ / settings_.rho;
        z_ = z_hat;
        projectOntoBox(z_, lb_, ub_);

        // ── y-update (dual)
        y_ = y_ + settings_.rho * (settings_.alpha * ax + (1 - settings_.alpha) * z_prev_ - z_);

        // ── Check convergence every 5 iterations (saves 60% of AT*dz products)
        if ((iter + 1) % 5 == 0 || iter == 0) {
            VecX primal_res = ax - z_;
            Scalar primal_norm = primal_res.norm();

            VecX dz = z_ - z_prev_;
            Scalar dual_norm = (settings_.rho * (AT_sp_ * dz)).norm();

            Scalar eps_primal = settings_.absTolerance * std::sqrt(static_cast<Scalar>(m_))
                                + settings_.relTolerance * std::max(ax.norm(), z_.norm());
            Scalar eps_dual = settings_.absTolerance * std::sqrt(static_cast<Scalar>(n_))
                              + settings_.relTolerance * std::max(
                                    settings_.rho * dz.norm(),
                                    y_.norm());

            result.primalResidual = primal_norm;
            result.dualResidual = dual_norm;

            if (primal_norm <= eps_primal && dual_norm <= eps_dual) {
                result.converged = true;
                result.iterations = iter + 1;
                break;
            }

            // ── Adaptive rho (only on check iterations)
            if (settings_.adaptiveRho) {
                bool changed = updateRho(primal_norm, dual_norm);
                if (changed) {
                    factorizeKKT();
                    ldlt.compute(kkt_factor_);
                }
            }
        }

        result.iterations = iter + 1;
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

void QPSolverADMM::factorizeKKT() {
    if (std::abs(settings_.rho - cached_rho_) < Scalar(1e-14) && cached_rho_ > 0)
        return;  // Already factorized for this rho
    kkt_factor_ = Q_ + settings_.sigma * MatX::Identity(n_, n_)
                  + settings_.rho * ATA_;
    cached_rho_ = settings_.rho;
}

bool QPSolverADMM::updateRho(Scalar primal_res, Scalar dual_res) {
    constexpr Scalar kMu = 10;
    constexpr Scalar kTauIncr = 2;
    constexpr Scalar kTauDecr = 2;

    Scalar old_rho = settings_.rho;
    if (primal_res > kMu * dual_res) {
        settings_.rho *= kTauIncr;
        y_ /= kTauIncr;
    } else if (dual_res > kMu * primal_res) {
        settings_.rho /= kTauDecr;
        y_ *= kTauDecr;
    }
    return settings_.rho != old_rho;
}

}  // namespace kinetra::solvers
