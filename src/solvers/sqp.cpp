// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/solvers/sqp.hpp"
#include "kinetra/solvers/qp_admm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace kinetra::solvers {

using Clock = std::chrono::steady_clock;

// ─── Static helpers ──────────────────────────────────────────────────────────

Scalar SQPSolver::constraintViolation(
    const VecX& g, const VecX& cl, const VecX& cu)
{
    Scalar viol = 0;
    for (int i = 0; i < g.size(); ++i) {
        if (g[i] < cl[i]) viol += cl[i] - g[i];
        if (g[i] > cu[i]) viol += g[i] - cu[i];
    }
    return viol;
}

Scalar SQPSolver::meritFunction(Scalar f, Scalar violation, Scalar mu) {
    return f + mu * violation;
}

void SQPSolver::bfgsUpdate(MatX& H, const VecX& s, const VecX& y) {
    Scalar sy = s.dot(y);
    // Skip update if curvature condition not satisfied (safeguard)
    if (sy < static_cast<Scalar>(1e-12) * s.squaredNorm()) return;

    // Standard BFGS: H_{k+1} = H - (H·s·sᵀ·H)/(sᵀ·H·s) + (y·yᵀ)/(yᵀ·s)
    VecX Hs = H * s;
    Scalar sHs = s.dot(Hs);

    if (std::abs(sHs) < static_cast<Scalar>(1e-16)) return;

    H += (y * y.transpose()) / sy - (Hs * Hs.transpose()) / sHs;
}

// ─── SQP Solve Loop ─────────────────────────────────────────────────────────

SQPResult SQPSolver::solve(optimization::NLPProblem& problem) {
    SQPResult result;
    auto t0 = Clock::now();
    total_qp_iters_ = 0;

    const int n = problem.numVariables();
    const int m = problem.numConstraints();

    if (n == 0) {
        result.exitMessage = "No variables in problem";
        result.converged = true;
        return result;
    }

    // Current iterate
    VecX x = problem.variableValues();
    VecX xl = problem.variableLowerBounds();
    VecX xu = problem.variableUpperBounds();

    // BFGS Hessian approximation — start with identity
    MatX H = MatX::Identity(n, n);

    // Cache constraint bounds (they never change during the solve)
    VecX cl = (m > 0) ? problem.constraintLowerBounds() : VecX{};
    VecX cu = (m > 0) ? problem.constraintUpperBounds() : VecX{};

    // Penalty parameter for L1 merit
    Scalar mu = settings_.initialMeritPenalty;

    // Previous Lagrangian gradient (for BFGS y-vector)
    VecX prev_grad_L = VecX::Zero(n);
    VecX prev_d = VecX::Zero(n);  // Previous QP step direction for warm-starting
    VecX prev_y_qp = VecX::Zero(m + n);  // Previous QP dual for warm-starting
    bool has_prev_grad = false;
    Scalar prev_cost = std::numeric_limits<Scalar>::max();

    for (int iter = 0; iter < settings_.maxIterations; ++iter) {
        // ── Time check ───────────────────────────────────────────────────────
        auto elapsed = std::chrono::duration<double, std::milli>(
            Clock::now() - t0).count();
        if (elapsed > settings_.timeLimitMs) {
            result.exitMessage = "Time limit exceeded";
            break;
        }

        // ── Evaluate NLP at current x ────────────────────────────────────────
        problem.setVariableValues(x);

        Scalar f  = problem.totalCost();
        VecX grad = problem.costGradient();

        VecX g  = (m > 0) ? problem.constraintValues() : VecX{};

        Scalar viol = (m > 0) ? constraintViolation(g, cl, cu) : 0;

        // ── Build QP sub-problem ─────────────────────────────────────────────
        //   min  grad'·d + 0.5·d'·H·d
        //   s.t. constraint linearization:  cl - g ≤ J·d ≤ cu - g
        //        variable bounds:           xl - x ≤  d  ≤ xu - x

        // Build sparse A_qp = [J; I_n] for efficient ADMM matrix-vector products.
        int m_qp = m + n;
        VecX lb_qp(m_qp), ub_qp(m_qp);

        // Assemble sparse A_qp from sparse Jacobian + identity for variable bounds
        SpMatX J_sp;
        if (m > 0) {
            J_sp = problem.constraintJacobianSparse();
            lb_qp.head(m) = cl - g;
            ub_qp.head(m) = cu - g;
        }

        // Build sparse A_qp = [J; I_n]
        std::vector<Triplet> triplets;
        triplets.reserve((m > 0 ? J_sp.nonZeros() : 0) + n);
        if (m > 0) {
            for (int k = 0; k < J_sp.outerSize(); ++k) {
                for (SpMatX::InnerIterator it(J_sp, k); it; ++it) {
                    triplets.emplace_back(
                        static_cast<int>(it.row()),
                        static_cast<int>(it.col()),
                        it.value());
                }
            }
        }
        for (int i = 0; i < n; ++i) {
            triplets.emplace_back(m + i, i, Scalar(1));
        }
        SpMatX A_qp(m_qp, n);
        A_qp.setFromTriplets(triplets.begin(), triplets.end());

        lb_qp.tail(n) = xl - x;
        ub_qp.tail(n) = xu - x;

        // Ensure H is positive definite (add regularization if needed)
        MatX H_reg = H;
        Scalar min_diag = H.diagonal().minCoeff();
        if (min_diag < static_cast<Scalar>(1e-8)) {
            H_reg += MatX::Identity(n, n) *
                     (static_cast<Scalar>(1e-8) - min_diag);
        }

        // Solve QP — progressively tighten tolerance for better convergence
        // Early SQP iterations use coarse QP; later iterations need precision
        QPSolverADMM qp;
        Scalar qp_tol = (iter < 5) ? Scalar(1e-3) :
                         (iter < 10) ? Scalar(5e-4) : Scalar(1e-4);
        qp.settings().maxIterations = (iter < 5) ? 200 : 400;
        qp.settings().absTolerance = qp_tol;
        qp.settings().relTolerance = qp_tol;

        if (!qp.setup(H_reg, grad, A_qp, lb_qp, ub_qp)) {
            result.exitMessage = "QP sub-problem setup failed";
            break;
        }

        // Warm-start QP with previous step direction if available
        if (has_prev_grad) {
            qp.warmStart(prev_d, prev_y_qp);
        }

        QPResult qp_result = qp.solve();
        VecX d = qp_result.x;
        prev_d = d;
        prev_y_qp = qp_result.y;
        total_qp_iters_ += qp_result.iterations;

        if (d.squaredNorm() < static_cast<Scalar>(1e-20)) {
            // Zero step → converged (or stuck)
            result.converged = (viol < settings_.constraintTolerance);
            result.exitMessage = result.converged ?
                "Converged (zero step)" : "Zero step, constraints not satisfied";
            break;
        }

        // ── Line search on L1 merit function ─────────────────────────────────
        Scalar merit_0 = meritFunction(f, viol, mu);

        // Directional derivative of merit (approximation)
        Scalar dmerit = grad.dot(d) - mu * viol;
        if (dmerit > 0) {
            // Increase penalty so direction is descent for merit
            mu = std::min(settings_.maxMeritPenalty,
                          (grad.dot(d) + static_cast<Scalar>(0.1)) / (viol + static_cast<Scalar>(1e-12)));
            merit_0 = meritFunction(f, viol, mu);
            dmerit = grad.dot(d) - mu * viol;
        }

        Scalar alpha = 1.0;
        VecX x_trial = x;
        bool ls_success = false;

        for (int ls = 0; ls < settings_.lineSearchMaxTrials; ++ls) {
            x_trial = x + alpha * d;
            // Clamp to variable bounds
            x_trial = x_trial.cwiseMax(xl).cwiseMin(xu);

            problem.setVariableValues(x_trial);
            Scalar f_trial = problem.totalCost();
            Scalar viol_trial = (m > 0) ?
                constraintViolation(problem.constraintValues(), cl, cu) : 0;
            Scalar merit_trial = meritFunction(f_trial, viol_trial, mu);

            if (merit_trial <= merit_0 + settings_.lineSearchAlpha * alpha * dmerit) {
                ls_success = true;
                break;
            }
            alpha *= settings_.lineSearchBeta;
        }

        if (!ls_success) {
            // Line search failed — try a reduced step to stay safer
            alpha = settings_.lineSearchBeta;
            x_trial = x + alpha * d;
            x_trial = x_trial.cwiseMax(xl).cwiseMin(xu);
            problem.setVariableValues(x_trial);
        }

        // ── BFGS update ─────────────────────────────────────────────────────
        VecX s = x_trial - x;
        VecX grad_new = problem.costGradient();

        // Use cost gradient for BFGS (more robust than Lagrangian gradient
        // when QP duals from ADMM are approximate / noisy)
        VecX grad_L_new = grad_new;

        if (has_prev_grad) {
            VecX y = grad_L_new - prev_grad_L;
            // Powell-style damped BFGS: ensure positive curvature
            Scalar sy = s.dot(y);
            Scalar sHs = s.dot(H * s);
            if (sy >= Scalar(0.2) * sHs) {
                // Standard BFGS (good curvature)
                bfgsUpdate(H, s, y);
            } else if (sHs > Scalar(1e-16)) {
                // Damped update: r = θ·y + (1-θ)·H·s  where θ keeps sᵀr ≥ 0.2·sᵀHs
                Scalar theta = Scalar(0.8) * sHs / (sHs - sy);
                VecX Hs = H * s;
                VecX r = theta * y + (Scalar(1) - theta) * Hs;
                bfgsUpdate(H, s, r);
            }
            // else: skip update (|sᵀHs| ≈ 0 → near-singular)
        }
        prev_grad_L = grad_L_new;
        has_prev_grad = true;

        // ── Accept step ─────────────────────────────────────────────────────
        x = x_trial;

        // ── Convergence check ────────────────────────────────────────────────
        // problem already has x_trial set from line search/BFGS gradient eval
        Scalar step_norm = s.norm();
        Scalar new_viol = (m > 0) ?
            constraintViolation(problem.constraintValues(), cl, cu) : 0;
        Scalar new_cost = problem.totalCost();

        result.iterations = iter + 1;
        result.cost = new_cost;
        result.constraintViolation = new_viol;

        if (step_norm < settings_.tolerance &&
            new_viol < settings_.constraintTolerance)
        {
            result.converged = true;
            result.exitMessage = "Converged";
            break;
        }

        // Also check if gradient is near zero and feasible (stationarity)
        // Reuse grad_new from BFGS update (already computed above)
        if (grad_new.norm() < settings_.tolerance &&
            new_viol < settings_.constraintTolerance)
        {
            result.converged = true;
            result.exitMessage = "Converged (gradient)";
            break;
        }

        // Feasible + small relative cost change ⇒ practically converged
        // For ADMM-based QP, exact KKT convergence is hard to achieve;
        // instead detect stagnation while feasible.
        if (iter >= 5 && new_viol < settings_.constraintTolerance) {
            Scalar rel_cost_change = std::abs(new_cost - prev_cost) /
                                     (std::abs(new_cost) + Scalar(1));
            if (rel_cost_change < Scalar(1e-3)) {
                result.converged = true;
                result.exitMessage = "Converged (feasible, small improvement)";
                break;
            }
        }

        prev_cost = new_cost;

        // Adaptive penalty: increase if constraint violation not decreasing
        if (new_viol > Scalar(0.9) * viol && viol > settings_.constraintTolerance) {
            mu = std::min(settings_.maxMeritPenalty,
                          mu * settings_.meritPenaltyGrowth);
        }
    }

    // ── Finalize ────────────────────────────────────────────────────────────
    problem.setVariableValues(x);
    result.x = x;
    result.totalQPIterations = total_qp_iters_;
    result.solveTimeMs = std::chrono::duration<double, std::milli>(
        Clock::now() - t0).count();

    if (result.iterations >= settings_.maxIterations && !result.converged) {
        result.exitMessage = "Max iterations reached";
    }

    return result;
}

}  // namespace kinetra::solvers
