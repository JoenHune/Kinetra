// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// MPCC template implementation — included by mpcc.hpp.

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "kinetra/solvers/riccati_solver.hpp"

namespace kinetra::planners {

// ═════════════════════════════════════════════════════════════════════════════
// NLP Component: State Variables   x_0 ... x_N (flattened)
// ═════════════════════════════════════════════════════════════════════════════
namespace mpcc_nlp {

class MPCCStateVars : public optimization::VariableSet {
public:
    MPCCStateVars(int N, int nx, VecX lo, VecX hi)
        : VariableSet((N + 1) * nx, "states"), N_(N), nx_(nx),
          lo_(std::move(lo)), hi_(std::move(hi)) {}

    VecX lowerBounds() const override {
        VecX lb(size());
        for (int k = 0; k <= N_; ++k)
            lb.segment(k * nx_, nx_) = lo_;
        return lb;
    }
    VecX upperBounds() const override {
        VecX ub(size());
        for (int k = 0; k <= N_; ++k)
            ub.segment(k * nx_, nx_) = hi_;
        return ub;
    }

private:
    int N_, nx_;
    VecX lo_, hi_;
};

class MPCCControlVars : public optimization::VariableSet {
public:
    MPCCControlVars(int N, int nu, VecX lo, VecX hi)
        : VariableSet(N * nu, "controls"), N_(N), nu_(nu),
          lo_(std::move(lo)), hi_(std::move(hi)) {}

    VecX lowerBounds() const override {
        VecX lb(size());
        for (int k = 0; k < N_; ++k)
            lb.segment(k * nu_, nu_) = lo_;
        return lb;
    }
    VecX upperBounds() const override {
        VecX ub(size());
        for (int k = 0; k < N_; ++k)
            ub.segment(k * nu_, nu_) = hi_;
        return ub;
    }

private:
    int N_, nu_;
    VecX lo_, hi_;
};

class MPCCProgressVars : public optimization::VariableSet {
public:
    MPCCProgressVars(int N, Scalar s_min, Scalar s_max)
        : VariableSet(N + 1, "progress"), s_min_(s_min), s_max_(s_max) {}

    VecX lowerBounds() const override {
        return VecX::Constant(size(), s_min_);
    }
    VecX upperBounds() const override {
        return VecX::Constant(size(), s_max_);
    }

private:
    Scalar s_min_, s_max_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Dynamics Constraint: x_{k+1} = f(x_k, u_k, dt)
// ═════════════════════════════════════════════════════════════════════════════
template <typename Model>
class DynamicsConstraint : public optimization::ConstraintSet {
    static constexpr int NX = Model::kStateDim;
    static constexpr int NU = Model::kControlDim;

public:
    DynamicsConstraint(int N, const Model& model, Scalar dt)
        : ConstraintSet(N * NX, "dynamics"), N_(N), model_(model), dt_(dt) {}

    VecX evaluate() const override {
        VecX states = getVariableValues("states");
        VecX controls = getVariableValues("controls");
        VecX g(size());

        for (int k = 0; k < N_; ++k) {
            typename Model::StateType xk, xk1;
            typename Model::ControlType uk;
            for (int i = 0; i < NX; ++i) {
                xk[i] = states[k * NX + i];
                xk1[i] = states[(k + 1) * NX + i];
            }
            for (int i = 0; i < NU; ++i)
                uk[i] = controls[k * NU + i];

            auto xnext = model_.dynamics(xk, uk, dt_);
            for (int i = 0; i < NX; ++i)
                g[k * NX + i] = xk1[i] - xnext[i];
        }
        return g;
    }

    VecX lowerBound() const override { return VecX::Zero(size()); }
    VecX upperBound() const override { return VecX::Zero(size()); }

    void fillJacobianBlock(const std::string& var_name, MatX& jac) const override {
        VecX states = getVariableValues("states");
        VecX controls = getVariableValues("controls");

        if (var_name == "states") {
            jac.setZero();
            for (int k = 0; k < N_; ++k) {
                typename Model::StateType xk;
                typename Model::ControlType uk;
                for (int i = 0; i < NX; ++i) xk[i] = states[k * NX + i];
                for (int i = 0; i < NU; ++i) uk[i] = controls[k * NU + i];

                auto A = model_.jacobianState(xk, uk, dt_);
                // ∂g/∂x_k = -A
                for (int i = 0; i < NX; ++i)
                    for (int j = 0; j < NX; ++j)
                        jac(k * NX + i, k * NX + j) = -A(i, j);
                // ∂g/∂x_{k+1} = I
                for (int i = 0; i < NX; ++i)
                    jac(k * NX + i, (k + 1) * NX + i) = 1.0;
            }
        } else if (var_name == "controls") {
            jac.setZero();
            for (int k = 0; k < N_; ++k) {
                typename Model::StateType xk;
                typename Model::ControlType uk;
                for (int i = 0; i < NX; ++i) xk[i] = states[k * NX + i];
                for (int i = 0; i < NU; ++i) uk[i] = controls[k * NU + i];

                auto B = model_.jacobianControl(xk, uk, dt_);
                // ∂g/∂u_k = -B
                for (int i = 0; i < NX; ++i)
                    for (int j = 0; j < NU; ++j)
                        jac(k * NX + i, k * NU + j) = -B(i, j);
            }
        }
        // No dependency on progress variables
    }

private:
    int N_;
    Model model_;
    Scalar dt_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Initial State Constraint: x_0 = x_init
// ═════════════════════════════════════════════════════════════════════════════
class InitialStateConstraint : public optimization::ConstraintSet {
public:
    InitialStateConstraint(int nx, VecX x0)
        : ConstraintSet(nx, "init_state"), nx_(nx), x0_(std::move(x0)) {}

    VecX evaluate() const override {
        VecX states = getVariableValues("states");
        return states.head(nx_);
    }
    VecX lowerBound() const override { return x0_; }
    VecX upperBound() const override { return x0_; }

    void fillJacobianBlock(const std::string& var_name, MatX& jac) const override {
        if (var_name == "states") {
            jac.setZero();
            for (int i = 0; i < nx_; ++i)
                jac(i, i) = 1.0;
        }
    }

private:
    int nx_;
    VecX x0_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Progress Monotonicity + Rate Bound:  0 ≤ s_{k+1} - s_k ≤ v̄_s · dt
// ═════════════════════════════════════════════════════════════════════════════
class ProgressConstraint : public optimization::ConstraintSet {
public:
    ProgressConstraint(int N, Scalar maxRate, Scalar dt)
        : ConstraintSet(N, "progress_mono"), N_(N),
          maxDelta_(maxRate * dt) {}

    VecX evaluate() const override {
        VecX s = getVariableValues("progress");
        VecX g(N_);
        for (int k = 0; k < N_; ++k)
            g[k] = s[k + 1] - s[k];
        return g;
    }
    VecX lowerBound() const override { return VecX::Zero(N_); }
    VecX upperBound() const override { return VecX::Constant(N_, maxDelta_); }

    void fillJacobianBlock(const std::string& var_name, MatX& jac) const override {
        if (var_name == "progress") {
            jac.setZero();
            for (int k = 0; k < N_; ++k) {
                jac(k, k) = -1.0;      // ∂g/∂s_k
                jac(k, k + 1) = 1.0;   // ∂g/∂s_{k+1}
            }
        }
    }

private:
    int N_;
    Scalar maxDelta_;
};

// ═════════════════════════════════════════════════════════════════════════════
// SE(2) Contouring Cost:  Σ(w_c·e_c² + w_l·e_l² + w_θ·e_θ²) - λ·s_N
// ═════════════════════════════════════════════════════════════════════════════
class ContouringCost : public optimization::CostTerm {
public:
    ContouringCost(int N, int nx, const ReferencePath& path,
                   Scalar wc, Scalar wl, Scalar wth, Scalar lambda)
        : CostTerm("contouring"), N_(N), nx_(nx), path_(path),
          wc_(wc), wl_(wl), wth_(wth), lambda_(lambda) {}

    Scalar evaluate() const override {
        VecX states = getVariableValues("states");
        VecX progress = getVariableValues("progress");

        Scalar cost = 0;
        for (int k = 0; k <= N_; ++k) {
            Scalar xk = states[k * nx_];
            Scalar yk = states[k * nx_ + 1];
            Scalar thk = states[k * nx_ + 2];
            Scalar sk = progress[k];

            Vec3 ref = path_.evaluate(sk);
            Vec2 tangent = path_.tangent(sk);
            Vec2 normal = path_.normal(sk);
            Scalar th_ref = ref[2];

            Vec2 dp(xk - ref[0], yk - ref[1]);
            Scalar ec = normal.dot(dp);
            Scalar el = tangent.dot(dp);
            Scalar eth = normalizeAngle(thk - th_ref);

            cost += wc_ * ec * ec + wl_ * el * el + wth_ * eth * eth;
        }
        // Progress reward: maximise s_N → minimise -λ·s_N
        cost -= lambda_ * progress[N_];
        return cost;
    }

    void fillGradientBlock(const std::string& var_name, VecX& grad) const override {
        VecX states = getVariableValues("states");
        VecX progress = getVariableValues("progress");

        if (var_name == "states") {
            grad.setZero();
            for (int k = 0; k <= N_; ++k) {
                Scalar xk = states[k * nx_];
                Scalar yk = states[k * nx_ + 1];
                Scalar thk = states[k * nx_ + 2];
                Scalar sk = progress[k];

                Vec3 ref = path_.evaluate(sk);
                Vec2 tangent = path_.tangent(sk);
                Vec2 normal = path_.normal(sk);
                Scalar th_ref = ref[2];

                Vec2 dp(xk - ref[0], yk - ref[1]);
                Scalar ec = normal.dot(dp);
                Scalar el = tangent.dot(dp);
                Scalar eth = normalizeAngle(thk - th_ref);

                // ∂cost/∂x_k = 2·w_c·e_c·n_x + 2·w_l·e_l·t_x
                grad[k * nx_]     = 2.0 * wc_ * ec * normal.x()
                                  + 2.0 * wl_ * el * tangent.x();
                // ∂cost/∂y_k
                grad[k * nx_ + 1] = 2.0 * wc_ * ec * normal.y()
                                  + 2.0 * wl_ * el * tangent.y();
                // ∂cost/∂θ_k
                grad[k * nx_ + 2] = 2.0 * wth_ * eth;
            }
        } else if (var_name == "progress") {
            grad.setZero();
            for (int k = 0; k <= N_; ++k) {
                Scalar xk = states[k * nx_];
                Scalar yk = states[k * nx_ + 1];
                Scalar thk = states[k * nx_ + 2];
                Scalar sk = progress[k];

                Vec3 ref = path_.evaluate(sk);
                Vec2 tangent = path_.tangent(sk);
                Vec2 normal = path_.normal(sk);
                Scalar pdot_norm = std::sqrt(
                    sq(tangent.x()) + sq(tangent.y()));
                Scalar th_ref = ref[2];
                Scalar thdot = path_.thetaDot(sk);
                Scalar kappa = path_.curvature(sk);

                Vec2 dp(xk - ref[0], yk - ref[1]);
                Scalar ec = normal.dot(dp);
                Scalar el = tangent.dot(dp);
                Scalar eth = normalizeAngle(thk - th_ref);

                // Gradient of errors w.r.t. s_k (simplified, using §4.1 formulas)
                Scalar dec_ds = -normal.dot(tangent * pdot_norm)
                                - kappa * tangent.dot(dp);
                Scalar del_ds = -tangent.dot(tangent * pdot_norm)
                                + kappa * normal.dot(dp);
                Scalar deth_ds = -thdot;

                grad[k] = 2.0 * wc_ * ec * dec_ds
                         + 2.0 * wl_ * el * del_ds
                         + 2.0 * wth_ * eth * deth_ds;
            }
            // Progress reward gradient
            grad[N_] -= lambda_;
        }
        // No dependency on controls
    }

private:
    int N_, nx_;
    const ReferencePath& path_;
    Scalar wc_, wl_, wth_, lambda_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Control Cost:  Σ w_u · u_k^T · u_k
// ═════════════════════════════════════════════════════════════════════════════
class ControlCost : public optimization::CostTerm {
public:
    ControlCost(int N, int nu, Scalar weight)
        : CostTerm("control"), N_(N), nu_(nu), w_(weight) {}

    Scalar evaluate() const override {
        VecX controls = getVariableValues("controls");
        return w_ * controls.squaredNorm();
    }

    void fillGradientBlock(const std::string& var_name, VecX& grad) const override {
        if (var_name == "controls") {
            VecX controls = getVariableValues("controls");
            grad = 2.0 * w_ * controls;
        } else {
            grad.setZero();
        }
    }

private:
    int N_, nu_;
    Scalar w_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Obstacle Cost (soft penalty via SDF)
// ═════════════════════════════════════════════════════════════════════════════
class ObstacleCost : public optimization::CostTerm {
public:
    ObstacleCost(int N, int nx, const collision::OccupancyGrid2D& grid,
                 Scalar safeDistance, Scalar weight)
        : CostTerm("obstacle"), N_(N), nx_(nx), grid_(grid),
          dSafe_(safeDistance), w_(weight) {}

    Scalar evaluate() const override {
        VecX states = getVariableValues("states");
        Scalar cost = 0;
        for (int k = 0; k <= N_; ++k) {
            Vec2 p(states[k * nx_], states[k * nx_ + 1]);
            Scalar sd = grid_.signedDistance(p);
            if (sd < dSafe_) {
                Scalar viol = dSafe_ - sd;
                cost += w_ * viol * viol;
            }
        }
        return cost;
    }

    void fillGradientBlock(const std::string& var_name, VecX& grad) const override {
        if (var_name != "states") { grad.setZero(); return; }

        VecX states = getVariableValues("states");
        grad.setZero();
        Scalar eps = static_cast<Scalar>(0.01);

        for (int k = 0; k <= N_; ++k) {
            Vec2 p(states[k * nx_], states[k * nx_ + 1]);
            Scalar sd = grid_.signedDistance(p);
            if (sd < dSafe_) {
                Scalar viol = dSafe_ - sd;
                // Numerical gradient of signed distance
                Scalar sd_px = grid_.signedDistance(Vec2(p.x() + eps, p.y()));
                Scalar sd_py = grid_.signedDistance(Vec2(p.x(), p.y() + eps));
                Scalar dsd_dx = (sd_px - sd) / eps;
                Scalar dsd_dy = (sd_py - sd) / eps;
                grad[k * nx_]     = -2.0 * w_ * viol * dsd_dx;
                grad[k * nx_ + 1] = -2.0 * w_ * viol * dsd_dy;
            }
        }
    }

private:
    int N_, nx_;
    const collision::OccupancyGrid2D& grid_;
    Scalar dSafe_, w_;
};

}  // namespace mpcc_nlp

// ═════════════════════════════════════════════════════════════════════════════
// MPCC Implementation
// ═════════════════════════════════════════════════════════════════════════════

template <typename Model>
    requires LinearizableModel<Model>
MPCCResult MPCC<Model>::solve(const PlanningProblem& problem) {
    // Build a minimal grid if there are obstacles
    collision::OccupancyGrid2D grid(
        problem.environment.bounds_min.x(),
        problem.environment.bounds_min.y(),
        problem.environment.bounds_max.x(),
        problem.environment.bounds_max.y(),
        0.1);

    for (const auto& obs : problem.environment.obstacles) {
        std::visit([&](auto&& o) {
            using T = std::decay_t<decltype(o)>;
            if constexpr (std::is_same_v<T, CircleObstacle>)
                grid.addCircleObstacle(o.center, o.radius);
            else if constexpr (std::is_same_v<T, RectangleObstacle>)
                grid.addRectObstacle(o.min_corner, o.max_corner);
            else if constexpr (std::is_same_v<T, PolygonObstacle>)
                grid.addPolygonObstacle(o.vertices);
        }, obs);
    }

    return solve(problem, grid);
}

template <typename Model>
    requires LinearizableModel<Model>
MPCCResult MPCC<Model>::solve(const PlanningProblem& problem,
                               const collision::OccupancyGrid2D& grid) {
    auto t_start = std::chrono::steady_clock::now();

    // Build initial state vector from problem.start
    StateType x0 = StateType::Zero();
    x0[0] = static_cast<Scalar>(problem.start.x);
    x0[1] = static_cast<Scalar>(problem.start.y);
    x0[2] = static_cast<Scalar>(problem.start.theta);

    // Find initial progress along reference path
    Scalar s_init = Scalar(0);

    // ── Dispatch to Riccati or NLP-based solve ───────────────────────────
    if (options_.useRiccati) {
        return solveRiccati(x0, s_init, grid);
    }

    // Build and solve NLP
    auto nlp = buildNLP(x0, s_init, &grid);
    initialiseVariables(nlp, x0, s_init);

    solvers::SQPSolver sqp(options_.sqpSettings);
    auto sqpResult = sqp.solve(nlp);

    auto t_end = std::chrono::steady_clock::now();
    double totalMs = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return extractResult(nlp, sqpResult, totalMs);
}

template <typename Model>
    requires LinearizableModel<Model>
optimization::NLPProblem MPCC<Model>::buildNLP(
    const StateType& x0, Scalar s_init,
    const collision::OccupancyGrid2D* grid) const {

    const int N = options_.horizon;
    optimization::NLPProblem nlp;

    // ── State Variables ──────────────────────────────────────────────────
    VecX slo(kStateDim), shi(kStateDim);
    auto lb = model_.stateLowerBound();
    auto ub = model_.stateUpperBound();
    for (int i = 0; i < kStateDim; ++i) { slo[i] = lb[i]; shi[i] = ub[i]; }

    auto stateVars = std::make_shared<mpcc_nlp::MPCCStateVars>(N, kStateDim, slo, shi);
    nlp.addVariableSet(stateVars);

    // ── Control Variables ────────────────────────────────────────────────
    VecX clo(kControlDim), chi(kControlDim);
    auto clb = model_.controlLowerBound();
    auto cub = model_.controlUpperBound();
    for (int i = 0; i < kControlDim; ++i) { clo[i] = clb[i]; chi[i] = cub[i]; }

    auto controlVars = std::make_shared<mpcc_nlp::MPCCControlVars>(N, kControlDim, clo, chi);
    nlp.addVariableSet(controlVars);

    // ── Progress Variables ───────────────────────────────────────────────
    Scalar S = refPath_.totalLength();
    auto progressVars = std::make_shared<mpcc_nlp::MPCCProgressVars>(N, s_init, S);
    nlp.addVariableSet(progressVars);

    // ── Dynamics Constraint ──────────────────────────────────────────────
    auto dynCon = std::make_shared<mpcc_nlp::DynamicsConstraint<Model>>(
        N, model_, options_.dt);
    dynCon->linkVariables({stateVars, controlVars});
    nlp.addConstraintSet(dynCon);

    // ── Initial State Constraint ─────────────────────────────────────────
    VecX x0v(kStateDim);
    for (int i = 0; i < kStateDim; ++i) x0v[i] = x0[i];
    auto initCon = std::make_shared<mpcc_nlp::InitialStateConstraint>(kStateDim, x0v);
    initCon->linkVariables({stateVars});
    nlp.addConstraintSet(initCon);

    // ── Progress Constraints ─────────────────────────────────────────────
    auto progCon = std::make_shared<mpcc_nlp::ProgressConstraint>(
        N, options_.maxProgressRate, options_.dt);
    progCon->linkVariables({progressVars});
    nlp.addConstraintSet(progCon);

    // ── Contouring Cost ──────────────────────────────────────────────────
    auto contCost = std::make_shared<mpcc_nlp::ContouringCost>(
        N, kStateDim, refPath_,
        options_.wContour, options_.wLag, options_.wHeading, options_.wProgress);
    contCost->linkVariables({stateVars, progressVars});
    nlp.addCostSet(contCost);

    // ── Control Cost ─────────────────────────────────────────────────────
    auto ctrlCost = std::make_shared<mpcc_nlp::ControlCost>(
        N, kControlDim, options_.wControl);
    ctrlCost->linkVariables({controlVars});
    nlp.addCostSet(ctrlCost);

    // ── Obstacle Cost ────────────────────────────────────────────────────
    if (grid) {
        auto obsCost = std::make_shared<mpcc_nlp::ObstacleCost>(
            N, kStateDim, *grid, options_.safeDistance, options_.wObstacle);
        obsCost->linkVariables({stateVars});
        nlp.addCostSet(obsCost);
    }

    return nlp;
}

template <typename Model>
    requires LinearizableModel<Model>
void MPCC<Model>::initialiseVariables(
    optimization::NLPProblem& nlp,
    const StateType& x0, Scalar s_init) const {

    const int N = options_.horizon;
    Scalar S = refPath_.totalLength();

    // Total variable vector
    int nStates = (N + 1) * kStateDim;
    int nControls = N * kControlDim;
    int nProgress = N + 1;
    VecX allVars = VecX::Zero(nStates + nControls + nProgress);

    // Initialise states along reference path
    Scalar ds = std::min((S - s_init) / Scalar(N), options_.maxProgressRate * options_.dt);
    for (int k = 0; k <= N; ++k) {
        Scalar sk = s_init + k * ds;
        sk = clamp(sk, Scalar(0), S);
        Vec3 ref = refPath_.evaluate(sk);

        allVars[k * kStateDim]     = ref[0];  // x
        allVars[k * kStateDim + 1] = ref[1];  // y
        allVars[k * kStateDim + 2] = ref[2];  // θ
        // Higher-order states (v, ω, ...) stay at zero

        // Progress
        allVars[nStates + nControls + k] = sk;
    }

    // Controls: zero initialisation
    // (already zero from VecX::Zero)

    nlp.setVariableValues(allVars);
}

template <typename Model>
    requires LinearizableModel<Model>
MPCCResult MPCC<Model>::extractResult(
    const optimization::NLPProblem& nlp,
    const solvers::SQPResult& sqpResult,
    double totalTimeMs) const {

    const int N = options_.horizon;
    MPCCResult result;

    VecX vars = nlp.variableValues();
    int stateOffset = 0;
    int controlOffset = (N + 1) * kStateDim;
    int progressOffset = controlOffset + N * kControlDim;

    // Build trajectory
    Trajectory2D traj;
    for (int k = 0; k <= N; ++k) {
        Waypoint2D wp;
        wp.x = vars[stateOffset + k * kStateDim];
        wp.y = vars[stateOffset + k * kStateDim + 1];
        wp.theta = vars[stateOffset + k * kStateDim + 2];
        wp.t = k * options_.dt;
        traj.append(wp);
    }

    // Progress values
    result.progressValues.resize(N + 1);
    for (int k = 0; k <= N; ++k)
        result.progressValues[static_cast<std::size_t>(k)] = vars[progressOffset + k];

    // Error analysis
    result.contourErrors.resize(N + 1);
    result.lagErrors.resize(N + 1);
    result.headingErrors.resize(N + 1);
    for (int k = 0; k <= N; ++k) {
        Scalar xk = vars[stateOffset + k * kStateDim];
        Scalar yk = vars[stateOffset + k * kStateDim + 1];
        Scalar thk = vars[stateOffset + k * kStateDim + 2];
        Scalar sk = vars[progressOffset + k];

        Vec3 ref = refPath_.evaluate(sk);
        Vec2 tangent = refPath_.tangent(sk);
        Vec2 normal = refPath_.normal(sk);

        Vec2 dp(xk - ref[0], yk - ref[1]);
        result.contourErrors[static_cast<std::size_t>(k)] = normal.dot(dp);
        result.lagErrors[static_cast<std::size_t>(k)] = tangent.dot(dp);
        result.headingErrors[static_cast<std::size_t>(k)] =
            normalizeAngle(thk - ref[2]);
    }

    // Controls
    result.controls.resize(N);
    for (int k = 0; k < N; ++k) {
        VecX uk(kControlDim);
        for (int i = 0; i < kControlDim; ++i)
            uk[i] = vars[controlOffset + k * kControlDim + i];
        result.controls[static_cast<std::size_t>(k)] = uk;
    }

    // Fill PlanningResult
    auto& pr = result.planningResult;
    pr.trajectory = std::move(traj);
    pr.solveTimeMs = totalTimeMs;
    pr.iterations = sqpResult.iterations;
    pr.cost = sqpResult.cost;
    pr.plannerName = "MPCC";
    pr.pathLength = pr.trajectory.pathLength();
    pr.maxCurvature = pr.trajectory.maxCurvature();
    pr.smoothness = pr.trajectory.smoothness();
    pr.status = sqpResult.converged ? SolveStatus::kSuccess
                                    : SolveStatus::kMaxIterations;

    // MPCC-specific metadata
    result.sqpIterations = sqpResult.iterations;
    result.totalQPIterations = sqpResult.totalQPIterations;
    result.finalCost = sqpResult.cost;
    result.constraintViolation = sqpResult.constraintViolation;

    return result;
}

template <typename Model>
    requires LinearizableModel<Model>
void MPCC<Model>::reset() {
    // Nothing to reset for single-shot solve
}

// ═════════════════════════════════════════════════════════════════════════════
// GN-SQP with Riccati QP solve  (Phase-1 performance upgrade)
//
// Exploits the OCP block-banded structure of the MPCC formulation:
//   – Augmented state  x̃_k = [x_k; s_k]       ∈ ℝ^(nx+1)
//   – Augmented control ũ_k = [u_k; δs_k]      ∈ ℝ^(nu+1)
//   – Dynamics: [f(x_k,u_k,dt); s_k + δs_k]
//
// Per SQP iteration the QP sub-problem is solved via backward Riccati
// recursion in O(N · ñx³) instead of O((N·ñx)³) with dense ADMM.
// ═════════════════════════════════════════════════════════════════════════════

template <typename Model>
    requires LinearizableModel<Model>
MPCCResult MPCC<Model>::solveRiccati(
    const StateType& x0, Scalar s_init,
    const collision::OccupancyGrid2D& grid) const {

    using SteadyClock = std::chrono::steady_clock;
    auto t_start = SteadyClock::now();

    const int N  = options_.horizon;
    constexpr int nx  = static_cast<int>(kStateDim);
    constexpr int nu  = static_cast<int>(kControlDim);
    constexpr int anx = nx + 1;   // augmented state  (+progress)
    constexpr int anu = nu + 1;   // augmented control (+progress rate)
    const Scalar dt = options_.dt;
    const Scalar S  = refPath_.totalLength();

    // ── 1. Initialise trajectory along reference path ────────────────────
    std::vector<VecX> xa(static_cast<std::size_t>(N + 1), VecX::Zero(anx));
    std::vector<VecX> ua(static_cast<std::size_t>(N),     VecX::Zero(anu));

    Scalar ds = std::min((S - s_init) / Scalar(N),
                         options_.maxProgressRate * dt);

    // First state = problem start (not the reference spline value)
    for (int i = 0; i < nx; ++i) xa[0][i] = x0[i];
    xa[0][nx] = s_init;

    for (int k = 1; k <= N; ++k) {
        Scalar sk = s_init + Scalar(k) * ds;
        sk = clamp(sk, Scalar(0), S);
        Vec3 ref = refPath_.evaluate(sk);
        xa[static_cast<std::size_t>(k)][0] = ref[0];
        xa[static_cast<std::size_t>(k)][1] = ref[1];
        xa[static_cast<std::size_t>(k)][2] = ref[2];
        // Higher-order states (v, ω, …) stay at zero
        xa[static_cast<std::size_t>(k)][nx] = sk;
    }
    for (int k = 0; k < N; ++k) {
        ua[static_cast<std::size_t>(k)][nu] = ds;   // progress rate
    }

    // Pre-fetch control bounds (constant across iteration)
    auto clb = model_.controlLowerBound();
    auto cub = model_.controlUpperBound();

    // ── 2. Cost / defect evaluation lambdas ──────────────────────────────

    auto evalCost = [&](const std::vector<VecX>& x,
                        const std::vector<VecX>& u) -> Scalar {
        Scalar cost = 0;
        for (int k = 0; k <= N; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            Scalar sk = x[uk][nx];
            Vec3 ref = refPath_.evaluate(sk);
            Vec2 tan = refPath_.tangent(sk);
            Vec2 nor = refPath_.normal(sk);
            Vec2 dp(x[uk][0] - ref[0], x[uk][1] - ref[1]);
            Scalar ec  = nor.dot(dp);
            Scalar el  = tan.dot(dp);
            Scalar eth = normalizeAngle(x[uk][2] - ref[2]);
            cost += options_.wContour * ec * ec
                  + options_.wLag     * el * el
                  + options_.wHeading * eth * eth;
            // Obstacle
            Vec2 pk(x[uk][0], x[uk][1]);
            Scalar sd = grid.signedDistance(pk);
            if (sd < options_.safeDistance) {
                Scalar v = options_.safeDistance - sd;
                cost += options_.wObstacle * v * v;
            }
        }
        for (int k = 0; k < N; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            for (int i = 0; i < nu; ++i)
                cost += options_.wControl * u[uk][i] * u[uk][i];
        }
        cost -= options_.wProgress * x[static_cast<std::size_t>(N)][nx];
        return cost;
    };

    auto evalDefect = [&](const std::vector<VecX>& x,
                          const std::vector<VecX>& u) -> Scalar {
        Scalar def = 0;
        for (int k = 0; k < N; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            StateType xk;
            ControlType uctl;
            for (int i = 0; i < nx; ++i) xk[i] = x[uk][i];
            for (int i = 0; i < nu; ++i) uctl[i] = u[uk][i];
            auto xnext = model_.dynamics(xk, uctl, dt);
            for (int i = 0; i < nx; ++i)
                def += std::abs(xnext[i] - x[uk + 1][i]);
            def += std::abs(x[uk][nx] + u[uk][nu] - x[uk + 1][nx]);
        }
        return def;
    };

    // ── 3. SQP outer loop ────────────────────────────────────────────────
    int sqpIter = 0;
    bool converged = false;
    Scalar prevMerit = std::numeric_limits<Scalar>::max();
    const int maxIter = options_.sqpSettings.maxIterations;
    const Scalar tolStep = options_.sqpSettings.tolerance;
    const Scalar tolCon  = options_.sqpSettings.constraintTolerance;

    for (sqpIter = 0; sqpIter < maxIter; ++sqpIter) {
        // Time limit
        double elapsed = std::chrono::duration<double, std::milli>(
            SteadyClock::now() - t_start).count();
        if (elapsed > options_.sqpSettings.timeLimitMs) break;

        // ── 3a. Build OCP QP stages ─────────────────────────────────
        solvers::OCPProblem ocp;
        ocp.nx = anx;
        ocp.nu = anu;
        ocp.N  = N;
        ocp.stages.resize(static_cast<std::size_t>(N));
        ocp.dx_0 = VecX::Zero(anx);   // initial state is fixed

        for (int k = 0; k < N; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            auto& stg = ocp.stages[uk];

            StateType xk;
            ControlType uctl;
            for (int i = 0; i < nx; ++i) xk[i] = xa[uk][i];
            for (int i = 0; i < nu; ++i) uctl[i] = ua[uk][i];
            Scalar sk  = xa[uk][nx];

            // ── Linearised dynamics ─────────────────────────────────
            auto A_r = model_.jacobianState(xk, uctl, dt);
            auto B_r = model_.jacobianControl(xk, uctl, dt);
            auto x_pred = model_.dynamics(xk, uctl, dt);

            stg.A = MatX::Zero(anx, anx);
            stg.A.topLeftCorner(nx, nx) = A_r;
            stg.A(nx, nx) = Scalar(1);

            stg.B = MatX::Zero(anx, anu);
            stg.B.topLeftCorner(nx, nu) = B_r;
            stg.B(nx, nu) = Scalar(1);

            stg.c = VecX::Zero(anx);
            for (int i = 0; i < nx; ++i)
                stg.c[i] = x_pred[i] - xa[uk + 1][i];
            stg.c[nx] = sk + ua[uk][nu] - xa[uk + 1][nx];

            // ── GN Hessian from contouring cost ─────────────────────
            Vec3 ref = refPath_.evaluate(sk);
            Vec2 tan = refPath_.tangent(sk);
            Vec2 nor = refPath_.normal(sk);
            Scalar kappa    = refPath_.curvature(sk);
            Scalar thdot    = refPath_.thetaDot(sk);
            Scalar pdot_n   = tan.norm();

            Vec2 dp(xa[uk][0] - ref[0], xa[uk][1] - ref[1]);
            Scalar ec  = nor.dot(dp);
            Scalar el  = tan.dot(dp);
            Scalar eth = normalizeAngle(xa[uk][2] - ref[2]);

            // Residual r = [√wc ec, √wl el, √wθ eθ]
            Eigen::Matrix<Scalar, 3, 1> res;
            Scalar swc = std::sqrt(options_.wContour);
            Scalar swl = std::sqrt(options_.wLag);
            Scalar swh = std::sqrt(options_.wHeading);
            res[0] = swc * ec;
            res[1] = swl * el;
            res[2] = swh * eth;

            // Jacobian J_r (3 × anx)
            MatX Jr = MatX::Zero(3, anx);
            Jr(0, 0) = swc * nor.x();
            Jr(0, 1) = swc * nor.y();
            Jr(1, 0) = swl * tan.x();
            Jr(1, 1) = swl * tan.y();
            Jr(2, 2) = swh;

            // ∂errors/∂s
            Scalar dec_ds = -nor.dot(tan * pdot_n) - kappa * tan.dot(dp);
            Scalar del_ds = -tan.dot(tan * pdot_n) + kappa * nor.dot(dp);
            Scalar deth_ds = -thdot;
            Jr(0, nx) = swc * dec_ds;
            Jr(1, nx) = swl * del_ds;
            Jr(2, nx) = swh * deth_ds;

            stg.Q = Scalar(2) * Jr.transpose() * Jr;    // anx × anx
            stg.q = Scalar(2) * Jr.transpose() * res;   // anx

            // ── Obstacle contribution ───────────────────────────────
            Vec2 pk(xa[uk][0], xa[uk][1]);
            Scalar sd = grid.signedDistance(pk);
            if (sd < options_.safeDistance) {
                Scalar viol = options_.safeDistance - sd;
                Vec2 gsd   = grid.sdfGradient(pk);
                Scalar sw   = std::sqrt(options_.wObstacle);
                VecX Jb = VecX::Zero(anx);
                Jb[0] = -sw * gsd.x();
                Jb[1] = -sw * gsd.y();
                stg.Q += Scalar(2) * Jb * Jb.transpose();
                stg.q += Scalar(2) * sw * viol * Jb;
            }

            // Regularise Q
            stg.Q.diagonal().array() += Scalar(1e-6);

            // ── Control cost ────────────────────────────────────────
            stg.R = Scalar(2) * options_.wControl
                  * MatX::Identity(anu, anu);
            stg.R(nu, nu) = Scalar(1e-4);  // small reg for progress rate
            stg.r = VecX::Zero(anu);
            for (int i = 0; i < nu; ++i)
                stg.r[i] = Scalar(2) * options_.wControl * ua[uk][i];

            // ── Box constraints (correction space) ──────────────────
            stg.u_lb = VecX(anu);
            stg.u_ub = VecX(anu);
            for (int i = 0; i < nu; ++i) {
                stg.u_lb[i] = clb[i] - ua[uk][i];
                stg.u_ub[i] = cub[i] - ua[uk][i];
            }
            stg.u_lb[nu] = -ua[uk][nu];
            stg.u_ub[nu] = options_.maxProgressRate * dt - ua[uk][nu];
        }

        // ── Terminal cost ────────────────────────────────────────────
        {
            const auto uN = static_cast<std::size_t>(N);
            Scalar sN = xa[uN][nx];
            Vec3 refN = refPath_.evaluate(sN);
            Vec2 tanN = refPath_.tangent(sN);
            Vec2 norN = refPath_.normal(sN);
            Scalar kN     = refPath_.curvature(sN);
            Scalar thdotN = refPath_.thetaDot(sN);
            Scalar pdotN  = tanN.norm();

            Vec2 dpN(xa[uN][0] - refN[0], xa[uN][1] - refN[1]);
            Scalar ecN  = norN.dot(dpN);
            Scalar elN  = tanN.dot(dpN);
            Scalar ethN = normalizeAngle(xa[uN][2] - refN[2]);

            Scalar swc = std::sqrt(options_.wContour);
            Scalar swl = std::sqrt(options_.wLag);
            Scalar swh = std::sqrt(options_.wHeading);

            Eigen::Matrix<Scalar, 3, 1> resN;
            resN[0] = swc * ecN;
            resN[1] = swl * elN;
            resN[2] = swh * ethN;

            MatX JrN = MatX::Zero(3, anx);
            JrN(0, 0) = swc * norN.x();
            JrN(0, 1) = swc * norN.y();
            JrN(1, 0) = swl * tanN.x();
            JrN(1, 1) = swl * tanN.y();
            JrN(2, 2) = swh;

            Scalar decN = -norN.dot(tanN * pdotN) - kN * tanN.dot(dpN);
            Scalar delN = -tanN.dot(tanN * pdotN) + kN * norN.dot(dpN);
            Scalar dethN = -thdotN;
            JrN(0, nx) = swc * decN;
            JrN(1, nx) = swl * delN;
            JrN(2, nx) = swh * dethN;

            ocp.Q_N = Scalar(2) * JrN.transpose() * JrN;
            ocp.q_N = Scalar(2) * JrN.transpose() * resN;
            ocp.q_N[nx] -= options_.wProgress;  // progress reward

            // Obstacle at terminal
            Vec2 pN(xa[uN][0], xa[uN][1]);
            Scalar sdN = grid.signedDistance(pN);
            if (sdN < options_.safeDistance) {
                Scalar vN = options_.safeDistance - sdN;
                Vec2 gsN  = grid.sdfGradient(pN);
                Scalar sw = std::sqrt(options_.wObstacle);
                VecX JbN = VecX::Zero(anx);
                JbN[0] = -sw * gsN.x();
                JbN[1] = -sw * gsN.y();
                ocp.Q_N += Scalar(2) * JbN * JbN.transpose();
                ocp.q_N += Scalar(2) * sw * vN * JbN;
            }
            ocp.Q_N.diagonal().array() += Scalar(1e-6);
        }

        // ── 3b. Solve QP via Riccati ────────────────────────────────
        solvers::RiccatiSolver riccati;
        auto qpSol = riccati.solve(ocp);

        // ── 3c. Line search ─────────────────────────────────────────
        Scalar mu    = Scalar(10);
        Scalar cost0 = evalCost(xa, ua);
        Scalar def0  = evalDefect(xa, ua);
        Scalar merit0 = cost0 + mu * def0;

        Scalar alpha = 1;
        bool   ls_ok = false;
        std::vector<VecX> xa_try(static_cast<std::size_t>(N + 1));
        std::vector<VecX> ua_try(static_cast<std::size_t>(N));

        for (int ls = 0; ls < 10; ++ls) {
            for (int k = 0; k <= N; ++k) {
                const auto uk = static_cast<std::size_t>(k);
                xa_try[uk] = xa[uk] + alpha * qpSol.dx[uk];
                xa_try[uk][nx] = clamp(xa_try[uk][nx], Scalar(0), S);
            }
            for (int k = 0; k < N; ++k) {
                const auto uk = static_cast<std::size_t>(k);
                ua_try[uk] = ua[uk] + alpha * qpSol.du[uk];
                ua_try[uk][nu] = clamp(ua_try[uk][nu], Scalar(0),
                                       options_.maxProgressRate * dt);
                for (int i = 0; i < nu; ++i)
                    ua_try[uk][i] = clamp(ua_try[uk][i], clb[i], cub[i]);
            }
            Scalar merit_try = evalCost(xa_try, ua_try)
                             + mu * evalDefect(xa_try, ua_try);
            if (merit_try < merit0) { ls_ok = true; break; }
            alpha *= Scalar(0.5);
        }

        if (!ls_ok) {                // accept a small step anyway
            alpha = Scalar(0.05);
            for (int k = 0; k <= N; ++k) {
                const auto uk = static_cast<std::size_t>(k);
                xa_try[uk] = xa[uk] + alpha * qpSol.dx[uk];
                xa_try[uk][nx] = clamp(xa_try[uk][nx], Scalar(0), S);
            }
            for (int k = 0; k < N; ++k) {
                const auto uk = static_cast<std::size_t>(k);
                ua_try[uk] = ua[uk] + alpha * qpSol.du[uk];
                ua_try[uk][nu] = clamp(ua_try[uk][nu], Scalar(0),
                                       options_.maxProgressRate * dt);
                for (int i = 0; i < nu; ++i)
                    ua_try[uk][i] = clamp(ua_try[uk][i], clb[i], cub[i]);
            }
        }

        // ── 3d. Accept step ─────────────────────────────────────────
        Scalar step_norm = 0;
        for (int k = 0; k <= N; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            step_norm += (xa_try[uk] - xa[uk]).squaredNorm();
        }
        for (int k = 0; k < N; ++k) {
            const auto uk = static_cast<std::size_t>(k);
            step_norm += (ua_try[uk] - ua[uk]).squaredNorm();
        }
        step_norm = std::sqrt(step_norm);

        xa = std::move(xa_try);
        ua = std::move(ua_try);

        // ── 3e. Convergence check ───────────────────────────────────
        Scalar newMerit = evalCost(xa, ua) + mu * evalDefect(xa, ua);
        Scalar newDef   = evalDefect(xa, ua);

        if (step_norm < tolStep && newDef < tolCon) {
            converged = true;  break;
        }
        if (sqpIter >= 3) {
            Scalar rel = std::abs(newMerit - prevMerit)
                       / (std::abs(newMerit) + Scalar(1));
            if (rel < Scalar(1e-4) && newDef < tolCon) {
                converged = true;  break;
            }
        }
        prevMerit = newMerit;
    }

    // ── 4. Build MPCCResult ──────────────────────────────────────────────
    auto t_end = SteadyClock::now();
    double totalMs = std::chrono::duration<double, std::milli>(
        t_end - t_start).count();

    MPCCResult result;
    Trajectory2D traj;
    for (int k = 0; k <= N; ++k) {
        const auto uk = static_cast<std::size_t>(k);
        Waypoint2D wp;
        wp.x = xa[uk][0];
        wp.y = xa[uk][1];
        wp.theta = xa[uk][2];
        wp.t = Scalar(k) * dt;
        traj.append(wp);
    }

    result.progressValues.resize(static_cast<std::size_t>(N + 1));
    for (int k = 0; k <= N; ++k)
        result.progressValues[static_cast<std::size_t>(k)] =
            xa[static_cast<std::size_t>(k)][nx];

    result.contourErrors.resize(static_cast<std::size_t>(N + 1));
    result.lagErrors.resize(static_cast<std::size_t>(N + 1));
    result.headingErrors.resize(static_cast<std::size_t>(N + 1));
    for (int k = 0; k <= N; ++k) {
        const auto uk = static_cast<std::size_t>(k);
        Scalar sk = xa[uk][nx];
        Vec3 ref = refPath_.evaluate(sk);
        Vec2 tan = refPath_.tangent(sk);
        Vec2 nor = refPath_.normal(sk);
        Vec2 dp(xa[uk][0] - ref[0], xa[uk][1] - ref[1]);
        result.contourErrors[uk] = nor.dot(dp);
        result.lagErrors[uk]     = tan.dot(dp);
        result.headingErrors[uk] = normalizeAngle(xa[uk][2] - ref[2]);
    }

    result.controls.resize(static_cast<std::size_t>(N));
    for (int k = 0; k < N; ++k) {
        const auto uk = static_cast<std::size_t>(k);
        VecX uout(kControlDim);
        for (int i = 0; i < nu; ++i) uout[i] = ua[uk][i];
        result.controls[uk] = uout;
    }

    auto& pr = result.planningResult;
    pr.trajectory   = std::move(traj);
    pr.solveTimeMs  = totalMs;
    pr.iterations   = sqpIter + 1;
    pr.cost         = evalCost(xa, ua);
    pr.plannerName  = "MPCC";
    pr.pathLength   = pr.trajectory.pathLength();
    pr.maxCurvature = pr.trajectory.maxCurvature();
    pr.smoothness   = pr.trajectory.smoothness();
    pr.status       = converged ? SolveStatus::kSuccess
                                : SolveStatus::kMaxIterations;

    result.sqpIterations    = sqpIter + 1;
    result.totalQPIterations = sqpIter + 1;  // 1 Riccati per SQP iter
    result.finalCost        = pr.cost;
    result.constraintViolation = evalDefect(xa, ua);

    return result;
}

}  // namespace kinetra::planners
