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

}  // namespace kinetra::planners
