// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <cmath>
#include "kinetra/solvers/sqp.hpp"

using namespace kinetra;
using namespace kinetra::optimization;
using namespace kinetra::solvers;

// ── Concrete test helpers (anonymous namespace to avoid ODR collisions) ──────

namespace {

/// 2D variables with optional box bounds
class TestVars2D : public VariableSet {
public:
    TestVars2D() : VariableSet(2, "x") {}
    TestVars2D(Scalar lo, Scalar hi) : VariableSet(2, "x"), lo_(lo), hi_(hi) {}

    VecX lowerBounds() const override { return VecX::Constant(2, lo_); }
    VecX upperBounds() const override { return VecX::Constant(2, hi_); }

private:
    Scalar lo_{-constants::kInfinity};
    Scalar hi_{constants::kInfinity};
};

/// Quadratic cost: 0.5 * x' Q x + c' x
class QuadraticCost : public CostTerm {
public:
    QuadraticCost(MatX Q, VecX c)
        : CostTerm("quad"), Q_(std::move(Q)), c_(std::move(c)) {}

    Scalar evaluate() const override {
        VecX x = getVariableValues("x");
        return Scalar(0.5) * x.dot(Q_ * x) + c_.dot(x);
    }
    void fillGradientBlock(const std::string&, VecX& grad) const override {
        VecX x = getVariableValues("x");
        grad = Q_ * x + c_;
    }

private:
    MatX Q_;
    VecX c_;
};

/// Rosenbrock cost: (a - x0)² + b*(x1 - x0²)²
class RosenbrockCost : public CostTerm {
public:
    RosenbrockCost(Scalar a = 1.0, Scalar b = 100.0)
        : CostTerm("rosenbrock"), a_(a), b_(b) {}

    Scalar evaluate() const override {
        VecX x = getVariableValues("x");
        Scalar t1 = a_ - x[0];
        Scalar t2 = x[1] - x[0] * x[0];
        return t1 * t1 + b_ * t2 * t2;
    }
    void fillGradientBlock(const std::string&, VecX& grad) const override {
        VecX x = getVariableValues("x");
        Scalar t2 = x[1] - x[0] * x[0];
        grad[0] = -2.0 * (a_ - x[0]) - 4.0 * b_ * x[0] * t2;
        grad[1] = 2.0 * b_ * t2;
    }

private:
    Scalar a_, b_;
};

/// Linear constraint: A*x (bounds set externally)
class LinearConstraint : public ConstraintSet {
public:
    LinearConstraint(MatX A, VecX lb, VecX ub)
        : ConstraintSet(static_cast<int>(A.rows()), "lin_con"),
          A_(std::move(A)), lb_(std::move(lb)), ub_(std::move(ub)) {}

    VecX evaluate() const override {
        return A_ * getVariableValues("x");
    }
    VecX lowerBound() const override { return lb_; }
    VecX upperBound() const override { return ub_; }
    void fillJacobianBlock(const std::string&, MatX& jac) const override {
        jac = A_;
    }

private:
    MatX A_;
    VecX lb_, ub_;
};

/// Nonlinear constraint: x0² + x1² (for circle constraint)
class CircleConstraint : public ConstraintSet {
public:
    CircleConstraint(Scalar lb, Scalar ub)
        : ConstraintSet(1, "circle"), lb_(lb), ub_(ub) {}

    VecX evaluate() const override {
        VecX x = getVariableValues("x");
        VecX g(1);
        g[0] = x[0] * x[0] + x[1] * x[1];
        return g;
    }
    VecX lowerBound() const override { VecX v(1); v[0] = lb_; return v; }
    VecX upperBound() const override { VecX v(1); v[0] = ub_; return v; }
    void fillJacobianBlock(const std::string&, MatX& jac) const override {
        VecX x = getVariableValues("x");
        jac(0, 0) = 2.0 * x[0];
        jac(0, 1) = 2.0 * x[1];
    }

private:
    Scalar lb_, ub_;
};

}  // anonymous namespace

// ── Tests ────────────────────────────────────────────────────────────────────

TEST(SQPSolver, UnconstrainedQuadratic) {
    // min 0.5*(x0² + x1²) + [-2, -3]*x
    // Solution: x = [2, 3]
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>();
    VecX x0(2); x0 << 0, 0;
    vars->setValues(x0);

    MatX Q = MatX::Identity(2, 2);
    VecX c(2); c << -2, -3;
    auto cost = std::make_shared<QuadraticCost>(Q, c);
    cost->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addCostSet(cost);

    SQPSolver solver;
    auto result = solver.solve(problem);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 2.0, 1e-3);
    EXPECT_NEAR(result.x[1], 3.0, 1e-3);
}

TEST(SQPSolver, BoxConstrainedQuadratic) {
    // min 0.5*(x0² + x1²) + [-5, -5]*x  s.t.  0 ≤ x ≤ 3
    // Unconstrained solution: [5, 5], box-constrained: [3, 3]
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>(0.0, 3.0);
    VecX x0(2); x0 << 1, 1;
    vars->setValues(x0);

    MatX Q = MatX::Identity(2, 2);
    VecX c(2); c << -5, -5;
    auto cost = std::make_shared<QuadraticCost>(Q, c);
    cost->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addCostSet(cost);

    SQPSolver solver;
    auto result = solver.solve(problem);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 3.0, 1e-2);
    EXPECT_NEAR(result.x[1], 3.0, 1e-2);
}

TEST(SQPSolver, LinearConstraint) {
    // min 0.5*(x0² + x1²)  s.t.  x0 + x1 = 1
    // Solution: x = [0.5, 0.5]
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>();
    VecX x0(2); x0 << 0, 1;
    vars->setValues(x0);

    MatX Q = MatX::Identity(2, 2);
    VecX c = VecX::Zero(2);
    auto cost = std::make_shared<QuadraticCost>(Q, c);
    cost->linkVariables({vars});

    MatX A(1, 2); A << 1, 1;
    VecX lb(1); lb << 1;
    VecX ub(1); ub << 1;
    auto con = std::make_shared<LinearConstraint>(A, lb, ub);
    con->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addConstraintSet(con);
    problem.addCostSet(cost);

    SQPSolver solver;
    auto result = solver.solve(problem);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 0.5, 1e-2);
    EXPECT_NEAR(result.x[1], 0.5, 1e-2);
    EXPECT_LT(result.constraintViolation, 1e-4);
}

TEST(SQPSolver, NonlinearConstraint) {
    // min 0.5*(x0² + x1²) - 2*x0  s.t.  x0² + x1² ≤ 1
    // Unconstrained minimizer at x = [2, 0]; constraint pushes to x = [1, 0]
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>();
    VecX x0(2); x0 << 0.5, 0.5;
    vars->setValues(x0);

    MatX Q = MatX::Identity(2, 2);
    VecX c(2); c << -2, 0;
    auto cost = std::make_shared<QuadraticCost>(Q, c);
    cost->linkVariables({vars});

    auto con = std::make_shared<CircleConstraint>(0.0, 1.0);
    con->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addConstraintSet(con);
    problem.addCostSet(cost);

    SQPSolver solver;
    solver.settings().tolerance = 1e-4;
    solver.settings().maxIterations = 200;
    auto result = solver.solve(problem);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 1.0, 0.1);
    EXPECT_NEAR(result.x[1], 0.0, 0.1);
    EXPECT_LT(result.constraintViolation, 1e-2);
}

TEST(SQPSolver, Rosenbrock) {
    // min (1-x0)² + 100*(x1-x0²)²
    // Solution: x = [1, 1]
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>();
    VecX x0(2); x0 << -1, 1;
    vars->setValues(x0);

    auto cost = std::make_shared<RosenbrockCost>();
    cost->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addCostSet(cost);

    SQPSettings settings;
    settings.maxIterations = 500;
    settings.tolerance = 1e-5;

    SQPSolver solver(settings);
    auto result = solver.solve(problem);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 1.0, 0.05);
    EXPECT_NEAR(result.x[1], 1.0, 0.05);
}

TEST(SQPSolver, EmptyProblem) {
    NLPProblem problem;
    SQPSolver solver;
    auto result = solver.solve(problem);
    EXPECT_TRUE(result.converged);
}

TEST(SQPSolver, FeasibilityFromInfeasibleStart) {
    // min 0.5*(x0² + x1²)  s.t.  x0²+x1² ≤ 1, starting far away at (5,5)
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>();
    VecX x0(2); x0 << 5, 5; // far from feasible
    vars->setValues(x0);

    MatX Q = MatX::Identity(2, 2);
    VecX c = VecX::Zero(2);
    auto cost = std::make_shared<QuadraticCost>(Q, c);
    cost->linkVariables({vars});

    auto con = std::make_shared<CircleConstraint>(0.0, 1.0);
    con->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addConstraintSet(con);
    problem.addCostSet(cost);

    SQPSolver solver;
    auto result = solver.solve(problem);

    // Should converge to feasible point near origin (inside circle)
    Scalar final_norm = result.x[0]*result.x[0] + result.x[1]*result.x[1];
    EXPECT_LE(final_norm, 1.0 + 0.05);
    EXPECT_LT(result.constraintViolation, 1e-3);
}

TEST(SQPSolver, TotalQPIterationsPopulated) {
    // Ensure totalQPIterations is reported by the solver
    NLPProblem problem;
    auto vars = std::make_shared<TestVars2D>(-10.0, 10.0);
    VecX x0(2); x0 << 3, 3;
    vars->setValues(x0);

    MatX Q = MatX::Identity(2, 2);
    VecX c = VecX::Zero(2);
    auto cost = std::make_shared<QuadraticCost>(Q, c);
    cost->linkVariables({vars});

    problem.addVariableSet(vars);
    problem.addCostSet(cost);

    SQPSolver solver;
    auto result = solver.solve(problem);
    EXPECT_GT(result.totalQPIterations, 0);
    EXPECT_LT(result.cost, 1e-3);
}
