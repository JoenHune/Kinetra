// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/optimization/nlp_problem.hpp"

using namespace kinetra;
using namespace kinetra::optimization;

// ── Concrete test variable set ───────────────────────────────────────────────
class TestVars : public VariableSet {
public:
    TestVars(int n, std::string name) : VariableSet(n, std::move(name)) {}

    [[nodiscard]] VecX lowerBounds() const override {
        return VecX::Constant(size(), -10.0);
    }
    [[nodiscard]] VecX upperBounds() const override {
        return VecX::Constant(size(), 10.0);
    }
};

// ── Concrete test constraint: sum(x) >= 1 ────────────────────────────────────
class SumConstraint : public ConstraintSet {
public:
    SumConstraint(std::string var_name)
        : ConstraintSet(1, "sum_constraint"), var_name_(std::move(var_name)) {}

    [[nodiscard]] VecX evaluate() const override {
        VecX x = getVariableValues(var_name_);
        VecX g(1);
        g[0] = x.sum();
        return g;
    }

    [[nodiscard]] VecX lowerBound() const override {
        VecX lb(1); lb[0] = 1.0;
        return lb;
    }
    [[nodiscard]] VecX upperBound() const override {
        VecX ub(1); ub[0] = constants::kInfinity;
        return ub;
    }

    void fillJacobianBlock(const std::string& vn, MatX& jac) const override {
        if (vn == var_name_) {
            VecX x = getVariableValues(var_name_);
            jac = MatX::Ones(1, x.size());
        }
    }

private:
    std::string var_name_;
};

// ── Concrete test cost: 0.5 * ||x||^2 ───────────────────────────────────────
class QuadraticCost : public CostTerm {
public:
    QuadraticCost(std::string var_name)
        : CostTerm("quadratic_cost"), var_name_(std::move(var_name)) {}

    [[nodiscard]] Scalar evaluate() const override {
        VecX x = getVariableValues(var_name_);
        return 0.5 * x.squaredNorm();
    }

    void fillGradientBlock(const std::string& vn, VecX& grad) const override {
        if (vn == var_name_) {
            grad = getVariableValues(var_name_);
        }
    }

private:
    std::string var_name_;
};

// ── Tests ────────────────────────────────────────────────────────────────────

TEST(NLPProblem, AddVariableSet) {
    NLPProblem nlp;
    auto vars = std::make_shared<TestVars>(3, "x");
    nlp.addVariableSet(vars);

    EXPECT_EQ(nlp.numVariables(), 3);
    EXPECT_EQ(nlp.numConstraints(), 0);
}

TEST(NLPProblem, MultipleVariableSets) {
    NLPProblem nlp;
    auto v1 = std::make_shared<TestVars>(3, "x");
    auto v2 = std::make_shared<TestVars>(2, "y");
    nlp.addVariableSet(v1);
    nlp.addVariableSet(v2);

    EXPECT_EQ(nlp.numVariables(), 5);
    EXPECT_EQ(v1->startIndex(), 0);
    EXPECT_EQ(v2->startIndex(), 3);
}

TEST(NLPProblem, VariableValuesRoundTrip) {
    NLPProblem nlp;
    auto v1 = std::make_shared<TestVars>(3, "x");
    auto v2 = std::make_shared<TestVars>(2, "y");
    nlp.addVariableSet(v1);
    nlp.addVariableSet(v2);

    VecX vals(5);
    vals << 1, 2, 3, 4, 5;
    nlp.setVariableValues(vals);

    VecX out = nlp.variableValues();
    EXPECT_EQ(out.size(), 5);
    for (int i = 0; i < 5; ++i) {
        EXPECT_NEAR(out[i], vals[i], 1e-12);
    }
    EXPECT_NEAR(v1->values()[0], 1.0, 1e-12);
    EXPECT_NEAR(v2->values()[0], 4.0, 1e-12);
}

TEST(NLPProblem, VariableBounds) {
    NLPProblem nlp;
    auto vars = std::make_shared<TestVars>(3, "x");
    nlp.addVariableSet(vars);

    VecX lb = nlp.variableLowerBounds();
    VecX ub = nlp.variableUpperBounds();
    EXPECT_EQ(lb.size(), 3);
    EXPECT_EQ(ub.size(), 3);
    EXPECT_NEAR(lb[0], -10.0, 1e-12);
    EXPECT_NEAR(ub[0],  10.0, 1e-12);
}

TEST(NLPProblem, ConstraintEvaluation) {
    NLPProblem nlp;
    auto vars = std::make_shared<TestVars>(3, "x");
    nlp.addVariableSet(vars);

    auto cons = std::make_shared<SumConstraint>("x");
    cons->linkVariables({vars});
    nlp.addConstraintSet(cons);

    EXPECT_EQ(nlp.numConstraints(), 1);

    // Set x = [1, 2, 3], sum = 6
    VecX x(3); x << 1, 2, 3;
    nlp.setVariableValues(x);

    VecX g = nlp.constraintValues();
    EXPECT_EQ(g.size(), 1);
    EXPECT_NEAR(g[0], 6.0, 1e-12);
}

TEST(NLPProblem, ConstraintJacobian) {
    NLPProblem nlp;
    auto vars = std::make_shared<TestVars>(3, "x");
    nlp.addVariableSet(vars);

    auto cons = std::make_shared<SumConstraint>("x");
    cons->linkVariables({vars});
    nlp.addConstraintSet(cons);

    VecX x(3); x << 1, 2, 3;
    nlp.setVariableValues(x);

    MatX jac = nlp.constraintJacobian();
    EXPECT_EQ(jac.rows(), 1);
    EXPECT_EQ(jac.cols(), 3);
    // Jacobian of sum is [1, 1, 1]
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(jac(0, i), 1.0, 1e-12);
    }
}

TEST(NLPProblem, CostEvaluation) {
    NLPProblem nlp;
    auto vars = std::make_shared<TestVars>(3, "x");
    nlp.addVariableSet(vars);

    auto cost = std::make_shared<QuadraticCost>("x");
    cost->linkVariables({vars});
    nlp.addCostSet(cost);

    VecX x(3); x << 1, 2, 3;
    nlp.setVariableValues(x);

    Scalar total = nlp.totalCost();
    // 0.5 * (1 + 4 + 9) = 7.0
    EXPECT_NEAR(total, 7.0, 1e-12);
}

TEST(NLPProblem, CostGradient) {
    NLPProblem nlp;
    auto vars = std::make_shared<TestVars>(3, "x");
    nlp.addVariableSet(vars);

    auto cost = std::make_shared<QuadraticCost>("x");
    cost->linkVariables({vars});
    nlp.addCostSet(cost);

    VecX x(3); x << 1, 2, 3;
    nlp.setVariableValues(x);

    VecX grad = nlp.costGradient();
    EXPECT_EQ(grad.size(), 3);
    // gradient of 0.5*||x||^2 is x itself
    EXPECT_NEAR(grad[0], 1.0, 1e-12);
    EXPECT_NEAR(grad[1], 2.0, 1e-12);
    EXPECT_NEAR(grad[2], 3.0, 1e-12);
}

TEST(NLPProblem, ComposedProblem) {
    NLPProblem nlp;
    auto v1 = std::make_shared<TestVars>(2, "x");
    auto v2 = std::make_shared<TestVars>(2, "y");
    nlp.addVariableSet(v1);
    nlp.addVariableSet(v2);

    auto cons = std::make_shared<SumConstraint>("x");
    cons->linkVariables({v1});
    nlp.addConstraintSet(cons);

    auto cost = std::make_shared<QuadraticCost>("y");
    cost->linkVariables({v2});
    nlp.addCostSet(cost);

    EXPECT_EQ(nlp.numVariables(), 4);
    EXPECT_EQ(nlp.numConstraints(), 1);

    VecX vals(4); vals << 1, 2, 3, 4;
    nlp.setVariableValues(vals);

    EXPECT_NEAR(nlp.constraintValues()[0], 3.0, 1e-12);  // sum of x = 1+2
    EXPECT_NEAR(nlp.totalCost(), 0.5 * (9 + 16), 1e-12); // 0.5*(3^2+4^2)

    VecX grad = nlp.costGradient();
    EXPECT_NEAR(grad[0], 0.0, 1e-12);  // cost only on y
    EXPECT_NEAR(grad[1], 0.0, 1e-12);
    EXPECT_NEAR(grad[2], 3.0, 1e-12);
    EXPECT_NEAR(grad[3], 4.0, 1e-12);
}
