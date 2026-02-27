// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/solvers/qp_admm.hpp"

using namespace kinetra;
using namespace kinetra::solvers;

TEST(QPSolverADMM, SimpleUnconstrained) {
    // min 0.5 * x' [2 0; 0 2] x + [-2; -4]' x
    // => x* = [1; 2]
    MatX Q(2, 2);
    Q << 2, 0, 0, 2;
    VecX c(2);
    c << -2, -4;

    // No constraints: A = I, lb = -inf, ub = inf
    MatX A = MatX::Identity(2, 2);
    VecX lb = VecX::Constant(2, -1e6);
    VecX ub = VecX::Constant(2, 1e6);

    QPSolverADMM solver;
    ASSERT_TRUE(solver.setup(Q, c, A, lb, ub));
    auto result = solver.solve();
    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 1.0, 1e-3);
    EXPECT_NEAR(result.x[1], 2.0, 1e-3);
}

TEST(QPSolverADMM, BoxConstrained) {
    // min 0.5 * x' [2 0; 0 2] x + [-10; -10]' x
    // s.t. 0 <= x <= 3
    // Unconstrained: x* = [5; 5], constrained: x* = [3; 3]
    MatX Q(2, 2);
    Q << 2, 0, 0, 2;
    VecX c(2);
    c << -10, -10;

    MatX A = MatX::Identity(2, 2);
    VecX lb = VecX::Zero(2);
    VecX ub = VecX::Constant(2, 3.0);

    QPSolverADMM solver;
    ASSERT_TRUE(solver.setup(Q, c, A, lb, ub));
    auto result = solver.solve();
    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0], 3.0, 1e-2);
    EXPECT_NEAR(result.x[1], 3.0, 1e-2);
}

TEST(QPSolverADMM, LinearConstraint) {
    // min 0.5 * x' I x
    // s.t. x1 + x2 >= 2
    MatX Q = MatX::Identity(2, 2);
    VecX c = VecX::Zero(2);

    MatX A(1, 2);
    A << 1, 1;
    VecX lb(1);
    lb << 2;
    VecX ub(1);
    ub << 1e6;

    QPSolverADMM solver;
    ASSERT_TRUE(solver.setup(Q, c, A, lb, ub));
    auto result = solver.solve();
    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.x[0] + result.x[1], 2.0, 1e-2);
    // Symmetric: both should be ~1
    EXPECT_NEAR(result.x[0], 1.0, 1e-2);
    EXPECT_NEAR(result.x[1], 1.0, 1e-2);
}

TEST(QPSolverADMM, InvalidSetup) {
    MatX Q(2, 2);
    Q << 1, 0, 0, 1;
    VecX c(3);  // Wrong size!
    c << 1, 2, 3;
    MatX A = MatX::Identity(2, 2);
    VecX lb = VecX::Zero(2);
    VecX ub = VecX::Ones(2);

    QPSolverADMM solver;
    EXPECT_FALSE(solver.setup(Q, c, A, lb, ub));
}

TEST(QPSolverADMM, WarmStart) {
    // Solve twice — warm-started should converge faster
    MatX Q = MatX::Identity(4, 4);
    VecX c(4);
    c << -1, -2, -3, -4;
    MatX A = MatX::Identity(4, 4);
    VecX lb = VecX::Constant(4, -5.0);
    VecX ub = VecX::Constant(4, 5.0);

    QPSolverADMM solver;
    solver.setup(Q, c, A, lb, ub);
    auto r1 = solver.solve();
    EXPECT_TRUE(r1.converged);

    // Warm-start with previous solution
    QPSolverADMM solver2;
    solver2.setup(Q, c, A, lb, ub);
    solver2.warmStart(r1.x, r1.y);
    auto r2 = solver2.solve();
    EXPECT_TRUE(r2.converged);
    EXPECT_LE(r2.iterations, r1.iterations);
    EXPECT_NEAR(r2.x[0], r1.x[0], 1e-3);
}

TEST(QPSolverADMM, AutoRhoScaling) {
    // Test auto-rho initialization with differently-scaled problems
    MatX Q = 100.0 * MatX::Identity(3, 3);
    VecX c = VecX::Zero(3);
    MatX A = MatX::Identity(3, 3);
    VecX lb = VecX::Constant(3, -1.0);
    VecX ub = VecX::Constant(3, 1.0);

    QPSolverADMM solver;
    solver.setup(Q, c, A, lb, ub);
    auto r = solver.solve();
    EXPECT_TRUE(r.converged);
    EXPECT_NEAR(r.x.squaredNorm(), 0.0, 1e-2);
}
