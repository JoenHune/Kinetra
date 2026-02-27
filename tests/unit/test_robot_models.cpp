// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"

using namespace kinetra;

// ─── Differential Drive ──────────────────────────────────────────────────────
TEST(DiffDriveSimple, ConceptSatisfied) {
    static_assert(RobotModel<DiffDriveSimple>);
    static_assert(LinearizableModel<DiffDriveSimple>);
    SUCCEED();
}

TEST(DiffDriveSimple, StraightLineDynamics) {
    DiffDriveSimple model;
    DiffDriveSimple::StateType x;
    x << 0, 0, 0;
    DiffDriveSimple::ControlType u;
    u << 1.0, 0;  // v=1, omega=0

    auto x_next = model.dynamics(x, u, 1.0);
    EXPECT_NEAR(x_next[0], 1.0, 1e-6);  // moved 1m forward
    EXPECT_NEAR(x_next[1], 0.0, 1e-6);
    EXPECT_NEAR(x_next[2], 0.0, 1e-6);
}

TEST(DiffDriveSimple, TurnInPlace) {
    DiffDriveSimple model;
    DiffDriveSimple::StateType x;
    x << 0, 0, 0;
    DiffDriveSimple::ControlType u;
    u << 0, 1.0;  // v=0, omega=1

    auto x_next = model.dynamics(x, u, 1.0);
    EXPECT_NEAR(x_next[0], 0.0, 1e-6);
    EXPECT_NEAR(x_next[1], 0.0, 1e-6);
    EXPECT_NEAR(x_next[2], 1.0, 1e-6);  // turned 1 radian
}

TEST(DiffDriveSimple, JacobianFiniteDiff) {
    DiffDriveSimple model;
    DiffDriveSimple::StateType x;
    x << 1, 2, 0.5;
    DiffDriveSimple::ControlType u;
    u << 0.5, 0.3;
    Scalar dt = 0.1;

    auto A = model.jacobianState(x, u, dt);
    // Verify Jacobian via finite differences
    Scalar eps = 1e-5;
    for (int i = 0; i < 3; ++i) {
        DiffDriveSimple::StateType x_plus = x, x_minus = x;
        x_plus[i] += eps;
        x_minus[i] -= eps;
        auto f_plus = model.dynamics(x_plus, u, dt);
        auto f_minus = model.dynamics(x_minus, u, dt);
        for (int j = 0; j < 3; ++j) {
            Scalar numerical = (f_plus[j] - f_minus[j]) / (2 * eps);
            EXPECT_NEAR(A(j, i), numerical, 1e-4)
                << "A(" << j << "," << i << ")";
        }
    }
}

// ─── Ackermann ───────────────────────────────────────────────────────────────
TEST(AckermannSimple, ConceptSatisfied) {
    static_assert(RobotModel<AckermannSimple>);
    static_assert(LinearizableModel<AckermannSimple>);
    SUCCEED();
}

TEST(AckermannSimple, StraightLine) {
    AckermannSimple model;
    AckermannSimple::StateType x;
    x << 0, 0, 0;
    AckermannSimple::ControlType u;
    u << 1.0, 0;  // v=1, steering=0

    auto x_next = model.dynamics(x, u, 1.0);
    EXPECT_NEAR(x_next[0], 1.0, 1e-6);
    EXPECT_NEAR(x_next[1], 0.0, 1e-6);
    EXPECT_NEAR(x_next[2], 0.0, 1e-6);
}

TEST(AckermannSimple, MinTurningRadius) {
    AckermannSimple model;
    model.wheelbase = 2.5;
    model.max_steering_angle = 0.5;
    Scalar r = model.minTurningRadius();
    EXPECT_GT(r, 0);
    EXPECT_NEAR(r, 2.5 / std::tan(0.5), 1e-6);
}

// ─── Omnidirectional ─────────────────────────────────────────────────────────
TEST(OmniSimple, ConceptSatisfied) {
    static_assert(RobotModel<OmniSimple>);
    static_assert(LinearizableModel<OmniSimple>);
    SUCCEED();
}

TEST(OmniSimple, StrafeMotion) {
    OmniSimple model;
    OmniSimple::StateType x;
    x << 0, 0, 0;  // facing +x
    OmniSimple::ControlType u;
    u << 0, 1.0, 0;  // strafe left (body frame vy=1)

    auto x_next = model.dynamics(x, u, 1.0);
    EXPECT_NEAR(x_next[0], 0.0, 1e-6);   // no forward motion
    EXPECT_NEAR(x_next[1], 1.0, 1e-6);   // moved in +y (world frame)
    EXPECT_NEAR(x_next[2], 0.0, 1e-6);
}

// ─── Differential Drive Jerk ─────────────────────────────────────────────────
TEST(DiffDriveJerk, ConceptSatisfied) {
    static_assert(RobotModel<DiffDriveJerk>);
    static_assert(LinearizableModel<DiffDriveJerk>);
    SUCCEED();
}

TEST(DiffDriveJerk, DynamicsPropagation) {
    DiffDriveJerk model;
    model.max_v = 2.0;  // raise limit so clamp doesn't interfere
    DiffDriveJerk::StateType x;
    // (x, y, theta, v, omega, a, alpha)
    x << 0, 0, 0, 1.0, 0, 0.5, 0;
    DiffDriveJerk::ControlType u;
    u << 0, 0;  // zero jerk

    auto x_next = model.dynamics(x, u, 0.1);
    EXPECT_NEAR(x_next[0], 0.1, 1e-6);   // moved with v=1
    EXPECT_NEAR(x_next[1], 0.0, 1e-6);
    EXPECT_NEAR(x_next[3], 1.05, 1e-6);  // v += a*dt = 1.0 + 0.5*0.1
    EXPECT_NEAR(x_next[5], 0.5, 1e-6);   // a unchanged (jerk=0)
}

TEST(DiffDriveJerk, JacobianFiniteDiff) {
    DiffDriveJerk model;
    DiffDriveJerk::StateType x;
    x << 1, 2, 0.3, 0.5, 0.1, 0.2, 0.1;
    DiffDriveJerk::ControlType u;
    u << 0.5, 0.3;
    Scalar dt = 0.05;

    auto A = model.jacobianState(x, u, dt);
    Scalar eps = 1e-5;
    for (int i = 0; i < 7; ++i) {
        DiffDriveJerk::StateType xp = x, xm = x;
        xp[i] += eps;
        xm[i] -= eps;
        auto fp = model.dynamics(xp, u, dt);
        auto fm = model.dynamics(xm, u, dt);
        for (int j = 0; j < 7; ++j) {
            Scalar numerical = (fp[j] - fm[j]) / (2 * eps);
            EXPECT_NEAR(A(j, i), numerical, 1e-3)
                << "A(" << j << "," << i << ")";
        }
    }
}

// ─── Differential Drive Snap ─────────────────────────────────────────────────
TEST(DiffDriveSnap, ConceptSatisfied) {
    static_assert(RobotModel<DiffDriveSnap>);
    static_assert(LinearizableModel<DiffDriveSnap>);
    SUCCEED();
}

TEST(DiffDriveSnap, ChainedIntegration) {
    DiffDriveSnap model;
    DiffDriveSnap::StateType x = DiffDriveSnap::StateType::Zero();
    x[7] = 1.0;  // jerk = 1 m/s³
    DiffDriveSnap::ControlType u = DiffDriveSnap::ControlType::Zero();

    // After 0.1s with jerk=1: a += jerk*dt = 0.1, v += a_old*dt = 0
    auto x1 = model.dynamics(x, u, 0.1);
    EXPECT_NEAR(x1[5], 0.1, 1e-6);  // a = 0 + 1*0.1
    EXPECT_NEAR(x1[3], 0.0, 1e-6);  // v = 0 + 0*0.1 = 0 (a was 0)
}

// ─── Ackermann Jerk ──────────────────────────────────────────────────────────
TEST(AckermannJerk, ConceptSatisfied) {
    static_assert(RobotModel<AckermannJerk>);
    static_assert(LinearizableModel<AckermannJerk>);
    SUCCEED();
}

TEST(AckermannJerk, StraightLine) {
    AckermannJerk model;
    AckermannJerk::StateType x;
    // (x, y, theta, v, phi, a, dphi)
    x << 0, 0, 0, 1.0, 0, 0, 0;
    AckermannJerk::ControlType u;
    u << 0, 0;

    auto x_next = model.dynamics(x, u, 1.0);
    EXPECT_NEAR(x_next[0], 1.0, 1e-6);
    EXPECT_NEAR(x_next[1], 0.0, 1e-6);
}

TEST(AckermannJerk, JacobianFiniteDiff) {
    AckermannJerk model;
    AckermannJerk::StateType x;
    x << 1, 2, 0.2, 1.0, 0.1, 0.5, 0.1;
    AckermannJerk::ControlType u;
    u << 0.3, 0.1;
    Scalar dt = 0.05;

    auto A = model.jacobianState(x, u, dt);
    Scalar eps = 1e-5;
    for (int i = 0; i < 7; ++i) {
        AckermannJerk::StateType xp = x, xm = x;
        xp[i] += eps;
        xm[i] -= eps;
        auto fp = model.dynamics(xp, u, dt);
        auto fm = model.dynamics(xm, u, dt);
        for (int j = 0; j < 7; ++j) {
            Scalar numerical = (fp[j] - fm[j]) / (2 * eps);
            EXPECT_NEAR(A(j, i), numerical, 1e-3)
                << "A(" << j << "," << i << ")";
        }
    }
}
