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
