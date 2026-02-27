// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/core/types.hpp"

using namespace kinetra;

TEST(Types, NormalizeAngle) {
    EXPECT_NEAR(normalizeAngle(0), 0, 1e-10);
    EXPECT_NEAR(normalizeAngle(constants::kPi), -constants::kPi, 1e-10);
    EXPECT_NEAR(normalizeAngle(-constants::kPi), -constants::kPi, 1e-10);
    EXPECT_NEAR(normalizeAngle(constants::kTwoPi), 0, 1e-10);
    EXPECT_NEAR(normalizeAngle(3 * constants::kPi), -constants::kPi, 1e-10);
}

TEST(Types, AngularDistance) {
    EXPECT_NEAR(angularDistance(0, constants::kPi), -constants::kPi, 1e-10);
    EXPECT_NEAR(angularDistance(0, constants::kHalfPi), constants::kHalfPi, 1e-10);
    // normalizeAngle(π) maps to -π by the [-π, π) convention
    EXPECT_NEAR(angularDistance(-constants::kHalfPi, constants::kHalfPi),
                -constants::kPi, 1e-10);
}

TEST(Types, Lerp) {
    EXPECT_NEAR(lerp(0, 10, 0.5), 5, 1e-10);
    EXPECT_NEAR(lerp(0, 10, 0), 0, 1e-10);
    EXPECT_NEAR(lerp(0, 10, 1), 10, 1e-10);
}

TEST(Types, Clamp) {
    EXPECT_EQ(clamp(5.0, 0.0, 10.0), 5.0);
    EXPECT_EQ(clamp(-1.0, 0.0, 10.0), 0.0);
    EXPECT_EQ(clamp(15.0, 0.0, 10.0), 10.0);
}

TEST(Types, Sq) {
    EXPECT_NEAR(sq(3.0), 9.0, 1e-10);
    EXPECT_NEAR(sq(-4.0), 16.0, 1e-10);
}

TEST(Types, Vec2Construction) {
    Vec2 v(1.0, 2.0);
    EXPECT_EQ(v.x(), 1.0);
    EXPECT_EQ(v.y(), 2.0);
}
