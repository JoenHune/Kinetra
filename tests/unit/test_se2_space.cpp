// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/spaces/se2.hpp"

using namespace kinetra;

TEST(SE2Space, ConceptSatisfied) {
    // Compile-time check already done via static_assert in header
    SUCCEED();
}

TEST(SE2Space, Distance) {
    SE2Space space(-10, 10, -10, 10);
    SE2State a{0, 0, 0};
    SE2State b{3, 4, 0};
    EXPECT_NEAR(space.distance(a, b), 5.0, 1e-6);
}

TEST(SE2Space, DistanceWithAngle) {
    SE2Space space(-10, 10, -10, 10);
    space.setAngularWeight(1.0);
    SE2State a{0, 0, 0};
    SE2State b{0, 0, constants::kPi};
    // Distance should be angular_weight * pi
    EXPECT_NEAR(space.distance(a, b), constants::kPi, 1e-6);
}

TEST(SE2Space, Interpolate) {
    SE2Space space(-10, 10, -10, 10);
    SE2State a{0, 0, 0};
    SE2State b{10, 0, constants::kHalfPi};

    auto mid = space.interpolate(a, b, 0.5);
    EXPECT_NEAR(mid.x, 5.0, 1e-6);
    EXPECT_NEAR(mid.y, 0.0, 1e-6);
    EXPECT_NEAR(mid.theta, constants::kHalfPi / 2, 1e-6);
}

TEST(SE2Space, InterpolateBoundary) {
    SE2Space space(-10, 10, -10, 10);
    SE2State a{1, 2, 0.5};
    SE2State b{5, 6, 1.5};

    auto start = space.interpolate(a, b, 0);
    EXPECT_NEAR(start.x, a.x, 1e-10);
    EXPECT_NEAR(start.y, a.y, 1e-10);

    auto end = space.interpolate(a, b, 1);
    EXPECT_NEAR(end.x, b.x, 1e-10);
    EXPECT_NEAR(end.y, b.y, 1e-10);
}

TEST(SE2Space, IsValid) {
    SE2Space space(-5, 5, -5, 5);
    EXPECT_TRUE(space.isValid({0, 0, 0}));
    EXPECT_TRUE(space.isValid({-5, -5, 0}));
    EXPECT_FALSE(space.isValid({6, 0, 0}));
    EXPECT_FALSE(space.isValid({0, -6, 0}));
}

TEST(SE2Space, SampleUniform) {
    SE2Space space(-10, 10, -10, 10);
    for (int i = 0; i < 100; ++i) {
        auto s = space.sampleUniform();
        EXPECT_TRUE(space.isValid(s));
        EXPECT_GE(s.theta, -constants::kPi);
        EXPECT_LE(s.theta, constants::kPi);
    }
}

TEST(SE2Space, Dimension) {
    SE2Space space(-10, 10, -10, 10);
    EXPECT_EQ(space.dimension(), 3u);
    EXPECT_EQ(SE2Space::kDimension, 3u);
}
