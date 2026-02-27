// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/spaces/dubins.hpp"

using namespace kinetra;

class DubinsTest : public ::testing::Test {
protected:
    DubinsSpace space{1.0, -20, 20, -20, 20};  // turning radius = 1
};

TEST_F(DubinsTest, SamePointDistance) {
    SE2State s{0, 0, 0};
    EXPECT_NEAR(space.distance(s, s), 0.0, 1e-6);
}

TEST_F(DubinsTest, StraightAheadDistance) {
    SE2State from{0, 0, 0};
    SE2State to{5, 0, 0};
    Scalar d = space.distance(from, to);
    // Straight line: distance should be ~5
    EXPECT_NEAR(d, 5.0, 0.5);
}

TEST_F(DubinsTest, ShortestPathExists) {
    SE2State from{0, 0, 0};
    SE2State to{5, 5, constants::kHalfPi};
    auto path = space.shortestPath(from, to);
    ASSERT_TRUE(path.has_value());
    EXPECT_GT(path->totalLength(), 0);
}

TEST_F(DubinsTest, AllPathsGenerated) {
    SE2State from{0, 0, 0};
    SE2State to{5, 5, constants::kHalfPi};
    auto paths = space.allPaths(from, to);
    EXPECT_GE(paths.size(), 1u);
    EXPECT_LE(paths.size(), 6u);
}

TEST_F(DubinsTest, SamplePathGeneratesPoints) {
    SE2State from{0, 0, 0};
    SE2State to{5, 0, 0};
    auto path = space.shortestPath(from, to);
    ASSERT_TRUE(path.has_value());

    auto samples = space.samplePath(from, *path, 0.5);
    EXPECT_GE(samples.size(), 2u);

    // First sample should be near start
    EXPECT_NEAR(samples.front().x, 0, 0.5);
    EXPECT_NEAR(samples.front().y, 0, 0.5);
}

TEST_F(DubinsTest, DistanceTriangleInequality) {
    SE2State a{0, 0, 0};
    SE2State b{3, 3, 1};
    SE2State c{6, 0, 0};

    Scalar d_ac = space.distance(a, c);
    Scalar d_ab = space.distance(a, b);
    Scalar d_bc = space.distance(b, c);

    EXPECT_LE(d_ac, d_ab + d_bc + 1e-6);
}
