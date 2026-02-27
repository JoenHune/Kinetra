// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/core/trajectory.hpp"

using namespace kinetra;

class TrajectoryTest : public ::testing::Test {
protected:
    Trajectory2D makeStraightLine(int n, Scalar length) {
        Trajectory2D traj;
        for (int i = 0; i < n; ++i) {
            Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(n - 1);
            traj.append({t * length, 0, 0, t});
        }
        return traj;
    }
};

TEST_F(TrajectoryTest, EmptyTrajectory) {
    Trajectory2D traj;
    EXPECT_TRUE(traj.empty());
    EXPECT_EQ(traj.size(), 0u);
    EXPECT_NEAR(traj.pathLength(), 0, 1e-10);
    EXPECT_NEAR(traj.duration(), 0, 1e-10);
}

TEST_F(TrajectoryTest, StraightLineLength) {
    auto traj = makeStraightLine(10, 5.0);
    EXPECT_EQ(traj.size(), 10u);
    EXPECT_NEAR(traj.pathLength(), 5.0, 1e-6);
}

TEST_F(TrajectoryTest, Duration) {
    auto traj = makeStraightLine(10, 5.0);
    EXPECT_NEAR(traj.duration(), 1.0, 1e-10);
}

TEST_F(TrajectoryTest, InterpolateAtTime) {
    auto traj = makeStraightLine(10, 10.0);
    auto wp = traj.interpolateAtTime(0.5);
    EXPECT_NEAR(wp.x, 5.0, 0.1);
    EXPECT_NEAR(wp.y, 0.0, 1e-10);
}

TEST_F(TrajectoryTest, InterpolateAtBoundaries) {
    auto traj = makeStraightLine(10, 10.0);
    auto wp_start = traj.interpolateAtTime(0.0);
    auto wp_end = traj.interpolateAtTime(1.0);
    EXPECT_NEAR(wp_start.x, 0.0, 1e-10);
    EXPECT_NEAR(wp_end.x, 10.0, 1e-10);
}

TEST_F(TrajectoryTest, Resample) {
    auto traj = makeStraightLine(5, 10.0);
    auto resampled = traj.resample(20);
    EXPECT_EQ(resampled.size(), 20u);
    EXPECT_NEAR(resampled.pathLength(), 10.0, 0.1);
}

TEST_F(TrajectoryTest, MaxCurvatureStraightLine) {
    auto traj = makeStraightLine(10, 10.0);
    EXPECT_NEAR(traj.maxCurvature(), 0.0, 1e-6);
}
