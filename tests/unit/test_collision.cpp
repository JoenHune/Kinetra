// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/collision/occupancy_grid.hpp"

using namespace kinetra;
using namespace kinetra::collision;

class OccupancyGridTest : public ::testing::Test {
protected:
    OccupancyGrid2D grid{-10, -10, 10, 10, Scalar(0.1)};
};

TEST_F(OccupancyGridTest, EmptyGridAllFree) {
    EXPECT_TRUE(grid.isFree(Vec2(0, 0)));
    EXPECT_TRUE(grid.isFree(Vec2(5, 5)));
    EXPECT_TRUE(grid.isFree(Vec2(-5, -5)));
}

TEST_F(OccupancyGridTest, CircleObstacle) {
    grid.addCircleObstacle(Vec2(0, 0), 1.0);
    EXPECT_FALSE(grid.isFree(Vec2(0, 0)));
    EXPECT_FALSE(grid.isFree(Vec2(0.5, 0)));
    EXPECT_TRUE(grid.isFree(Vec2(2, 0)));
}

TEST_F(OccupancyGridTest, RectObstacle) {
    grid.addRectObstacle(Vec2(-1, -1), Vec2(1, 1));
    EXPECT_FALSE(grid.isFree(Vec2(0, 0)));
    EXPECT_FALSE(grid.isFree(Vec2(0.5, 0.5)));
    EXPECT_TRUE(grid.isFree(Vec2(2, 2)));
}

TEST_F(OccupancyGridTest, SegmentFreeInEmptyGrid) {
    EXPECT_TRUE(grid.isSegmentFree(Vec2(-5, 0), Vec2(5, 0)));
}

TEST_F(OccupancyGridTest, SegmentBlockedByObstacle) {
    grid.addCircleObstacle(Vec2(0, 0), 2.0);
    EXPECT_FALSE(grid.isSegmentFree(Vec2(-5, 0), Vec2(5, 0)));
}

TEST_F(OccupancyGridTest, SegmentAroundObstacle) {
    grid.addCircleObstacle(Vec2(0, 0), 1.0);
    // Segment that goes around the obstacle
    EXPECT_TRUE(grid.isSegmentFree(Vec2(-5, 3), Vec2(5, 3)));
}

TEST_F(OccupancyGridTest, SignedDistance) {
    grid.addCircleObstacle(Vec2(0, 0), 1.0);
    // Inside obstacle: negative distance
    EXPECT_LT(grid.signedDistance(Vec2(0, 0)), 0);
    // Outside obstacle: positive distance
    EXPECT_GT(grid.signedDistance(Vec2(5, 0)), 0);
}

TEST_F(OccupancyGridTest, ClearGrid) {
    grid.addCircleObstacle(Vec2(0, 0), 1.0);
    EXPECT_FALSE(grid.isFree(Vec2(0, 0)));
    grid.clear();
    EXPECT_TRUE(grid.isFree(Vec2(0, 0)));
}

TEST_F(OccupancyGridTest, OutOfBoundsNotFree) {
    EXPECT_FALSE(grid.isFree(Vec2(100, 100)));
}
