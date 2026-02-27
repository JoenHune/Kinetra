// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/collision/occupancy_grid.hpp"

using namespace kinetra;
using namespace kinetra::collision;

class PolygonObstacleTest : public ::testing::Test {
protected:
    OccupancyGrid2D grid_{-10, -10, 10, 10, 0.1};
};

TEST_F(PolygonObstacleTest, AddPolygonMarksOccupied) {
    std::vector<Vec2> square = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    grid_.addPolygonObstacle(square);

    // Center of the polygon should be occupied
    EXPECT_FALSE(grid_.isFree(Vec2(1.0, 1.0)));
    // Well outside should be free
    EXPECT_TRUE(grid_.isFree(Vec2(5.0, 5.0)));
}

TEST_F(PolygonObstacleTest, AddTriangleObstacle) {
    std::vector<Vec2> triangle = {
        Vec2(-1, 0), Vec2(1, 0), Vec2(0, 2)
    };
    grid_.addPolygonObstacle(triangle);

    // Inside the triangle
    EXPECT_FALSE(grid_.isFree(Vec2(0, 0.5)));
    // Outside
    EXPECT_TRUE(grid_.isFree(Vec2(3, 3)));
}

TEST_F(PolygonObstacleTest, GJKDisjointPolygons) {
    std::vector<Vec2> poly_a = {
        Vec2(0, 0), Vec2(1, 0), Vec2(1, 1), Vec2(0, 1)
    };
    std::vector<Vec2> poly_b = {
        Vec2(3, 0), Vec2(4, 0), Vec2(4, 1), Vec2(3, 1)
    };
    EXPECT_TRUE(OccupancyGrid2D::isPolygonCollisionFree(poly_a, poly_b));
}

TEST_F(PolygonObstacleTest, GJKOverlappingPolygons) {
    std::vector<Vec2> poly_a = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    std::vector<Vec2> poly_b = {
        Vec2(1, 1), Vec2(3, 1), Vec2(3, 3), Vec2(1, 3)
    };
    EXPECT_FALSE(OccupancyGrid2D::isPolygonCollisionFree(poly_a, poly_b));
}

TEST_F(PolygonObstacleTest, GJKIdenticalPolygons) {
    std::vector<Vec2> poly = {
        Vec2(0, 0), Vec2(1, 0), Vec2(1, 1), Vec2(0, 1)
    };
    EXPECT_FALSE(OccupancyGrid2D::isPolygonCollisionFree(poly, poly));
}

TEST_F(PolygonObstacleTest, GJKContainedPolygon) {
    std::vector<Vec2> outer = {
        Vec2(-2, -2), Vec2(2, -2), Vec2(2, 2), Vec2(-2, 2)
    };
    std::vector<Vec2> inner = {
        Vec2(-0.5, -0.5), Vec2(0.5, -0.5), Vec2(0.5, 0.5), Vec2(-0.5, 0.5)
    };
    EXPECT_FALSE(OccupancyGrid2D::isPolygonCollisionFree(outer, inner));
}
