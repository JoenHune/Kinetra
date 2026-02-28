// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/collision/polygon_obstacles.hpp"

using namespace kinetra;
using namespace kinetra::collision;

class PolygonObstacleTest : public ::testing::Test {
protected:
    OccupancyGrid2D grid_{-10, -10, 10, 10, Scalar(0.1)};
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

// ═════════════════════════════════════════════════════════════════════════════
// Public API: pointInConvexPolygon
// ═════════════════════════════════════════════════════════════════════════════

TEST(PointInPolygon, InsideSquare) {
    std::vector<Vec2> square = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    EXPECT_TRUE(pointInConvexPolygon(Vec2(1, 1), square));
    EXPECT_TRUE(pointInConvexPolygon(Vec2(0.1, 0.1), square));
}

TEST(PointInPolygon, OutsideSquare) {
    std::vector<Vec2> square = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    EXPECT_FALSE(pointInConvexPolygon(Vec2(3, 1), square));
    EXPECT_FALSE(pointInConvexPolygon(Vec2(-1, 1), square));
    EXPECT_FALSE(pointInConvexPolygon(Vec2(1, 3), square));
}

TEST(PointInPolygon, InsideTriangle) {
    std::vector<Vec2> tri = {Vec2(0, 0), Vec2(4, 0), Vec2(2, 3)};
    EXPECT_TRUE(pointInConvexPolygon(Vec2(2, 1), tri));
}

TEST(PointInPolygon, OutsideTriangle) {
    std::vector<Vec2> tri = {Vec2(0, 0), Vec2(4, 0), Vec2(2, 3)};
    EXPECT_FALSE(pointInConvexPolygon(Vec2(5, 5), tri));
}

// ═════════════════════════════════════════════════════════════════════════════
// Public API: signedDistanceToPolygon
// ═════════════════════════════════════════════════════════════════════════════

TEST(SignedDistance, InsideSquare) {
    std::vector<Vec2> square = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    Scalar sd = signedDistanceToPolygon(Vec2(1, 1), square);
    EXPECT_LT(sd, 0);  // Inside → negative
    EXPECT_NEAR(sd, -1.0, 1e-6);  // Distance to nearest edge = 1
}

TEST(SignedDistance, OutsideSquare) {
    std::vector<Vec2> square = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    Scalar sd = signedDistanceToPolygon(Vec2(3, 1), square);
    EXPECT_GT(sd, 0);  // Outside → positive
    EXPECT_NEAR(sd, 1.0, 1e-6);
}

TEST(SignedDistance, OnEdge) {
    std::vector<Vec2> square = {
        Vec2(0, 0), Vec2(2, 0), Vec2(2, 2), Vec2(0, 2)
    };
    Scalar sd = signedDistanceToPolygon(Vec2(1, 0), square);
    EXPECT_NEAR(std::abs(sd), 0.0, 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════════
// Public API: gjkIntersect
// ═════════════════════════════════════════════════════════════════════════════

TEST(GJKIntersect, DisjointSquares) {
    std::vector<Vec2> a = {Vec2(0,0), Vec2(1,0), Vec2(1,1), Vec2(0,1)};
    std::vector<Vec2> b = {Vec2(5,5), Vec2(6,5), Vec2(6,6), Vec2(5,6)};
    EXPECT_FALSE(gjkIntersect(a, b));
}

TEST(GJKIntersect, OverlappingSquares) {
    std::vector<Vec2> a = {Vec2(0,0), Vec2(2,0), Vec2(2,2), Vec2(0,2)};
    std::vector<Vec2> b = {Vec2(1,1), Vec2(3,1), Vec2(3,3), Vec2(1,3)};
    EXPECT_TRUE(gjkIntersect(a, b));
}

TEST(GJKIntersect, Contained) {
    std::vector<Vec2> outer = {Vec2(-5,-5), Vec2(5,-5), Vec2(5,5), Vec2(-5,5)};
    std::vector<Vec2> inner = {Vec2(-1,-1), Vec2(1,-1), Vec2(1,1), Vec2(-1,1)};
    EXPECT_TRUE(gjkIntersect(outer, inner));
}

TEST(GJKIntersect, TriangleVsSquare) {
    std::vector<Vec2> tri = {Vec2(0,0), Vec2(2,0), Vec2(1,2)};
    std::vector<Vec2> sq  = {Vec2(3,0), Vec2(4,0), Vec2(4,1), Vec2(3,1)};
    EXPECT_FALSE(gjkIntersect(tri, sq));
}

