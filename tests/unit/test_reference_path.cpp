// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "kinetra/core/reference_path.hpp"

using namespace kinetra;

// ═════════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, DefaultConstruction) {
    ReferencePath path;
    EXPECT_EQ(path.waypointCount(), 0u);
    EXPECT_DOUBLE_EQ(path.totalLength(), 0.0);
}

TEST(ReferencePath, FromWaypoints) {
    std::vector<Waypoint2D> wps = {
        {0, 0, 0, 0},
        {1, 0, 0, Scalar(0.1)},
        {2, 0, 0, Scalar(0.2)},
    };
    ReferencePath path(wps);
    EXPECT_EQ(path.waypointCount(), 3u);
    EXPECT_NEAR(path.totalLength(), 2.0, 1e-10);
    EXPECT_EQ(path.arcLengths().size(), 3u);
    EXPECT_DOUBLE_EQ(path.arcLengths()[0], 0.0);
}

TEST(ReferencePath, FromTrajectory) {
    Trajectory2D traj;
    traj.append({0, 0, 0, 0});
    traj.append({1, 0, 0, Scalar(0.1)});
    traj.append({2, 0, 0, Scalar(0.2)});
    ReferencePath path(traj);
    EXPECT_EQ(path.waypointCount(), 3u);
    EXPECT_NEAR(path.totalLength(), 2.0, 1e-10);
}

TEST(ReferencePath, CustomOptions) {
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {1,0,0,0}};
    ReferencePath::Options opts;
    opts.lengthTheta = 1.0;
    opts.maxCurvature = 5.0;
    ReferencePath path(wps, opts);
    EXPECT_DOUBLE_EQ(path.options().lengthTheta, 1.0);
    EXPECT_DOUBLE_EQ(path.options().maxCurvature, 5.0);
}

// ═════════════════════════════════════════════════════════════════════════════
// Arc-Length Table
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, ArcLengthStraightLine) {
    // Straight horizontal line: 3 points at x=0,1,2, y=0, θ=0
    std::vector<Waypoint2D> wps = {
        {0, 0, 0, 0},
        {1, 0, 0, 0},
        {2, 0, 0, 0},
    };
    ReferencePath path(wps);
    EXPECT_NEAR(path.arcLengths()[0], 0.0, 1e-12);
    EXPECT_NEAR(path.arcLengths()[1], 1.0, 1e-12);
    EXPECT_NEAR(path.arcLengths()[2], 2.0, 1e-12);
}

TEST(ReferencePath, ArcLengthDiagonal) {
    // Diagonal from (0,0) to (3,4)
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {3,4,0,0}};
    ReferencePath path(wps);
    EXPECT_NEAR(path.totalLength(), 5.0, 1e-10);
}

TEST(ReferencePath, ArcLengthWithRotation) {
    // Pure rotation: same position, θ changes by π/2
    ReferencePath::Options opts;
    opts.lengthTheta = 1.0;  // 1 m/rad
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {0, 0, Scalar(M_PI/2), 0}};
    ReferencePath path(wps, opts);
    // ds = sqrt(0 + 0 + 1.0² * (π/2)²) = π/2
    EXPECT_NEAR(path.totalLength(), M_PI / 2, 1e-10);
}

TEST(ReferencePath, ArcLengthSE2Weighted) {
    // Movement + rotation: dx=1, dy=0, dθ=1, ℓ_θ=0.5
    ReferencePath::Options opts;
    opts.lengthTheta = 0.5;
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {1,0,1,0}};
    ReferencePath path(wps, opts);
    // ds = sqrt(1 + 0 + 0.25) = sqrt(1.25)
    EXPECT_NEAR(path.totalLength(), std::sqrt(1.25), 1e-10);
}

// ═════════════════════════════════════════════════════════════════════════════
// Evaluate
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, EvaluateAtEndpoints) {
    std::vector<Waypoint2D> wps = {
        {1, 2, 0.5, 0},
        {3, 4, 1.0, 0},
    };
    ReferencePath path(wps);

    Vec3 start = path.evaluate(0);
    EXPECT_NEAR(start[0], 1.0, 1e-10);
    EXPECT_NEAR(start[1], 2.0, 1e-10);
    EXPECT_NEAR(start[2], 0.5, 1e-10);

    Vec3 end = path.evaluate(path.totalLength());
    EXPECT_NEAR(end[0], 3.0, 1e-10);
    EXPECT_NEAR(end[1], 4.0, 1e-10);
    EXPECT_NEAR(end[2], 1.0, 1e-10);
}

TEST(ReferencePath, EvaluateMidpoint) {
    std::vector<Waypoint2D> wps = {
        {0, 0, 0, 0},
        {2, 0, 0, 0},
    };
    ReferencePath path(wps);
    Vec3 mid = path.evaluate(1.0);  // halfway
    EXPECT_NEAR(mid[0], 1.0, 1e-10);
    EXPECT_NEAR(mid[1], 0.0, 1e-10);
}

TEST(ReferencePath, EvaluateClampsBeyondRange) {
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {1,0,0,0}};
    ReferencePath path(wps);
    // Beyond total length → should clamp to last point
    Vec3 beyond = path.evaluate(10.0);
    EXPECT_NEAR(beyond[0], 1.0, 1e-10);
    // Negative → should clamp to first point
    Vec3 before = path.evaluate(-1.0);
    EXPECT_NEAR(before[0], 0.0, 1e-10);
}

TEST(ReferencePath, EvaluateMultiSegment) {
    std::vector<Waypoint2D> wps = {
        {0, 0, 0, 0},
        {1, 0, 0, 0},
        {1, 1, Scalar(M_PI/2), 0},
    };
    ReferencePath path(wps);
    // At s=0.5, should be on first segment
    Vec3 p1 = path.evaluate(0.5);
    EXPECT_NEAR(p1[0], 0.5, 1e-6);
    EXPECT_NEAR(p1[1], 0.0, 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════════
// Tangent + Normal
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, TangentHorizontal) {
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {2,0,0,0}};
    ReferencePath path(wps);
    Vec2 t = path.tangent(1.0);
    // Heading θ=0 → tangent = (cos0, sin0) = (1, 0)
    EXPECT_NEAR(t[0], 1.0, 1e-6);
    EXPECT_NEAR(t[1], 0.0, 1e-6);
}

TEST(ReferencePath, NormalPerpendicular) {
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {2,0,0,0}};
    ReferencePath path(wps);
    Vec2 n = path.normal(1.0);
    Vec2 t = path.tangent(1.0);
    // Normal should be perpendicular to tangent
    EXPECT_NEAR(t.dot(n), 0.0, 1e-10);
    // Normal should be unit length
    EXPECT_NEAR(n.norm(), 1.0, 1e-10);
}

TEST(ReferencePath, TangentDiagonal) {
    // 45° path
    std::vector<Waypoint2D> wps = {
        {0, 0, Scalar(M_PI/4), 0},
        {1, 1, Scalar(M_PI/4), 0}
    };
    ReferencePath path(wps);
    Vec2 t = path.tangent(0.5);
    EXPECT_NEAR(t[0], std::cos(M_PI/4), 1e-6);
    EXPECT_NEAR(t[1], std::sin(M_PI/4), 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════════
// findSegment
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, FindSegment) {
    std::vector<Waypoint2D> wps = {
        {0, 0, 0, 0},
        {1, 0, 0, 0},
        {2, 0, 0, 0},
        {3, 0, 0, 0},
    };
    ReferencePath path(wps);
    EXPECT_EQ(path.findSegment(0.0), 0u);
    EXPECT_EQ(path.findSegment(0.5), 0u);
    EXPECT_EQ(path.findSegment(1.0), 1u);
    EXPECT_EQ(path.findSegment(1.5), 1u);
    EXPECT_EQ(path.findSegment(2.5), 2u);
}

// ═════════════════════════════════════════════════════════════════════════════
// Curvature + thetaDot
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, ThetaDotStraight) {
    // No heading change → dθ/ds ≈ 0
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {2,0,0,0}};
    ReferencePath path(wps);
    EXPECT_NEAR(path.thetaDot(1.0), 0.0, 1e-10);
}

TEST(ReferencePath, CurvatureStraight) {
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {2,0,0,0}};
    ReferencePath path(wps);
    EXPECT_NEAR(path.curvature(1.0), 0.0, 1e-10);
}

TEST(ReferencePath, CurvatureTurn) {
    // Quarter turn: θ goes from 0 to π/2 over length 1
    std::vector<Waypoint2D> wps = {{0,0,0,0}, {1, 0, Scalar(M_PI/2), 0}};
    ReferencePath path(wps);
    Scalar kappa = path.curvature(0.5);
    // κ ≈ Δθ / Δ_pos_s ≈ (π/2) / 1 ≈ 1.57
    EXPECT_GT(kappa, 1.0);
    EXPECT_LT(kappa, path.options().maxCurvature);
}

// ═════════════════════════════════════════════════════════════════════════════
// Edge cases
// ═════════════════════════════════════════════════════════════════════════════

TEST(ReferencePath, EmptyPath) {
    ReferencePath path;
    EXPECT_DOUBLE_EQ(path.totalLength(), 0.0);
    EXPECT_EQ(path.waypointCount(), 0u);
}

TEST(ReferencePath, SingleWaypoint) {
    std::vector<Waypoint2D> wps = {{5, 3, 1.0, 0}};
    ReferencePath path(wps);
    EXPECT_EQ(path.waypointCount(), 1u);
    EXPECT_DOUBLE_EQ(path.totalLength(), 0.0);
}
