// SPDX-License-Identifier: BSD-3-Clause
// GJK-based convex polygon collision detection for 2D.
// Used to rasterize PolygonObstacle into OccupancyGrid2D.

#include "kinetra/collision/polygon_obstacles.hpp"
#include "kinetra/collision/occupancy_grid.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace kinetra::collision {

namespace {

// ── GJK support function: farthest vertex in direction d ─────────────────────
Vec2 gjkSupport(const std::vector<Vec2>& vertices, const Vec2& d) {
    Scalar best = -constants::kInfinity;
    Vec2 result = vertices[0];
    for (const auto& v : vertices) {
        Scalar dot = v.dot(d);
        if (dot > best) {
            best = dot;
            result = v;
        }
    }
    return result;
}

// Minkowski difference support: support(A, d) - support(B, -d)
Vec2 supportMinkowski(const std::vector<Vec2>& A,
                       const std::vector<Vec2>& B,
                       const Vec2& d) {
    return gjkSupport(A, d) - gjkSupport(B, -d);
}

// Triple product: (A × B) × C  (2D version returns a Vec2)
Vec2 tripleProduct(const Vec2& a, const Vec2& b, const Vec2& c) {
    // In 2D: (a × b) is scalar = a.x*b.y - a.y*b.x
    // (a × b) × c = scalar * Vec2(-c.y, c.x)
    Scalar cross = a.x() * b.y() - a.y() * b.x();
    return Vec2(-cross * c.y(), cross * c.x());
}

}  // anonymous namespace

// ── GJK algorithm: returns true if convex shapes A and B overlap ─────────────
bool gjkIntersect(const std::vector<Vec2>& A, const std::vector<Vec2>& B) {
    // Initial direction: from center of B to center of A
    Vec2 centerA = Vec2::Zero();
    for (const auto& v : A) centerA += v;
    centerA /= static_cast<Scalar>(A.size());

    Vec2 centerB = Vec2::Zero();
    for (const auto& v : B) centerB += v;
    centerB /= static_cast<Scalar>(B.size());

    Vec2 d = centerA - centerB;
    if (d.squaredNorm() < constants::kEpsilon) d = Vec2(1, 0);

    // First simplex point
    Vec2 s = supportMinkowski(A, B, d);
    std::array<Vec2, 3> simplex;
    int simplexSize = 1;
    simplex[0] = s;

    d = -s;  // Direction towards origin

    for (int iter = 0; iter < 64; ++iter) {
        Vec2 a_pt = supportMinkowski(A, B, d);

        // If the new point doesn't pass the origin, no intersection
        if (a_pt.dot(d) < 0) return false;

        simplex[static_cast<std::size_t>(simplexSize)] = a_pt;
        simplexSize++;

        if (simplexSize == 2) {
            // Line case: simplex = {B_pt, A_pt}
            Vec2 ab = simplex[0] - simplex[1];  // B - A
            Vec2 ao = -simplex[1];               // Origin - A

            // Direction perpendicular to AB towards origin
            d = tripleProduct(ab, ao, ab);
            if (d.squaredNorm() < constants::kEpsilon) {
                // Origin is on the line segment — intersecting
                return true;
            }
        } else {
            // Triangle case: simplex = {C, B, A} where A is newest
            Vec2 a_s = simplex[2];
            Vec2 b_s = simplex[1];
            Vec2 c_s = simplex[0];
            Vec2 ao = -a_s;
            Vec2 ab = b_s - a_s;
            Vec2 ac = c_s - a_s;

            Vec2 abPerp = tripleProduct(ac, ab, ab);
            Vec2 acPerp = tripleProduct(ab, ac, ac);

            if (abPerp.dot(ao) > 0) {
                // Region AB: remove C
                simplex[0] = simplex[1];
                simplex[1] = simplex[2];
                simplexSize = 2;
                d = abPerp;
            } else if (acPerp.dot(ao) > 0) {
                // Region AC: remove B
                simplex[1] = simplex[2];
                simplexSize = 2;
                d = acPerp;
            } else {
                // Origin is inside the triangle
                return true;
            }
        }
    }
    return false;
}

// ── Point-in-polygon test (ray casting) ──────────────────────────────────────
bool pointInConvexPolygon(const Vec2& point, const std::vector<Vec2>& vertices) {
    int n = static_cast<int>(vertices.size());
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const auto& vi = vertices[static_cast<std::size_t>(i)];
        const auto& vj = vertices[static_cast<std::size_t>(j)];
        if (((vi.y() > point.y()) != (vj.y() > point.y())) &&
            (point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) /
                         (vj.y() - vi.y()) + vi.x())) {
            inside = !inside;
        }
    }
    return inside;
}

// ── Signed distance from point to convex polygon ────────────────────────────
Scalar signedDistanceToPolygon(const Vec2& point, const std::vector<Vec2>& vertices) {
    int n = static_cast<int>(vertices.size());
    if (n < 3) return constants::kInfinity;

    // Minimum distance to any edge
    Scalar min_dist = constants::kInfinity;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        const Vec2& a = vertices[static_cast<std::size_t>(i)];
        const Vec2& b = vertices[static_cast<std::size_t>(j)];
        Vec2 ab = b - a;
        Vec2 ap = point - a;
        Scalar t = clamp(ap.dot(ab) / ab.squaredNorm(), Scalar(0), Scalar(1));
        Vec2 closest = a + t * ab;
        Scalar dist = (point - closest).norm();
        min_dist = std::min(min_dist, dist);
    }

    // Negate if inside
    if (pointInConvexPolygon(point, vertices)) {
        return -min_dist;
    }
    return min_dist;
}

// ── Public API: add polygon obstacle to the occupancy grid ───────────────────
void OccupancyGrid2D::addPolygonObstacle(const std::vector<Vec2>& vertices) {
    if (vertices.size() < 3) return;

    // Compute bounding box
    Scalar min_x = constants::kInfinity, min_y = constants::kInfinity;
    Scalar max_x = -constants::kInfinity, max_y = -constants::kInfinity;
    for (const auto& v : vertices) {
        min_x = std::min(min_x, v.x());
        min_y = std::min(min_y, v.y());
        max_x = std::max(max_x, v.x());
        max_y = std::max(max_y, v.y());
    }

    int gx_min = worldToGridX(min_x);
    int gx_max = worldToGridX(max_x);
    int gy_min = worldToGridY(min_y);
    int gy_max = worldToGridY(max_y);

    // Rasterize: for each cell in bounding box, check if center is inside polygon
    for (int gy = gy_min; gy <= gy_max; ++gy) {
        for (int gx = gx_min; gx <= gx_max; ++gx) {
            if (!inBounds(gx, gy)) continue;
            Vec2 cell_center(gridToWorldX(gx), gridToWorldY(gy));
            if (pointInConvexPolygon(cell_center, vertices)) {
                grid_[static_cast<std::size_t>(idx(gx, gy))] = true;
            }
        }
    }
    distance_field_dirty_ = true;
}

bool OccupancyGrid2D::isPolygonCollisionFree(const std::vector<Vec2>& robot_vertices,
                                               const std::vector<Vec2>& obstacle_vertices) {
    return !gjkIntersect(robot_vertices, obstacle_vertices);
}

}  // namespace kinetra::collision
