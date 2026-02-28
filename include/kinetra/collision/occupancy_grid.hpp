// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// 2D occupancy grid collision checker with signed distance field.

#pragma once

#include <cstdint>
#include <vector>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra::collision {

/// 2D occupancy grid — stores binary occupied/free cells.
/// Provides collision checking and signed distance queries.
class OccupancyGrid2D {
public:
    OccupancyGrid2D(Scalar x_min, Scalar y_min, Scalar x_max, Scalar y_max,
                     Scalar resolution)
        : x_min_(x_min), y_min_(y_min)
        , resolution_(resolution)
        , inv_resolution_(static_cast<Scalar>(1.0) / resolution)
        , width_(static_cast<int>((x_max - x_min) * inv_resolution_) + 1)
        , height_(static_cast<int>((y_max - y_min) * inv_resolution_) + 1)
        , grid_(static_cast<std::size_t>(width_ * height_), false)
        , distance_field_dirty_(true) {}

    // ── Grid manipulation ────────────────────────────────────────────────────

    /// Mark a cell as occupied
    void setOccupied(int gx, int gy);

    /// Mark a circular region as occupied
    void addCircleObstacle(const Vec2& center, Scalar radius);

    /// Mark a rectangular region as occupied
    void addRectObstacle(const Vec2& min_corner, const Vec2& max_corner);

    /// Mark a convex polygon region as occupied (rasterized via point-in-polygon)
    void addPolygonObstacle(const std::vector<Vec2>& vertices);

    /// Clear all obstacles
    void clear();

    // ── Collision checking (CollisionChecker concept) ────────────────────────

    /// Check if a world-frame point is free (not occupied)
    [[nodiscard]] bool isFree(const Vec2& point) const;

    /// Signed distance to nearest obstacle (positive = free)
    [[nodiscard]] Scalar signedDistance(const Vec2& point) const;

    /// Analytical SDF gradient via central differences on the distance grid.
    /// Returns (∂sdf/∂x, ∂sdf/∂y).  Cost: two grid look-ups per axis.
    [[nodiscard]] Vec2 sdfGradient(const Vec2& point) const;

    /// Check if segment is collision-free (Bresenham line + buffer)
    [[nodiscard]] bool isSegmentFree(const Vec2& from, const Vec2& to) const;

    // ── Distance field ───────────────────────────────────────────────────────

    /// Recompute the signed distance field (Euclidean distance transform).
    /// Called lazily when signedDistance() is queried after modifications.
    void recomputeDistanceField();

    // ── Polygon collision (GJK) ──────────────────────────────────────────────

    /// GJK intersection test between two convex polygons.
    /// Returns true if the polygons do NOT overlap (collision-free).
    [[nodiscard]] static bool isPolygonCollisionFree(
        const std::vector<Vec2>& robot_vertices,
        const std::vector<Vec2>& obstacle_vertices);

    // ── Coordinate conversion ────────────────────────────────────────────────

    [[nodiscard]] int worldToGridX(Scalar x) const noexcept {
        return static_cast<int>((x - x_min_) * inv_resolution_);
    }
    [[nodiscard]] int worldToGridY(Scalar y) const noexcept {
        return static_cast<int>((y - y_min_) * inv_resolution_);
    }
    [[nodiscard]] Scalar gridToWorldX(int gx) const noexcept {
        return x_min_ + static_cast<Scalar>(gx) * resolution_;
    }
    [[nodiscard]] Scalar gridToWorldY(int gy) const noexcept {
        return y_min_ + static_cast<Scalar>(gy) * resolution_;
    }

    // ── Accessors ────────────────────────────────────────────────────────────
    [[nodiscard]] int width() const noexcept { return width_; }
    [[nodiscard]] int height() const noexcept { return height_; }
    [[nodiscard]] Scalar resolution() const noexcept { return resolution_; }

private:
    Scalar x_min_, y_min_;
    Scalar resolution_, inv_resolution_;
    int width_, height_;
    std::vector<bool> grid_;              // true = occupied
    std::vector<Scalar> distance_field_;  // signed distance (world units)
    mutable bool distance_field_dirty_;

    [[nodiscard]] bool inBounds(int gx, int gy) const noexcept {
        return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
    }
    [[nodiscard]] int idx(int gx, int gy) const noexcept {
        return gy * width_ + gx;
    }
};

// Concept verification
static_assert(CollisionChecker<OccupancyGrid2D>);
static_assert(ContinuousCollisionChecker<OccupancyGrid2D>);

}  // namespace kinetra::collision
