// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/collision/occupancy_grid.hpp"

#include <cmath>
#include <queue>

namespace kinetra::collision {

void OccupancyGrid2D::setOccupied(int gx, int gy) {
    if (inBounds(gx, gy)) {
        grid_[static_cast<std::size_t>(idx(gx, gy))] = true;
        distance_field_dirty_ = true;
    }
}

void OccupancyGrid2D::addCircleObstacle(const Vec2& center, Scalar radius) {
    int gx_min = worldToGridX(center.x() - radius);
    int gx_max = worldToGridX(center.x() + radius);
    int gy_min = worldToGridY(center.y() - radius);
    int gy_max = worldToGridY(center.y() + radius);

    for (int gy = gy_min; gy <= gy_max; ++gy) {
        for (int gx = gx_min; gx <= gx_max; ++gx) {
            if (!inBounds(gx, gy)) continue;
            Scalar wx = gridToWorldX(gx) - center.x();
            Scalar wy = gridToWorldY(gy) - center.y();
            if (wx * wx + wy * wy <= radius * radius) {
                grid_[static_cast<std::size_t>(idx(gx, gy))] = true;
            }
        }
    }
    distance_field_dirty_ = true;
}

void OccupancyGrid2D::addRectObstacle(const Vec2& min_corner, const Vec2& max_corner) {
    int gx_min = worldToGridX(min_corner.x());
    int gx_max = worldToGridX(max_corner.x());
    int gy_min = worldToGridY(min_corner.y());
    int gy_max = worldToGridY(max_corner.y());

    for (int gy = gy_min; gy <= gy_max; ++gy) {
        for (int gx = gx_min; gx <= gx_max; ++gx) {
            if (inBounds(gx, gy)) {
                grid_[static_cast<std::size_t>(idx(gx, gy))] = true;
            }
        }
    }
    distance_field_dirty_ = true;
}

void OccupancyGrid2D::clear() {
    std::fill(grid_.begin(), grid_.end(), false);
    distance_field_dirty_ = true;
}

bool OccupancyGrid2D::isFree(const Vec2& point) const {
    int gx = worldToGridX(point.x());
    int gy = worldToGridY(point.y());
    if (!inBounds(gx, gy)) return false;
    return !grid_[static_cast<std::size_t>(idx(gx, gy))];
}

Scalar OccupancyGrid2D::signedDistance(const Vec2& point) const {
    if (distance_field_dirty_) {
        const_cast<OccupancyGrid2D*>(this)->recomputeDistanceField();
    }
    int gx = worldToGridX(point.x());
    int gy = worldToGridY(point.y());
    if (!inBounds(gx, gy)) return -constants::kInfinity;
    return distance_field_[static_cast<std::size_t>(idx(gx, gy))];
}

bool OccupancyGrid2D::isSegmentFree(const Vec2& from, const Vec2& to) const {
    // Bresenham-like line traversal
    int x0 = worldToGridX(from.x()), y0 = worldToGridY(from.y());
    int x1 = worldToGridX(to.x()),   y1 = worldToGridY(to.y());

    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (!inBounds(x0, y0) || grid_[static_cast<std::size_t>(idx(x0, y0))]) {
            return false;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
    return true;
}

void OccupancyGrid2D::recomputeDistanceField() {
    // BFS-based approximate Euclidean distance transform
    distance_field_.resize(static_cast<std::size_t>(width_ * height_));
    constexpr Scalar kLargeVal = static_cast<Scalar>(1e6);

    // Initialize: obstacles = 0, free = large
    for (int i = 0; i < width_ * height_; ++i) {
        distance_field_[static_cast<std::size_t>(i)] =
            grid_[static_cast<std::size_t>(i)] ? 0 : kLargeVal;
    }

    // Two-pass distance transform (Rosenfeld & Pfaltz)
    // Forward pass
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            auto& d = distance_field_[static_cast<std::size_t>(idx(x, y))];
            if (x > 0) d = std::min(d, distance_field_[static_cast<std::size_t>(idx(x-1, y))] + resolution_);
            if (y > 0) d = std::min(d, distance_field_[static_cast<std::size_t>(idx(x, y-1))] + resolution_);
        }
    }
    // Backward pass
    for (int y = height_ - 1; y >= 0; --y) {
        for (int x = width_ - 1; x >= 0; --x) {
            auto& d = distance_field_[static_cast<std::size_t>(idx(x, y))];
            if (x + 1 < width_) d = std::min(d, distance_field_[static_cast<std::size_t>(idx(x+1, y))] + resolution_);
            if (y + 1 < height_) d = std::min(d, distance_field_[static_cast<std::size_t>(idx(x, y+1))] + resolution_);
        }
    }

    // Compute interior distance field (distance from occupied cells to nearest free)
    std::vector<Scalar> interior_field(static_cast<std::size_t>(width_ * height_));
    for (int i = 0; i < width_ * height_; ++i) {
        interior_field[static_cast<std::size_t>(i)] =
            grid_[static_cast<std::size_t>(i)] ? kLargeVal : 0;
    }
    // Forward pass (interior)
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            auto& d = interior_field[static_cast<std::size_t>(idx(x, y))];
            if (x > 0) d = std::min(d, interior_field[static_cast<std::size_t>(idx(x-1, y))] + resolution_);
            if (y > 0) d = std::min(d, interior_field[static_cast<std::size_t>(idx(x, y-1))] + resolution_);
        }
    }
    // Backward pass (interior)
    for (int y = height_ - 1; y >= 0; --y) {
        for (int x = width_ - 1; x >= 0; --x) {
            auto& d = interior_field[static_cast<std::size_t>(idx(x, y))];
            if (x + 1 < width_) d = std::min(d, interior_field[static_cast<std::size_t>(idx(x+1, y))] + resolution_);
            if (y + 1 < height_) d = std::min(d, interior_field[static_cast<std::size_t>(idx(x, y+1))] + resolution_);
        }
    }

    // Combine: free cells get exterior distance, occupied cells get negative interior distance
    for (int i = 0; i < width_ * height_; ++i) {
        if (grid_[static_cast<std::size_t>(i)]) {
            distance_field_[static_cast<std::size_t>(i)] =
                -interior_field[static_cast<std::size_t>(i)];
        }
    }

    distance_field_dirty_ = false;
}

}  // namespace kinetra::collision
