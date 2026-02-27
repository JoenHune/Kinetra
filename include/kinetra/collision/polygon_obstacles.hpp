// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// GJK-based convex polygon collision detection for 2D.
// Provides point-in-polygon testing, signed distance to polygon edges,
// and GJK intersection test between convex polygon pairs.
//
// Implementation lives in collision/polygon_obstacles.cpp.

#pragma once

#include <vector>

#include "kinetra/core/types.hpp"

namespace kinetra::collision {

/// Point-in-convex-polygon test via ray casting.
/// Works for convex and concave (simple) polygons.
[[nodiscard]] bool pointInConvexPolygon(const Vec2& point,
                                        const std::vector<Vec2>& vertices);

/// Signed distance from a point to the boundary of a convex polygon.
/// Negative = inside, positive = outside.
[[nodiscard]] Scalar signedDistanceToPolygon(const Vec2& point,
                                             const std::vector<Vec2>& vertices);

/// GJK intersection test between two convex polygons.
/// Returns true if the polygons overlap.
[[nodiscard]] bool gjkIntersect(const std::vector<Vec2>& A,
                                const std::vector<Vec2>& B);

}  // namespace kinetra::collision
