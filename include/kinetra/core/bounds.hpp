// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Bounds representation for state/control variables.

#pragma once

#include "kinetra/core/types.hpp"

namespace kinetra {

/// Axis-aligned bounding box in N dimensions.
template <int N>
struct Bounds {
    using VecN = Eigen::Matrix<Scalar, N, 1>;

    VecN lower;
    VecN upper;

    /// Check if a point is within bounds (inclusive).
    [[nodiscard]] bool contains(const VecN& x) const noexcept {
        return (x.array() >= lower.array()).all() &&
               (x.array() <= upper.array()).all();
    }

    /// Clamp a point to within bounds.
    [[nodiscard]] VecN clamp(const VecN& x) const noexcept {
        return x.cwiseMax(lower).cwiseMin(upper);
    }

    /// Volume of the bounding box.
    [[nodiscard]] Scalar volume() const noexcept {
        return (upper - lower).prod();
    }
};

using Bounds2D = Bounds<2>;
using Bounds3D = Bounds<3>;

}  // namespace kinetra
