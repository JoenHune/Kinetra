// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Reference Path — arc-length parameterized SE(2) path representation.
//
// Given an ordered sequence of Waypoint2D, this class builds a continuous
// SE(2)-weighted arc-length parameterization P(s) = (x(s), y(s), θ(s)),
// supporting O(log M) queries via precomputed cumulative arc-length table.
//
// Used by MPCC (Model Predictive Contouring Control) for path tracking.

#pragma once

#include <vector>

#include "kinetra/core/trajectory.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

/// SE(2)-weighted arc-length parameterized reference path.
///
/// The arc-length parameter s accumulates:
///   ds = sqrt(dx² + dy² + ℓ_θ² · dθ²)
///
/// where ℓ_θ (m/rad) converts angular displacement to equivalent length.
/// This ensures both pure translation and pure rotation advance s.
class ReferencePath {
public:
    struct Options {
        Scalar lengthTheta = static_cast<Scalar>(0.5);   // ℓ_θ (m/rad)
        Scalar maxCurvature = static_cast<Scalar>(10.0);  // Curvature clamp
    };

    ReferencePath() = default;
    explicit ReferencePath(const std::vector<Waypoint2D>& waypoints);
    ReferencePath(const std::vector<Waypoint2D>& waypoints,
                  Options opts);
    explicit ReferencePath(const Trajectory2D& trajectory);
    ReferencePath(const Trajectory2D& trajectory,
                  Options opts);

    /// Evaluate reference pose at arc-length s.
    /// Returns (x, y, θ) via linear/angular interpolation.
    [[nodiscard]] Vec3 evaluate(Scalar s) const;

    /// Tangent vector (dx/ds, dy/ds) at arc-length s (unit or zero for rotation).
    [[nodiscard]] Vec2 tangent(Scalar s) const;

    /// Normal vector (perpendicular to tangent, rotated +90°).
    [[nodiscard]] Vec2 normal(Scalar s) const;

    /// Heading derivative dθ/ds at arc-length s.
    [[nodiscard]] Scalar thetaDot(Scalar s) const;

    /// Path curvature κ(s) = dθ_xy/ds (clamped to maxCurvature).
    [[nodiscard]] Scalar curvature(Scalar s) const;

    /// Total arc-length of the reference path.
    [[nodiscard]] Scalar totalLength() const noexcept {
        return arc_lengths_.empty() ? Scalar(0)
                                    : arc_lengths_.back();
    }

    /// Number of waypoints.
    [[nodiscard]] std::size_t waypointCount() const noexcept {
        return waypoints_.size();
    }

    /// Access the arc-length table.
    [[nodiscard]] const std::vector<Scalar>& arcLengths() const noexcept {
        return arc_lengths_;
    }

    /// Options used for this path.
    [[nodiscard]] const Options& options() const noexcept { return opts_; }

    /// Find segment index for given s (binary search).
    [[nodiscard]] std::size_t findSegment(Scalar s) const;

private:
    std::vector<Waypoint2D> waypoints_;
    std::vector<Scalar> arc_lengths_;  // Cumulative, arc_lengths_[0] = 0
    Options opts_;

    void buildArcLengthTable();
};

}  // namespace kinetra
