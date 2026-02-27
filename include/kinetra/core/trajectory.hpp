// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Trajectory representation — a timed sequence of waypoints.

#pragma once

#include <algorithm>
#include <cassert>
#include <span>
#include <vector>

#include "kinetra/core/types.hpp"

namespace kinetra {

// ─── Waypoint2D: position + heading + time ───────────────────────────────────
struct Waypoint2D {
    Scalar x{0};
    Scalar y{0};
    Scalar theta{0};  // heading angle (radians)
    Scalar t{0};      // time stamp (seconds)

    [[nodiscard]] Vec2 position() const noexcept { return {x, y}; }
    [[nodiscard]] Vec3 pose() const noexcept { return {x, y, theta}; }

    friend bool operator==(const Waypoint2D&, const Waypoint2D&) = default;
};

// ─── Trajectory2D: ordered sequence of Waypoint2D ────────────────────────────
class Trajectory2D {
public:
    Trajectory2D() = default;

    explicit Trajectory2D(std::vector<Waypoint2D> waypoints)
        : waypoints_(std::move(waypoints)) {}

    // ── Accessors ────────────────────────────────────────────────────────────
    [[nodiscard]] std::size_t size() const noexcept { return waypoints_.size(); }
    [[nodiscard]] bool empty() const noexcept { return waypoints_.empty(); }

    [[nodiscard]] const Waypoint2D& operator[](std::size_t i) const noexcept {
        assert(i < waypoints_.size());
        return waypoints_[i];
    }
    [[nodiscard]] Waypoint2D& operator[](std::size_t i) noexcept {
        assert(i < waypoints_.size());
        return waypoints_[i];
    }

    [[nodiscard]] const Waypoint2D& front() const noexcept { return waypoints_.front(); }
    [[nodiscard]] const Waypoint2D& back() const noexcept { return waypoints_.back(); }

    [[nodiscard]] std::span<const Waypoint2D> waypoints() const noexcept {
        return waypoints_;
    }
    [[nodiscard]] std::span<Waypoint2D> waypoints() noexcept {
        return waypoints_;
    }

    // ── Mutators ─────────────────────────────────────────────────────────────
    void append(Waypoint2D wp) { waypoints_.push_back(wp); }
    void reserve(std::size_t n) { waypoints_.reserve(n); }
    void clear() { waypoints_.clear(); }

    // ── Metrics ──────────────────────────────────────────────────────────────

    /// Total path length (Euclidean positions only)
    [[nodiscard]] Scalar pathLength() const noexcept;

    /// Total duration (last time - first time)
    [[nodiscard]] Scalar duration() const noexcept;

    /// Maximum curvature along the trajectory
    [[nodiscard]] Scalar maxCurvature() const noexcept;

    /// Smoothness metric: sum of squared angular changes
    [[nodiscard]] Scalar smoothness() const noexcept;

    // ── Interpolation ────────────────────────────────────────────────────────

    /// Interpolate waypoint at time t (linear between knots)
    [[nodiscard]] Waypoint2D interpolateAtTime(Scalar t) const;

    /// Resample trajectory to N uniformly-spaced waypoints
    [[nodiscard]] Trajectory2D resample(std::size_t n) const;

    // ── Iterators ────────────────────────────────────────────────────────────
    auto begin() noexcept { return waypoints_.begin(); }
    auto end() noexcept { return waypoints_.end(); }
    auto begin() const noexcept { return waypoints_.begin(); }
    auto end() const noexcept { return waypoints_.end(); }

private:
    std::vector<Waypoint2D> waypoints_;
};

}  // namespace kinetra
