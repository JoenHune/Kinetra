// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Dubins path state space — minimum-length paths for forward-only vehicles.

#pragma once

#include <array>
#include <cmath>
#include <optional>

#include "kinetra/core/types.hpp"
#include "kinetra/spaces/se2.hpp"

namespace kinetra {

/// The 6 Dubins path types
enum class DubinsPathType : uint8_t {
    LSL, LSR, RSL, RSR, RLR, LRL
};

/// A Dubins path segment: type + 3 segment lengths
struct DubinsPath {
    DubinsPathType type;
    std::array<Scalar, 3> lengths;  // segment lengths

    [[nodiscard]] Scalar totalLength() const noexcept {
        return lengths[0] + lengths[1] + lengths[2];
    }
};

/// Dubins space — plans optimal-length forward-only curved paths.
/// The turning radius determines the minimum curvature.
class DubinsSpace {
public:
    using StateType = SE2State;
    static constexpr std::size_t kDimension = 3;

    explicit DubinsSpace(Scalar turning_radius,
                         Scalar x_min, Scalar x_max,
                         Scalar y_min, Scalar y_max)
        : rho_(turning_radius)
        , x_min_(x_min), x_max_(x_max)
        , y_min_(y_min), y_max_(y_max) {}

    // ── StateSpace concept ───────────────────────────────────────────────────

    /// Distance = shortest Dubins path length
    [[nodiscard]] Scalar distance(const SE2State& from, const SE2State& to) const;

    /// Interpolation along Dubins path at parameter t ∈ [0, 1]
    [[nodiscard]] SE2State interpolate(const SE2State& from,
                                        const SE2State& to,
                                        Scalar t) const;

    [[nodiscard]] static constexpr std::size_t dimension() noexcept { return kDimension; }

    [[nodiscard]] bool isValid(const SE2State& s) const noexcept {
        return s.x >= x_min_ && s.x <= x_max_ &&
               s.y >= y_min_ && s.y <= y_max_;
    }

    // ── Dubins-specific API ──────────────────────────────────────────────────

    /// Compute the shortest Dubins path between two SE2 states.
    [[nodiscard]] std::optional<DubinsPath> shortestPath(
        const SE2State& from, const SE2State& to) const;

    /// Compute all 6 Dubins path types and return all valid ones.
    [[nodiscard]] std::vector<DubinsPath> allPaths(
        const SE2State& from, const SE2State& to) const;

    /// Sample points along a Dubins path at given resolution.
    [[nodiscard]] std::vector<SE2State> samplePath(
        const SE2State& from, const DubinsPath& path,
        Scalar step_size) const;

    [[nodiscard]] Scalar turningRadius() const noexcept { return rho_; }

private:
    Scalar rho_;  // minimum turning radius
    Scalar x_min_, x_max_;
    Scalar y_min_, y_max_;

    // Internal: compute path for a specific type (normalized coordinates)
    [[nodiscard]] std::optional<DubinsPath> computePath(
        DubinsPathType type, Scalar d, Scalar alpha, Scalar beta) const;
};

static_assert(StateSpace<DubinsSpace>);

}  // namespace kinetra
