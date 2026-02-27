// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// SE(2) state space: position (x, y) + orientation (theta).
// The fundamental space for 2D mobile robot planning.

#pragma once

#include <cmath>
#include <random>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

/// SE(2) state: (x, y, theta)
struct SE2State {
    Scalar x{0};
    Scalar y{0};
    Scalar theta{0};

    [[nodiscard]] Vec2 position() const noexcept { return {x, y}; }
    [[nodiscard]] Vec3 toVec() const noexcept { return {x, y, theta}; }

    static SE2State fromVec(const Vec3& v) noexcept {
        return {v[0], v[1], v[2]};
    }

    friend bool operator==(const SE2State&, const SE2State&) = default;
};

/// SE(2) state space with Euclidean + angular metric.
class SE2Space {
public:
    using StateType = SE2State;
    static constexpr std::size_t kDimension = 3;

    SE2Space(Scalar x_min, Scalar x_max,
             Scalar y_min, Scalar y_max)
        : x_min_(x_min), x_max_(x_max)
        , y_min_(y_min), y_max_(y_max)
        , gen_(std::random_device{}()) {}

    // ── StateSpace concept requirements ──────────────────────────────────────

    [[nodiscard]] Scalar distance(const SE2State& a, const SE2State& b) const noexcept {
        Scalar dx = b.x - a.x;
        Scalar dy = b.y - a.y;
        Scalar dtheta = std::abs(angularDistance(a.theta, b.theta));
        // Weighted metric: position + orientation
        return std::sqrt(dx * dx + dy * dy) + angular_weight_ * dtheta;
    }

    [[nodiscard]] SE2State interpolate(const SE2State& from,
                                        const SE2State& to,
                                        Scalar t) const noexcept {
        return {
            lerp(from.x, to.x, t),
            lerp(from.y, to.y, t),
            normalizeAngle(from.theta + t * angularDistance(from.theta, to.theta))
        };
    }

    [[nodiscard]] static constexpr std::size_t dimension() noexcept {
        return kDimension;
    }

    [[nodiscard]] bool isValid(const SE2State& s) const noexcept {
        return s.x >= x_min_ && s.x <= x_max_ &&
               s.y >= y_min_ && s.y <= y_max_;
    }

    // ── SamplableSpace extension ─────────────────────────────────────────────

    [[nodiscard]] SE2State sampleUniform() const {
        auto& gen = const_cast<std::mt19937&>(gen_);  // NOTE: thread-local in production
        std::uniform_real_distribution<Scalar> x_dist(x_min_, x_max_);
        std::uniform_real_distribution<Scalar> y_dist(y_min_, y_max_);
        std::uniform_real_distribution<Scalar> theta_dist(-constants::kPi, constants::kPi);
        return {x_dist(gen), y_dist(gen), theta_dist(gen)};
    }

    // ── Configuration ────────────────────────────────────────────────────────

    void setAngularWeight(Scalar w) noexcept { angular_weight_ = w; }
    [[nodiscard]] Scalar angularWeight() const noexcept { return angular_weight_; }

    [[nodiscard]] Scalar xMin() const noexcept { return x_min_; }
    [[nodiscard]] Scalar xMax() const noexcept { return x_max_; }
    [[nodiscard]] Scalar yMin() const noexcept { return y_min_; }
    [[nodiscard]] Scalar yMax() const noexcept { return y_max_; }

private:
    Scalar x_min_, x_max_;
    Scalar y_min_, y_max_;
    Scalar angular_weight_{static_cast<Scalar>(0.5)};
    mutable std::mt19937 gen_;
};

// Verify concepts at compile time
static_assert(StateSpace<SE2Space>);
static_assert(SamplableSpace<SE2Space>);

}  // namespace kinetra
