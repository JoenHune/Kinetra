// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Real-vector state space R^N for generic N-dimensional planning.

#pragma once

#include <random>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

/// Generic real-vector state space R^N.
/// Supports both compile-time (N > 0) and runtime (N = Eigen::Dynamic) dimensions.
template <int N = Eigen::Dynamic>
class RealVectorSpace {
public:
    using StateType = Eigen::Matrix<Scalar, N, 1>;
    static constexpr std::size_t kDimension = (N > 0) ? static_cast<std::size_t>(N) : 0;

    RealVectorSpace(StateType lower, StateType upper)
        : lower_(std::move(lower)), upper_(std::move(upper))
        , gen_(std::random_device{}()) {
        assert(lower_.size() == upper_.size());
    }

    // ── StateSpace concept ───────────────────────────────────────────────────

    [[nodiscard]] Scalar distance(const StateType& a, const StateType& b) const noexcept {
        return (b - a).norm();
    }

    [[nodiscard]] StateType interpolate(const StateType& from,
                                         const StateType& to,
                                         Scalar t) const noexcept {
        return from + t * (to - from);
    }

    [[nodiscard]] std::size_t dimension() const noexcept {
        return static_cast<std::size_t>(lower_.size());
    }

    [[nodiscard]] bool isValid(const StateType& s) const noexcept {
        return (s.array() >= lower_.array()).all() &&
               (s.array() <= upper_.array()).all();
    }

    // ── SamplableSpace extension ─────────────────────────────────────────────

    [[nodiscard]] StateType sampleUniform() const {
        auto& gen = const_cast<std::mt19937&>(gen_);
        std::uniform_real_distribution<Scalar> dist(0, 1);
        StateType sample = StateType::Zero(lower_.size());
        for (Index i = 0; i < lower_.size(); ++i) {
            sample[i] = lower_[i] + dist(gen) * (upper_[i] - lower_[i]);
        }
        return sample;
    }

    // ── Accessors ────────────────────────────────────────────────────────────
    [[nodiscard]] const StateType& lowerBound() const noexcept { return lower_; }
    [[nodiscard]] const StateType& upperBound() const noexcept { return upper_; }

private:
    StateType lower_, upper_;
    mutable std::mt19937 gen_;
};

// Common aliases
using RealVector2Space = RealVectorSpace<2>;
using RealVector3Space = RealVectorSpace<3>;
using RealVectorXSpace = RealVectorSpace<Eigen::Dynamic>;

}  // namespace kinetra
