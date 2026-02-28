// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Core type definitions for the Kinetra trajectory planning library.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace kinetra {

// ─── Scalar Type ─────────────────────────────────────────────────────────────
// Use float on resource-constrained platforms (ARMv7 NEON: 2x throughput)
#ifdef KINETRA_SCALAR_FLOAT
using Scalar = float;
#else
using Scalar = double;
#endif

// ─── Common Eigen Types ─────────────────────────────────────────────────────
using Vec2   = Eigen::Matrix<Scalar, 2, 1>;
using Vec3   = Eigen::Matrix<Scalar, 3, 1>;
using Vec4   = Eigen::Matrix<Scalar, 4, 1>;
using VecX   = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using Mat2   = Eigen::Matrix<Scalar, 2, 2>;
using Mat3   = Eigen::Matrix<Scalar, 3, 3>;
using Mat4   = Eigen::Matrix<Scalar, 4, 4>;
using MatX   = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using SpMatX = Eigen::SparseMatrix<Scalar>;  // CSC sparse matrix
using Triplet = Eigen::Triplet<Scalar>;
using Quat   = Eigen::Quaternion<Scalar>;
using AngleAxis = Eigen::AngleAxis<Scalar>;

// ─── Index Type ──────────────────────────────────────────────────────────────
using Index = Eigen::Index;

// ─── Time ────────────────────────────────────────────────────────────────────
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = std::chrono::duration<double>;  // seconds as double

// ─── Constants ───────────────────────────────────────────────────────────────
namespace constants {
    inline constexpr Scalar kPi        = static_cast<Scalar>(3.14159265358979323846);
    inline constexpr Scalar kTwoPi     = static_cast<Scalar>(2.0) * kPi;
    inline constexpr Scalar kHalfPi    = kPi / static_cast<Scalar>(2.0);
    inline constexpr Scalar kEpsilon   = std::numeric_limits<Scalar>::epsilon();
    inline constexpr Scalar kInfinity  = std::numeric_limits<Scalar>::infinity();
    inline constexpr Scalar kDegToRad  = kPi / static_cast<Scalar>(180.0);
    inline constexpr Scalar kRadToDeg  = static_cast<Scalar>(180.0) / kPi;
}  // namespace constants

// ─── Utility Functions ───────────────────────────────────────────────────────

/// Normalize angle to [-pi, pi)
constexpr Scalar normalizeAngle(Scalar theta) noexcept {
    while (theta >= constants::kPi) theta -= constants::kTwoPi;
    while (theta < -constants::kPi) theta += constants::kTwoPi;
    return theta;
}

/// Shortest angular distance from a to b (result in [-pi, pi))
constexpr Scalar angularDistance(Scalar a, Scalar b) noexcept {
    return normalizeAngle(b - a);
}

/// Linear interpolation
constexpr Scalar lerp(Scalar a, Scalar b, Scalar t) noexcept {
    return a + t * (b - a);
}

/// Clamp value to [lo, hi]
constexpr Scalar clamp(Scalar v, Scalar lo, Scalar hi) noexcept {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/// Squared value
constexpr Scalar sq(Scalar v) noexcept { return v * v; }

}  // namespace kinetra
