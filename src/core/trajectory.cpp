// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/core/trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace kinetra {

Scalar Trajectory2D::pathLength() const noexcept {
    if (waypoints_.size() < 2) return 0;
    Scalar total = 0;
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        Scalar dx = waypoints_[i].x - waypoints_[i - 1].x;
        Scalar dy = waypoints_[i].y - waypoints_[i - 1].y;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

Scalar Trajectory2D::duration() const noexcept {
    if (waypoints_.size() < 2) return 0;
    return waypoints_.back().t - waypoints_.front().t;
}

Scalar Trajectory2D::maxCurvature() const noexcept {
    if (waypoints_.size() < 3) return 0;
    Scalar max_k = 0;
    for (std::size_t i = 1; i + 1 < waypoints_.size(); ++i) {
        // Discrete curvature via Menger curvature
        Vec2 p0 = waypoints_[i - 1].position();
        Vec2 p1 = waypoints_[i].position();
        Vec2 p2 = waypoints_[i + 1].position();

        Vec2 d1 = p1 - p0;
        Vec2 d2 = p2 - p1;
        Scalar cross = d1.x() * d2.y() - d1.y() * d2.x();
        Scalar a = d1.norm(), b = d2.norm(), c = (p2 - p0).norm();
        Scalar denom = a * b * c;
        if (denom > constants::kEpsilon) {
            Scalar curvature = static_cast<Scalar>(2.0) * std::abs(cross) / denom;
            max_k = std::max(max_k, curvature);
        }
    }
    return max_k;
}

Scalar Trajectory2D::smoothness() const noexcept {
    if (waypoints_.size() < 3) return 0;
    Scalar total = 0;
    for (std::size_t i = 1; i + 1 < waypoints_.size(); ++i) {
        Scalar dtheta1 = angularDistance(waypoints_[i - 1].theta, waypoints_[i].theta);
        Scalar dtheta2 = angularDistance(waypoints_[i].theta, waypoints_[i + 1].theta);
        total += sq(dtheta2 - dtheta1);
    }
    return total;
}

Waypoint2D Trajectory2D::interpolateAtTime(Scalar t) const {
    if (waypoints_.empty()) throw std::runtime_error("Empty trajectory");
    if (waypoints_.size() == 1 || t <= waypoints_.front().t) return waypoints_.front();
    if (t >= waypoints_.back().t) return waypoints_.back();

    // Binary search for the segment
    std::size_t lo = 0, hi = waypoints_.size() - 1;
    while (lo + 1 < hi) {
        std::size_t mid = (lo + hi) / 2;
        if (waypoints_[mid].t <= t) lo = mid;
        else hi = mid;
    }

    const auto& w0 = waypoints_[lo];
    const auto& w1 = waypoints_[hi];
    Scalar alpha = (t - w0.t) / (w1.t - w0.t);

    return {
        lerp(w0.x, w1.x, alpha),
        lerp(w0.y, w1.y, alpha),
        normalizeAngle(w0.theta + alpha * angularDistance(w0.theta, w1.theta)),
        t
    };
}

Trajectory2D Trajectory2D::resample(std::size_t n) const {
    if (n < 2 || waypoints_.size() < 2) return *this;

    Trajectory2D resampled;
    resampled.reserve(n);
    Scalar t_start = waypoints_.front().t;
    Scalar t_end = waypoints_.back().t;
    for (std::size_t i = 0; i < n; ++i) {
        Scalar t = t_start + static_cast<Scalar>(i) / static_cast<Scalar>(n - 1) * (t_end - t_start);
        resampled.append(interpolateAtTime(t));
    }
    return resampled;
}

}  // namespace kinetra
