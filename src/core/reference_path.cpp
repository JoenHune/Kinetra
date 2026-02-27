// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors

#include "kinetra/core/reference_path.hpp"

#include <algorithm>
#include <cmath>

namespace kinetra {

ReferencePath::ReferencePath(const std::vector<Waypoint2D>& waypoints)
    : ReferencePath(waypoints, Options{}) {}

ReferencePath::ReferencePath(const std::vector<Waypoint2D>& waypoints,
                             Options opts)
    : waypoints_(waypoints), opts_(std::move(opts)) {
    buildArcLengthTable();
}

ReferencePath::ReferencePath(const Trajectory2D& trajectory)
    : ReferencePath(trajectory, Options{}) {}

ReferencePath::ReferencePath(const Trajectory2D& trajectory, Options opts)
    : opts_(std::move(opts)) {
    waypoints_.reserve(trajectory.size());
    for (const auto& wp : trajectory.waypoints()) {
        waypoints_.push_back(wp);
    }
    buildArcLengthTable();
}

void ReferencePath::buildArcLengthTable() {
    arc_lengths_.resize(waypoints_.size());
    if (waypoints_.empty()) return;
    arc_lengths_[0] = Scalar(0);
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        Scalar dx = waypoints_[i].x - waypoints_[i - 1].x;
        Scalar dy = waypoints_[i].y - waypoints_[i - 1].y;
        Scalar dth = angularDistance(waypoints_[i - 1].theta, waypoints_[i].theta);
        Scalar ds = std::sqrt(dx * dx + dy * dy +
                              opts_.lengthTheta * opts_.lengthTheta * dth * dth);
        arc_lengths_[i] = arc_lengths_[i - 1] + ds;
    }
}

std::size_t ReferencePath::findSegment(Scalar s) const {
    if (arc_lengths_.size() <= 1) return 0;
    s = clamp(s, Scalar(0), arc_lengths_.back());
    auto it = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
    if (it == arc_lengths_.begin()) return 0;
    std::size_t idx = static_cast<std::size_t>(it - arc_lengths_.begin()) - 1;
    return std::min(idx, waypoints_.size() - 2);
}

Vec3 ReferencePath::evaluate(Scalar s) const {
    if (waypoints_.empty()) return Vec3::Zero();
    if (waypoints_.size() == 1) {
        return Vec3(waypoints_[0].x, waypoints_[0].y, waypoints_[0].theta);
    }
    s = clamp(s, Scalar(0), arc_lengths_.back());
    std::size_t seg = findSegment(s);

    Scalar seg_len = arc_lengths_[seg + 1] - arc_lengths_[seg];
    Scalar t = (seg_len > constants::kEpsilon)
                   ? (s - arc_lengths_[seg]) / seg_len
                   : Scalar(0);
    t = clamp(t, Scalar(0), Scalar(1));

    const auto& w0 = waypoints_[seg];
    const auto& w1 = waypoints_[seg + 1];

    Scalar x = w0.x + t * (w1.x - w0.x);
    Scalar y = w0.y + t * (w1.y - w0.y);
    Scalar th = w0.theta + t * angularDistance(w0.theta, w1.theta);
    th = normalizeAngle(th);

    return Vec3(x, y, th);
}

Vec2 ReferencePath::tangent(Scalar s) const {
    if (waypoints_.size() < 2) return Vec2(1, 0);
    s = clamp(s, Scalar(0), arc_lengths_.back());
    std::size_t seg = findSegment(s);

    const auto& w0 = waypoints_[seg];
    const auto& w1 = waypoints_[seg + 1];
    Scalar dx = w1.x - w0.x;
    Scalar dy = w1.y - w0.y;
    Scalar norm = std::sqrt(dx * dx + dy * dy);

    if (norm < constants::kEpsilon) {
        // Pure rotation — use heading-based tangent
        Scalar th = w0.theta;
        return Vec2(std::cos(th), std::sin(th));
    }
    return Vec2(dx / norm, dy / norm);
}

Vec2 ReferencePath::normal(Scalar s) const {
    Vec2 t = tangent(s);
    return Vec2(-t.y(), t.x());
}

Scalar ReferencePath::thetaDot(Scalar s) const {
    if (waypoints_.size() < 2) return Scalar(0);
    s = clamp(s, Scalar(0), arc_lengths_.back());
    std::size_t seg = findSegment(s);

    Scalar seg_len = arc_lengths_[seg + 1] - arc_lengths_[seg];
    if (seg_len < constants::kEpsilon) return Scalar(0);

    Scalar dth = angularDistance(waypoints_[seg].theta, waypoints_[seg + 1].theta);
    return dth / seg_len;
}

Scalar ReferencePath::curvature(Scalar s) const {
    if (waypoints_.size() < 2) return Scalar(0);
    s = clamp(s, Scalar(0), arc_lengths_.back());
    std::size_t seg = findSegment(s);

    const auto& w0 = waypoints_[seg];
    const auto& w1 = waypoints_[seg + 1];
    Scalar dx = w1.x - w0.x;
    Scalar dy = w1.y - w0.y;
    Scalar seg_len = arc_lengths_[seg + 1] - arc_lengths_[seg];

    if (seg_len < constants::kEpsilon) return Scalar(0);

    Scalar pos_len = std::sqrt(dx * dx + dy * dy);
    if (pos_len < constants::kEpsilon) return Scalar(0);

    // Curvature from heading change per positional arc length
    Scalar dth = angularDistance(w0.theta, w1.theta);
    Scalar kappa = dth / pos_len;
    return clamp(kappa, -opts_.maxCurvature, opts_.maxCurvature);
}

}  // namespace kinetra
