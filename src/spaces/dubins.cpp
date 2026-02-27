// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/spaces/dubins.hpp"

#include <algorithm>
#include <cmath>

namespace kinetra {

// ─── Helper: mod2pi ──────────────────────────────────────────────────────────
static Scalar mod2pi(Scalar angle) {
    while (angle < 0) angle += constants::kTwoPi;
    while (angle >= constants::kTwoPi) angle -= constants::kTwoPi;
    return angle;
}

// ─── Compute individual Dubins path type in normalized coordinates ───────────
std::optional<DubinsPath> DubinsSpace::computePath(
    DubinsPathType type, Scalar d, Scalar alpha, Scalar beta) const {

    Scalar sa = std::sin(alpha), ca = std::cos(alpha);
    Scalar sb = std::sin(beta),  cb = std::cos(beta);
    Scalar cab = std::cos(alpha - beta);

    std::array<Scalar, 3> lengths{};

    switch (type) {
    case DubinsPathType::LSL: {
        Scalar p_sq = 2 + d * d - 2 * cab + 2 * d * (sa - sb);
        if (p_sq < 0) return std::nullopt;
        Scalar tmp = std::atan2(cb - ca, d + sa - sb);
        lengths[0] = mod2pi(tmp - alpha);
        lengths[1] = std::sqrt(p_sq);
        lengths[2] = mod2pi(beta - tmp);
        break;
    }
    case DubinsPathType::RSR: {
        Scalar p_sq = 2 + d * d - 2 * cab + 2 * d * (sb - sa);
        if (p_sq < 0) return std::nullopt;
        Scalar tmp = std::atan2(ca - cb, d - sa + sb);
        lengths[0] = mod2pi(alpha - tmp);
        lengths[1] = std::sqrt(p_sq);
        lengths[2] = mod2pi(tmp - beta);
        break;
    }
    case DubinsPathType::LSR: {
        Scalar p_sq = -2 + d * d + 2 * cab + 2 * d * (sa + sb);
        if (p_sq < 0) return std::nullopt;
        Scalar p = std::sqrt(p_sq);
        Scalar tmp = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2, p);
        lengths[0] = mod2pi(tmp - alpha);
        lengths[1] = p;
        lengths[2] = mod2pi(tmp - mod2pi(beta));
        break;
    }
    case DubinsPathType::RSL: {
        Scalar p_sq = -2 + d * d + 2 * cab - 2 * d * (sa + sb);
        if (p_sq < 0) return std::nullopt;
        Scalar p = std::sqrt(p_sq);
        Scalar tmp = std::atan2(ca + cb, d - sa - sb) - std::atan2(2, p);
        lengths[0] = mod2pi(alpha - tmp);
        lengths[1] = p;
        lengths[2] = mod2pi(beta - tmp);
        break;
    }
    case DubinsPathType::RLR: {
        Scalar tmp = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8;
        if (std::abs(tmp) > 1) return std::nullopt;
        Scalar p = mod2pi(std::acos(tmp));
        lengths[0] = mod2pi(alpha - std::atan2(ca - cb, d - sa + sb) + p / 2);
        lengths[1] = p;
        lengths[2] = mod2pi(alpha - beta - lengths[0] + p);
        break;
    }
    case DubinsPathType::LRL: {
        Scalar tmp = (6 - d * d + 2 * cab + 2 * d * (sb - sa)) / 8;
        if (std::abs(tmp) > 1) return std::nullopt;
        Scalar p = mod2pi(std::acos(tmp));
        lengths[0] = mod2pi(-alpha + std::atan2(-ca + cb, d + sa - sb) + p / 2);
        lengths[1] = p;
        lengths[2] = mod2pi(beta - alpha - lengths[0] + p);
        break;
    }
    }

    return DubinsPath{type, lengths};
}

// ─── Shortest Dubins path ────────────────────────────────────────────────────
std::optional<DubinsPath> DubinsSpace::shortestPath(
    const SE2State& from, const SE2State& to) const {

    Scalar dx = to.x - from.x;
    Scalar dy = to.y - from.y;
    Scalar d = std::sqrt(dx * dx + dy * dy) / rho_;
    Scalar theta = std::atan2(dy, dx);
    Scalar alpha = mod2pi(from.theta - theta);
    Scalar beta  = mod2pi(to.theta - theta);

    std::optional<DubinsPath> best;
    Scalar best_length = constants::kInfinity;

    for (auto type : {DubinsPathType::LSL, DubinsPathType::LSR,
                      DubinsPathType::RSL, DubinsPathType::RSR,
                      DubinsPathType::RLR, DubinsPathType::LRL}) {
        auto path = computePath(type, d, alpha, beta);
        if (path && path->totalLength() < best_length) {
            best = path;
            best_length = path->totalLength();
        }
    }

    if (best) {
        // Scale from normalized to actual lengths
        best->lengths[0] *= rho_;
        best->lengths[1] *= rho_;
        best->lengths[2] *= rho_;
    }
    return best;
}

// ─── All valid paths ─────────────────────────────────────────────────────────
std::vector<DubinsPath> DubinsSpace::allPaths(
    const SE2State& from, const SE2State& to) const {

    Scalar dx = to.x - from.x;
    Scalar dy = to.y - from.y;
    Scalar d = std::sqrt(dx * dx + dy * dy) / rho_;
    Scalar theta = std::atan2(dy, dx);
    Scalar alpha = mod2pi(from.theta - theta);
    Scalar beta  = mod2pi(to.theta - theta);

    std::vector<DubinsPath> results;
    for (auto type : {DubinsPathType::LSL, DubinsPathType::LSR,
                      DubinsPathType::RSL, DubinsPathType::RSR,
                      DubinsPathType::RLR, DubinsPathType::LRL}) {
        auto path = computePath(type, d, alpha, beta);
        if (path) {
            path->lengths[0] *= rho_;
            path->lengths[1] *= rho_;
            path->lengths[2] *= rho_;
            results.push_back(*path);
        }
    }
    return results;
}

// ─── Distance ────────────────────────────────────────────────────────────────
Scalar DubinsSpace::distance(const SE2State& from, const SE2State& to) const {
    auto path = shortestPath(from, to);
    return path ? path->totalLength() : constants::kInfinity;
}

// ─── Sample points along path ────────────────────────────────────────────────
std::vector<SE2State> DubinsSpace::samplePath(
    const SE2State& from, const DubinsPath& path, Scalar step_size) const {

    std::vector<SE2State> samples;
    Scalar total = path.totalLength();
    int n = std::max(2, static_cast<int>(total / step_size) + 1);
    samples.reserve(static_cast<std::size_t>(n));

    // For each sample point, walk along the 3 segments
    auto segmentType = [&](int seg) -> int {
        // Returns: -1=right, 0=straight, 1=left
        switch (path.type) {
            case DubinsPathType::LSL: return (seg == 1) ? 0 : 1;
            case DubinsPathType::LSR: return (seg == 0) ? 1 : (seg == 1) ? 0 : -1;
            case DubinsPathType::RSL: return (seg == 0) ? -1 : (seg == 1) ? 0 : 1;
            case DubinsPathType::RSR: return (seg == 1) ? 0 : -1;
            case DubinsPathType::RLR: return (seg == 1) ? 1 : -1;
            case DubinsPathType::LRL: return (seg == 1) ? -1 : 1;
        }
        return 0;
    };

    SE2State current = from;
    Scalar accumulated = 0;
    int sample_idx = 0;

    for (int seg = 0; seg < 3; ++seg) {
        Scalar seg_len = path.lengths[static_cast<std::size_t>(seg)];
        int turn = segmentType(seg);
        Scalar seg_start = accumulated;

        while (sample_idx < n) {
            Scalar target = static_cast<Scalar>(sample_idx) / static_cast<Scalar>(n - 1) * total;
            if (target > seg_start + seg_len + constants::kEpsilon) break;

            Scalar ds = target - seg_start;
            SE2State s;
            if (turn == 0) {
                // Straight
                s.x = current.x + ds * std::cos(current.theta);
                s.y = current.y + ds * std::sin(current.theta);
                s.theta = current.theta;
            } else {
                // Arc: left (turn=1) or right (turn=-1)
                Scalar signed_rho = static_cast<Scalar>(turn) * rho_;
                Scalar dphi = ds / rho_;
                Scalar cx = current.x - signed_rho * std::sin(current.theta);
                Scalar cy = current.y + signed_rho * std::cos(current.theta);
                Scalar new_theta = current.theta + static_cast<Scalar>(turn) * dphi;
                s.x = cx + signed_rho * std::sin(new_theta);
                s.y = cy - signed_rho * std::cos(new_theta);
                s.theta = normalizeAngle(new_theta);
            }
            samples.push_back(s);
            ++sample_idx;
        }

        // Advance current state to end of segment
        if (turn == 0) {
            current.x += seg_len * std::cos(current.theta);
            current.y += seg_len * std::sin(current.theta);
        } else {
            Scalar signed_rho = static_cast<Scalar>(turn) * rho_;
            Scalar dphi = seg_len / rho_;
            Scalar cx = current.x - signed_rho * std::sin(current.theta);
            Scalar cy = current.y + signed_rho * std::cos(current.theta);
            Scalar new_theta = current.theta + static_cast<Scalar>(turn) * dphi;
            current.x = cx + signed_rho * std::sin(new_theta);
            current.y = cy - signed_rho * std::cos(new_theta);
            current.theta = normalizeAngle(new_theta);
        }
        accumulated += seg_len;
    }

    return samples;
}

// ─── Interpolation ───────────────────────────────────────────────────────────
SE2State DubinsSpace::interpolate(const SE2State& from, const SE2State& to,
                                   Scalar t) const {
    auto path = shortestPath(from, to);
    if (!path) return from;

    auto samples = samplePath(from, *path, path->totalLength() / 100);
    if (samples.empty()) return from;

    int idx = static_cast<int>(t * static_cast<Scalar>(samples.size() - 1));
    idx = std::clamp(idx, 0, static_cast<int>(samples.size()) - 1);
    return samples[static_cast<std::size_t>(idx)];
}

}  // namespace kinetra
