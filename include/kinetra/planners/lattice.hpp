// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// State Lattice Planner — graph-based planning with motion primitives.
// Uses A* search over a discretized SE(2) state lattice. Each edge
// corresponds to a pre-computed kinematically-feasible motion primitive.
// Used by Apollo, Autoware and other industrial-grade planning stacks.

#pragma once

#include <functional>
#include <vector>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/types.hpp"
#include "kinetra/spaces/se2.hpp"

namespace kinetra::planners {

// ── Motion Primitive ─────────────────────────────────────────────────────────
/// A short kinematically-feasible trajectory segment connecting
/// a discretized start heading to a discretized end heading.
struct MotionPrimitive {
    int start_theta_idx{0};    // Discrete heading index at start
    int end_theta_idx{0};      // Discrete heading index at end
    Scalar dx{0};              // x displacement (in start frame)
    Scalar dy{0};              // y displacement (in start frame)
    Scalar cost{0};            // Traversal cost (e.g. arc length)
    std::vector<Waypoint2D> intermediate;  // Intermediate points for collision check
};

// ── Options ──────────────────────────────────────────────────────────────────
struct LatticeOptions {
    // Grid discretization
    Scalar xyResolution{static_cast<Scalar>(0.5)};    // Spatial grid cell size (m)
    int numAngles{16};                                  // Number of heading discretizations

    // Search parameters
    int maxExpansions{50000};           // Maximum A* node expansions
    double timeLimitMs{5000.0};
    Scalar heuristicWeight{static_cast<Scalar>(1.5)};  // Weighted A* (1.0 = optimal)

    // Motion primitive generation
    Scalar maxSteerAngle{static_cast<Scalar>(0.6)};    // Max steering per primitive (rad)
    Scalar primitiveLength{static_cast<Scalar>(1.0)};  // Length of each primitive
    int primitivesPerAngle{5};                          // Number of primitives per heading

    // Penalty weights
    Scalar reversePenalty{static_cast<Scalar>(2.0)};   // Cost multiplier for reverse
    Scalar steerChangePenalty{static_cast<Scalar>(0.5)}; // Penalty for steering change
};

/// Type-erased collision checker callbacks (same as RRT*)
using CollisionCheckFn = std::function<bool(const Vec2&)>;
using SegmentCheckFn   = std::function<bool(const Vec2&, const Vec2&)>;

// ── Lattice Planner ──────────────────────────────────────────────────────────
/// State lattice planner with A* search.
///
/// Algorithm:
///   1. Discretize workspace into (x, y, θ) lattice
///   2. Generate motion primitives connecting lattice nodes
///   3. A* search from start to goal with SE(2) heuristic
///   4. Extract and refine the path
class LatticePlanner {
public:
    LatticePlanner() { generatePrimitives(); }
    explicit LatticePlanner(LatticeOptions options)
        : options_(std::move(options)) { generatePrimitives(); }

    /// Plan a path from start to goal.
    [[nodiscard]] PlanningResult solve(const PlanningProblem& problem);

    /// Set collision checker (same interface as RRT*)
    void setCollisionChecker(CollisionCheckFn point_check, SegmentCheckFn segment_check) {
        point_check_ = std::move(point_check);
        segment_check_ = std::move(segment_check);
    }

    [[nodiscard]] std::string_view name() const noexcept { return "Lattice"; }
    void reset();

    [[nodiscard]] const LatticeOptions& options() const noexcept { return options_; }
    LatticeOptions& options() noexcept { return options_; }

    /// Access generated motion primitives
    [[nodiscard]] const std::vector<MotionPrimitive>& primitives() const { return primitives_; }

private:
    LatticeOptions options_;
    std::vector<MotionPrimitive> primitives_;
    CollisionCheckFn point_check_;
    SegmentCheckFn segment_check_;

    /// Generate motion primitives for all heading discretizations
    void generatePrimitives();

    // ── Lattice state ────────────────────────────────────────────────────────
    struct LatticeState {
        int gx, gy;          // Grid cell indices
        int theta_idx;        // Discrete heading index

        bool operator==(const LatticeState& o) const noexcept {
            return gx == o.gx && gy == o.gy && theta_idx == o.theta_idx;
        }
    };

    struct LatticeStateHash {
        std::size_t operator()(const LatticeState& s) const noexcept {
            auto h1 = std::hash<int>{}(s.gx);
            auto h2 = std::hash<int>{}(s.gy);
            auto h3 = std::hash<int>{}(s.theta_idx);
            return h1 ^ (h2 << 11) ^ (h3 << 22);
        }
    };

    // ── Helpers ──────────────────────────────────────────────────────────────
    [[nodiscard]] LatticeState worldToLattice(Scalar x, Scalar y, Scalar theta) const;
    [[nodiscard]] SE2State latticeToWorld(const LatticeState& s) const;
    [[nodiscard]] Scalar heuristic(const LatticeState& a, const LatticeState& b) const;
    [[nodiscard]] int normalizeAngleIdx(int idx) const noexcept;
    [[nodiscard]] Scalar angleFromIdx(int idx) const noexcept;
    [[nodiscard]] bool checkPrimitiveCollision(const SE2State& from,
                                                const MotionPrimitive& prim) const;
};

}  // namespace kinetra::planners
