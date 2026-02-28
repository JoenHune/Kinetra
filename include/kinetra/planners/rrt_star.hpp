// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// RRT* planner — asymptotically optimal sampling-based planner.

#pragma once

#include <functional>
#include <memory>
#include <random>
#include <vector>

#include "kinetra/core/concepts.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/types.hpp"
#include "kinetra/spaces/se2.hpp"

namespace kinetra {
class DubinsSpace;  // forward declaration
}  // namespace kinetra

namespace kinetra::planners {

struct RRTStarOptions {
    int maxIterations{5000};
    Scalar stepSize{static_cast<Scalar>(0.5)};
    Scalar goalBias{static_cast<Scalar>(0.05)};     // Probability of sampling goal
    Scalar rewireRadius{static_cast<Scalar>(2.0)};   // Radius for rewiring
    bool useKNearest{true};                           // Use k-nearest instead of radius
    double timeLimitMs{5000.0};

    /// Enable Dubins steering for kinematically feasible paths.
    /// When true, RRT* uses Dubins curves instead of straight-line
    /// interpolation, producing paths that respect a minimum turning radius.
    bool useDubinsSteering{false};
    Scalar turningRadius{static_cast<Scalar>(1.0)};
    Scalar dubinsCollisionStep{static_cast<Scalar>(0.1)};  // sampling step for path checking
};

/// Type-erased collision checker for 2D points
using CollisionCheckFn = std::function<bool(const Vec2&)>;
using SegmentCheckFn   = std::function<bool(const Vec2&, const Vec2&)>;

/// RRT* planner for SE(2) state space.
class RRTStar {
public:
    RRTStar() = default;
    explicit RRTStar(RRTStarOptions options) : options_(std::move(options)) {}

    /// Plan a path from start to goal.
    [[nodiscard]] PlanningResult solve(const PlanningProblem& problem);

    /// Set collision checker
    void setCollisionChecker(CollisionCheckFn point_check, SegmentCheckFn segment_check) {
        point_check_ = std::move(point_check);
        segment_check_ = std::move(segment_check);
    }

    [[nodiscard]] std::string_view name() const noexcept { return "RRT*"; }
    void reset();

    [[nodiscard]] const RRTStarOptions& options() const noexcept { return options_; }
    RRTStarOptions& options() noexcept { return options_; }

    /// Get number of nodes in the tree (for diagnostics)
    [[nodiscard]] std::size_t treeSize() const noexcept;

private:
    struct Node {
        SE2State state;
        int parent{-1};
        Scalar cost{0};
        std::vector<int> children;
    };

    RRTStarOptions options_;
    std::vector<Node> tree_;
    CollisionCheckFn point_check_;
    SegmentCheckFn segment_check_;
    mutable std::mt19937 gen_{std::random_device{}()};

    // Internal helpers
    [[nodiscard]] int nearest(const SE2State& state, const SE2Space& space) const;
    [[nodiscard]] std::vector<int> nearNeighbors(const SE2State& state,
                                                  const SE2Space& space,
                                                  Scalar radius) const;
    [[nodiscard]] SE2State steer(const SE2State& from, const SE2State& to,
                                  const SE2Space& space) const;
    void rewire(int new_node, const std::vector<int>& neighbors,
                const SE2Space& space);
    [[nodiscard]] Trajectory2D extractPath(int goal_node) const;
    [[nodiscard]] Trajectory2D extractPathDubins(
        int goal_node, const DubinsSpace& dubins) const;
};

}  // namespace kinetra::planners
