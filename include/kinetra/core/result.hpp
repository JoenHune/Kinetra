// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Planning problem definition and result types.

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "kinetra/core/trajectory.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra {

// ─── Solver Status ───────────────────────────────────────────────────────────
enum class SolveStatus {
    kSuccess,              // Found a valid solution
    kTimeout,              // Time limit exceeded
    kMaxIterations,        // Iteration limit exceeded
    kInfeasible,           // Problem proven infeasible
    kNumericalFailure,     // Solver encountered numerical issues
    kInvalidProblem,       // Problem definition is invalid
    kNotSolved,            // Solve has not been called
};

[[nodiscard]] constexpr std::string_view toString(SolveStatus s) noexcept {
    switch (s) {
        case SolveStatus::kSuccess:          return "Success";
        case SolveStatus::kTimeout:          return "Timeout";
        case SolveStatus::kMaxIterations:    return "MaxIterations";
        case SolveStatus::kInfeasible:       return "Infeasible";
        case SolveStatus::kNumericalFailure: return "NumericalFailure";
        case SolveStatus::kInvalidProblem:   return "InvalidProblem";
        case SolveStatus::kNotSolved:        return "NotSolved";
    }
    return "Unknown";
}

// ─── Infeasibility Report ────────────────────────────────────────────────────
// When the solver reports infeasibility, this provides detailed information
// about WHICH constraints are violated and by how much.
struct InfeasibilityReport {
    struct ViolatedConstraint {
        std::string name;           // Name of the constraint
        Scalar violation{0};        // Amount of violation
        std::string description;    // Human-readable description
    };

    std::vector<ViolatedConstraint> violations;
    std::string summary;

    [[nodiscard]] bool hasViolations() const noexcept {
        return !violations.empty();
    }
};

// ─── Planning Result ─────────────────────────────────────────────────────────
struct PlanningResult {
    SolveStatus status{SolveStatus::kNotSolved};
    Trajectory2D trajectory;

    // Performance metrics
    double solveTimeMs{0};        // Wall-clock planning time (milliseconds)
    int iterations{0};            // Number of solver iterations
    Scalar cost{constants::kInfinity};  // Final objective value

    // Quality metrics
    Scalar pathLength{0};
    Scalar maxCurvature{0};
    Scalar minClearance{0};       // Minimum distance to obstacles
    Scalar smoothness{0};

    // Infeasibility details (populated when status == kInfeasible)
    std::optional<InfeasibilityReport> infeasibilityReport;

    // Planner info
    std::string plannerName;

    [[nodiscard]] bool success() const noexcept {
        return status == SolveStatus::kSuccess;
    }
};

// ─── Planner Options ─────────────────────────────────────────────────────────
struct PlannerOptions {
    int maxIterations{1000};
    double timeLimitMs{5000.0};         // Maximum planning time (ms)
    Scalar tolerance{static_cast<Scalar>(1e-6)};
    Scalar goalTolerance{static_cast<Scalar>(0.1)};  // Position tolerance to goal
    Scalar goalAngleTolerance{static_cast<Scalar>(0.1)};  // Angle tolerance (rad)
    bool verbose{false};
};

// ─── Obstacle Definitions ────────────────────────────────────────────────────
struct CircleObstacle {
    Vec2 center;
    Scalar radius;
};

struct RectangleObstacle {
    Vec2 min_corner;  // Bottom-left
    Vec2 max_corner;  // Top-right
};

struct PolygonObstacle {
    std::vector<Vec2> vertices;  // Counter-clockwise ordered
};

using Obstacle = std::variant<CircleObstacle, RectangleObstacle, PolygonObstacle>;

// ─── Environment ─────────────────────────────────────────────────────────────
struct Environment2D {
    Vec2 bounds_min{-10, -10};  // Workspace lower bound
    Vec2 bounds_max{10, 10};    // Workspace upper bound
    std::vector<Obstacle> obstacles;
};

// ─── Planning Problem ────────────────────────────────────────────────────────
struct PlanningProblem {
    // Start and goal (required)
    Waypoint2D start;
    Waypoint2D goal;

    // Environment
    Environment2D environment;

    // Planner options
    PlannerOptions options;

    // Initial guess trajectory (optional, for optimization-based planners)
    std::optional<Trajectory2D> initialGuess;
};

}  // namespace kinetra
