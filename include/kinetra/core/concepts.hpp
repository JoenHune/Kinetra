// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// C++20 concepts defining the core abstractions of the Kinetra library.
// These replace virtual interfaces with zero-cost compile-time polymorphism,
// critical for embedded deployment (ARMv7: no vtable indirection, full inlining).

#pragma once

#include <concepts>
#include <cstddef>
#include <span>
#include <type_traits>

#include "kinetra/core/types.hpp"

namespace kinetra {

// ═════════════════════════════════════════════════════════════════════════════
// StateSpace Concept
// ═════════════════════════════════════════════════════════════════════════════
// Inspired by OMPL's StateSpace hierarchy, but enforced at compile time.
// A StateSpace defines a metric space with distance, interpolation, sampling,
// and dimensionality.

template <typename S>
concept StateSpace = requires(const S& space,
                              typename S::StateType s1,
                              typename S::StateType s2,
                              Scalar t) {
    // Associated types
    typename S::StateType;

    // Compile-time dimension (0 = dynamic)
    { S::kDimension } -> std::convertible_to<std::size_t>;

    // Metric distance between two states
    { space.distance(s1, s2) } -> std::convertible_to<Scalar>;

    // Interpolate between states: t ∈ [0, 1]
    { space.interpolate(s1, s2, t) } -> std::same_as<typename S::StateType>;

    // Dimension query (runtime, may differ from kDimension for dynamic spaces)
    { space.dimension() } -> std::convertible_to<std::size_t>;

    // Bounds checking
    { space.isValid(s1) } -> std::convertible_to<bool>;
};

// ═════════════════════════════════════════════════════════════════════════════
// SamplableSpace Concept (extension of StateSpace)
// ═════════════════════════════════════════════════════════════════════════════
// Spaces that support uniform random sampling. Required for sampling-based
// planners (RRT*, RRTConnect, PRM*).

template <typename S>
concept SamplableSpace = StateSpace<S> && requires(const S& space) {
    { space.sampleUniform() } -> std::same_as<typename S::StateType>;
};

// ═════════════════════════════════════════════════════════════════════════════
// RobotModel Concept
// ═════════════════════════════════════════════════════════════════════════════
// Defines robot kinematics/dynamics for trajectory optimization.
// Supports differential drive, Ackermann, omnidirectional, and custom models.
// Multi-order modeling: propagation can include velocity, accel, jerk, snap.

template <typename M>
concept RobotModel = requires(const M& model,
                              typename M::StateType state,
                              typename M::ControlType control,
                              Scalar dt) {
    // Associated types
    typename M::StateType;
    typename M::ControlType;

    // State and control dimensions
    { M::kStateDim } -> std::convertible_to<std::size_t>;
    { M::kControlDim } -> std::convertible_to<std::size_t>;

    // Forward dynamics: x_{k+1} = f(x_k, u_k, dt)
    { model.dynamics(state, control, dt) } -> std::same_as<typename M::StateType>;

    // State bounds (min/max)
    { model.stateLowerBound() } -> std::same_as<typename M::StateType>;
    { model.stateUpperBound() } -> std::same_as<typename M::StateType>;

    // Control bounds (min/max)
    { model.controlLowerBound() } -> std::same_as<typename M::ControlType>;
    { model.controlUpperBound() } -> std::same_as<typename M::ControlType>;
};

// Extension: Robot models with linearization support (required for iLQR/DDP)
template <typename M>
concept LinearizableModel = RobotModel<M> && requires(
    const M& model,
    typename M::StateType state,
    typename M::ControlType control,
    Scalar dt) {
    // Jacobians: A = df/dx, B = df/du
    { model.jacobianState(state, control, dt) };    // Returns state-sized matrix
    { model.jacobianControl(state, control, dt) };  // Returns control-sized matrix
};

// ═════════════════════════════════════════════════════════════════════════════
// CostFunction Concept
// ═════════════════════════════════════════════════════════════════════════════
// Defines cost terms for trajectory optimization.

template <typename C>
concept CostFunction = requires(const C& cost, VecX x) {
    { cost.evaluate(x) } -> std::convertible_to<Scalar>;
};

template <typename C>
concept DifferentiableCost = CostFunction<C> && requires(const C& cost, VecX x) {
    { cost.gradient(x) } -> std::same_as<VecX>;
};

template <typename C>
concept TwiceDifferentiableCost = DifferentiableCost<C> && requires(const C& cost, VecX x) {
    { cost.hessian(x) } -> std::same_as<MatX>;
};

// ═════════════════════════════════════════════════════════════════════════════
// Constraint Concept
// ═════════════════════════════════════════════════════════════════════════════
// User-defined constraint factors for the optimization problem.
// Inspired by ifopt's composable ConstraintSet pattern.

template <typename C>
concept Constraint = requires(const C& constraint, VecX x) {
    // Evaluate constraint: g(x), where lb <= g(x) <= ub
    { constraint.evaluate(x) } -> std::same_as<VecX>;

    // Bounds
    { constraint.lowerBound() } -> std::same_as<VecX>;
    { constraint.upperBound() } -> std::same_as<VecX>;

    // Constraint dimension
    { constraint.dimension() } -> std::convertible_to<std::size_t>;
};

template <typename C>
concept DifferentiableConstraint = Constraint<C> && requires(const C& c, VecX x) {
    // Jacobian: dg/dx (sparse or dense)
    { c.jacobian(x) } -> std::same_as<MatX>;
};

// ═════════════════════════════════════════════════════════════════════════════
// CollisionChecker Concept
// ═════════════════════════════════════════════════════════════════════════════
// Environment collision checking. Supports both discrete point checking
// and continuous segment checking.

template <typename C>
concept CollisionChecker = requires(const C& checker, Vec2 point) {
    // Check if a point is collision-free
    { checker.isFree(point) } -> std::convertible_to<bool>;

    // Signed distance to nearest obstacle (positive = free, negative = collision)
    { checker.signedDistance(point) } -> std::convertible_to<Scalar>;
};

template <typename C>
concept ContinuousCollisionChecker = CollisionChecker<C> &&
    requires(const C& checker, Vec2 from, Vec2 to) {
    // Check if the segment [from, to] is collision-free
    { checker.isSegmentFree(from, to) } -> std::convertible_to<bool>;
};

// ═════════════════════════════════════════════════════════════════════════════
// Planner Concept
// ═════════════════════════════════════════════════════════════════════════════
// High-level planner interface. Planners take a problem and produce a result.

// Forward declarations
struct PlanningProblem;
struct PlanningResult;

template <typename P>
concept Planner = requires(P& planner, const PlanningProblem& problem) {
    { planner.name() } -> std::convertible_to<std::string_view>;
    { planner.solve(problem) } -> std::same_as<PlanningResult>;
    { planner.reset() } -> std::same_as<void>;
};

// Planners that support iterative refinement
template <typename P>
concept AnytimePlanner = Planner<P> && requires(P& planner) {
    { planner.iterate() } -> std::convertible_to<bool>;  // returns true if improved
    { planner.bestCost() } -> std::convertible_to<Scalar>;
};

// ═════════════════════════════════════════════════════════════════════════════
// Solver Concept (low-level numerical solver)
// ═════════════════════════════════════════════════════════════════════════════

template <typename S>
concept QPSolver = requires(S& solver, MatX Q, VecX c, MatX A, VecX lb, VecX ub) {
    { solver.setup(Q, c, A, lb, ub) } -> std::convertible_to<bool>;
    { solver.solve() } -> std::same_as<VecX>;
    { solver.iterations() } -> std::convertible_to<int>;
};

}  // namespace kinetra
