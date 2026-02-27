// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/planners/lattice.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>

namespace kinetra::planners {

// ═════════════════════════════════════════════════════════════════════════════
// Motion Primitive Generation
// ═════════════════════════════════════════════════════════════════════════════

void LatticePlanner::generatePrimitives() {
    primitives_.clear();
    int N = options_.numAngles;
    int P = options_.primitivesPerAngle;
    Scalar L = options_.primitiveLength;

    for (int theta_idx = 0; theta_idx < N; ++theta_idx) {
        Scalar theta = angleFromIdx(theta_idx);

        for (int p = 0; p < P; ++p) {
            // Spread steering angles from -maxSteer to +maxSteer
            Scalar steer = (P == 1) ? Scalar(0) :
                options_.maxSteerAngle * (static_cast<Scalar>(2 * p) /
                static_cast<Scalar>(P - 1) - Scalar(1));

            MotionPrimitive prim;
            prim.start_theta_idx = theta_idx;

            // Simulate arc: if steer ≈ 0, straight line; else circular arc
            constexpr int kSteps = 5;
            Scalar dt_step = L / static_cast<Scalar>(kSteps);
            Scalar cx = 0, cy = 0, ctheta = theta;

            for (int s = 0; s < kSteps; ++s) {
                cx += dt_step * std::cos(ctheta);
                cy += dt_step * std::sin(ctheta);
                ctheta += steer * (dt_step / L);
                prim.intermediate.push_back({
                    cx, cy, normalizeAngle(ctheta), Scalar(0)
                });
            }

            prim.dx = cx;
            prim.dy = cy;
            prim.end_theta_idx = normalizeAngleIdx(
                static_cast<int>(std::round(normalizeAngle(ctheta) /
                    (constants::kTwoPi / static_cast<Scalar>(N)))));
            prim.cost = L * (Scalar(1) + options_.steerChangePenalty * std::abs(steer));
            primitives_.push_back(std::move(prim));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// A* Search
// ═════════════════════════════════════════════════════════════════════════════

PlanningResult LatticePlanner::solve(const PlanningProblem& problem) {
    PlanningResult result;
    result.plannerName = "Lattice";
    auto start_time = Clock::now();

    auto point_check = point_check_ ? point_check_ :
        [](const Vec2&) { return true; };
    auto segment_check = segment_check_ ? segment_check_ :
        [](const Vec2&, const Vec2&) { return true; };

    LatticeState start = worldToLattice(problem.start.x, problem.start.y,
                                         problem.start.theta);
    LatticeState goal  = worldToLattice(problem.goal.x, problem.goal.y,
                                         problem.goal.theta);

    // Goal tolerance in lattice cells
    Scalar goal_tol = problem.options.goalTolerance;
    int goal_tol_cells = std::max(0, static_cast<int>(
        std::ceil(goal_tol / options_.xyResolution)));

    // A* data structures
    struct AStarNode {
        LatticeState state;
        Scalar g_cost;   // cost-so-far
        Scalar f_cost;   // g + h
        int parent_idx;  // index into closed list (-1 for start)
        int prim_idx;    // which primitive got us here
    };

    auto cmp = [](const AStarNode& a, const AStarNode& b) {
        return a.f_cost > b.f_cost;
    };
    std::priority_queue<AStarNode, std::vector<AStarNode>, decltype(cmp)> open(cmp);
    std::unordered_map<LatticeState, Scalar, LatticeStateHash> g_scores;
    std::vector<AStarNode> closed;  // for path reconstruction

    // Seed the search
    Scalar h0 = heuristic(start, goal);
    open.push({start, 0, h0, -1, -1});
    g_scores[start] = 0;

    int expansions = 0;
    int goal_idx = -1;

    while (!open.empty()) {
        // Time check
        auto elapsed = std::chrono::duration<double, std::milli>(
            Clock::now() - start_time).count();
        if (elapsed > options_.timeLimitMs) {
            result.status = SolveStatus::kTimeout;
            break;
        }

        if (expansions >= options_.maxExpansions) {
            result.status = SolveStatus::kMaxIterations;
            break;
        }

        AStarNode current = open.top();
        open.pop();
        ++expansions;

        // Skip if we already found a better path to this state
        auto it = g_scores.find(current.state);
        if (it != g_scores.end() && current.g_cost > it->second + constants::kEpsilon) {
            continue;
        }

        // Record in closed list
        int current_idx = static_cast<int>(closed.size());
        closed.push_back(current);

        // Goal check (with tolerance)
        int dgx = std::abs(current.state.gx - goal.gx);
        int dgy = std::abs(current.state.gy - goal.gy);
        if (dgx <= goal_tol_cells && dgy <= goal_tol_cells) {
            goal_idx = current_idx;
            break;
        }

        // Expand: try all primitives that start from current heading
        SE2State world_state = latticeToWorld(current.state);

        for (int pi = 0; pi < static_cast<int>(primitives_.size()); ++pi) {
            const auto& prim = primitives_[static_cast<std::size_t>(pi)];
            if (prim.start_theta_idx != current.state.theta_idx) continue;

            // Compute successor world position
            Scalar cos_t = std::cos(world_state.theta);
            Scalar sin_t = std::sin(world_state.theta);
            Scalar nx = world_state.x + cos_t * prim.dx - sin_t * prim.dy;
            Scalar ny = world_state.y + sin_t * prim.dx + cos_t * prim.dy;

            // Discretize successor
            LatticeState succ;
            succ.gx = static_cast<int>(std::round(nx / options_.xyResolution));
            succ.gy = static_cast<int>(std::round(ny / options_.xyResolution));
            succ.theta_idx = prim.end_theta_idx;

            // Collision check on intermediate points
            bool collision = false;
            Vec2 prev_pos(world_state.x, world_state.y);
            for (const auto& wp : prim.intermediate) {
                Scalar wx = world_state.x + cos_t * wp.x - sin_t * wp.y;
                Scalar wy = world_state.y + sin_t * wp.x + cos_t * wp.y;
                Vec2 pos(wx, wy);
                if (!point_check(pos) || !segment_check(prev_pos, pos)) {
                    collision = true;
                    break;
                }
                prev_pos = pos;
            }
            if (collision) continue;

            Scalar new_g = current.g_cost + prim.cost;
            auto git = g_scores.find(succ);
            if (git != g_scores.end() && new_g >= git->second) continue;

            g_scores[succ] = new_g;
            Scalar h = options_.heuristicWeight * heuristic(succ, goal);
            open.push({succ, new_g, new_g + h, current_idx, pi});
        }
    }

    // Extract result
    auto end_time = Clock::now();
    result.solveTimeMs = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    result.iterations = expansions;

    if (goal_idx >= 0) {
        result.status = SolveStatus::kSuccess;

        // Reconstruct path (reverse)
        std::vector<int> node_indices;
        int ci = goal_idx;
        while (ci >= 0) {
            node_indices.push_back(ci);
            ci = closed[static_cast<std::size_t>(ci)].parent_idx;
        }
        std::reverse(node_indices.begin(), node_indices.end());

        // Build trajectory with intermediate waypoints
        Trajectory2D traj;
        Scalar t = 0;
        for (std::size_t i = 0; i < node_indices.size(); ++i) {
            const auto& node = closed[static_cast<std::size_t>(node_indices[i])];
            if (node.prim_idx >= 0 && i > 0) {
                // Append intermediate points from the motion primitive
                const auto& prim = primitives_[static_cast<std::size_t>(node.prim_idx)];
                SE2State parent_world = latticeToWorld(
                    closed[static_cast<std::size_t>(node.parent_idx)].state);
                Scalar cos_t = std::cos(parent_world.theta);
                Scalar sin_t = std::sin(parent_world.theta);

                for (const auto& wp : prim.intermediate) {
                    Scalar wx = parent_world.x + cos_t * wp.x - sin_t * wp.y;
                    Scalar wy = parent_world.y + sin_t * wp.x + cos_t * wp.y;
                    Scalar wt = normalizeAngle(parent_world.theta + wp.theta -
                                                angleFromIdx(prim.start_theta_idx));
                    t += options_.xyResolution * Scalar(0.5);
                    traj.append({wx, wy, wt, t});
                }
            } else {
                SE2State ws = latticeToWorld(node.state);
                traj.append({ws.x, ws.y, ws.theta, t});
            }
        }

        result.trajectory = traj;
        result.cost = closed[static_cast<std::size_t>(goal_idx)].g_cost;
        result.pathLength = traj.pathLength();
    } else if (result.status != SolveStatus::kTimeout) {
        result.status = SolveStatus::kMaxIterations;
    }

    return result;
}

void LatticePlanner::reset() {
    // No persistent state beyond primitives (regenerated on construction)
}

// ═════════════════════════════════════════════════════════════════════════════
// Helpers
// ═════════════════════════════════════════════════════════════════════════════

LatticePlanner::LatticeState
LatticePlanner::worldToLattice(Scalar x, Scalar y, Scalar theta) const {
    int N = options_.numAngles;
    Scalar angle_res = constants::kTwoPi / static_cast<Scalar>(N);
    int theta_idx = static_cast<int>(std::round(normalizeAngle(theta) / angle_res));
    theta_idx = normalizeAngleIdx(theta_idx);
    return {
        static_cast<int>(std::round(x / options_.xyResolution)),
        static_cast<int>(std::round(y / options_.xyResolution)),
        theta_idx
    };
}

SE2State LatticePlanner::latticeToWorld(const LatticeState& s) const {
    return {
        static_cast<Scalar>(s.gx) * options_.xyResolution,
        static_cast<Scalar>(s.gy) * options_.xyResolution,
        angleFromIdx(s.theta_idx)
    };
}

Scalar LatticePlanner::heuristic(const LatticeState& a, const LatticeState& b) const {
    // Euclidean distance + angular component
    Scalar dx = static_cast<Scalar>(b.gx - a.gx) * options_.xyResolution;
    Scalar dy = static_cast<Scalar>(b.gy - a.gy) * options_.xyResolution;
    Scalar dtheta = std::abs(angularDistance(angleFromIdx(a.theta_idx),
                                             angleFromIdx(b.theta_idx)));
    return std::sqrt(dx * dx + dy * dy) + Scalar(0.5) * dtheta;
}

int LatticePlanner::normalizeAngleIdx(int idx) const noexcept {
    int N = options_.numAngles;
    return ((idx % N) + N) % N;
}

Scalar LatticePlanner::angleFromIdx(int idx) const noexcept {
    return static_cast<Scalar>(idx) * constants::kTwoPi /
           static_cast<Scalar>(options_.numAngles) - constants::kPi;
}

bool LatticePlanner::checkPrimitiveCollision(
    const SE2State& from, const MotionPrimitive& prim) const {
    if (!point_check_) return false;
    Scalar cos_t = std::cos(from.theta);
    Scalar sin_t = std::sin(from.theta);

    Vec2 prev(from.x, from.y);
    for (const auto& wp : prim.intermediate) {
        Scalar wx = from.x + cos_t * wp.x - sin_t * wp.y;
        Scalar wy = from.y + sin_t * wp.x + cos_t * wp.y;
        Vec2 pos(wx, wy);
        if (!point_check_(pos)) return true;
        if (segment_check_ && !segment_check_(prev, pos)) return true;
        prev = pos;
    }
    return false;
}

}  // namespace kinetra::planners
