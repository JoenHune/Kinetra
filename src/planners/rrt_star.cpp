// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/spaces/dubins.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace kinetra::planners {

PlanningResult RRTStar::solve(const PlanningProblem& problem) {
    PlanningResult result;
    result.plannerName = "RRT*";
    auto start_time = Clock::now();

    SE2Space space(problem.environment.bounds_min.x(),
                   problem.environment.bounds_max.x(),
                   problem.environment.bounds_min.y(),
                   problem.environment.bounds_max.y());

    SE2State start_state{problem.start.x, problem.start.y, problem.start.theta};
    SE2State goal_state{problem.goal.x, problem.goal.y, problem.goal.theta};

    // ── Dubins space (created only when enabled) ─────────────────────────
    std::optional<DubinsSpace> dubins;
    if (options_.useDubinsSteering) {
        dubins.emplace(options_.turningRadius,
                       problem.environment.bounds_min.x(),
                       problem.environment.bounds_max.x(),
                       problem.environment.bounds_min.y(),
                       problem.environment.bounds_max.y());
    }

    // Initialize tree with start node
    reset();
    tree_.push_back(Node{start_state, -1, 0, {}});

    // Default collision checker if none set
    auto point_check = point_check_ ? point_check_ :
        [&](const Vec2&) { return true; };
    auto segment_check = segment_check_ ? segment_check_ :
        [&](const Vec2&, const Vec2&) { return true; };

    // ── Dubins-aware helper lambdas ──────────────────────────────────────

    // Distance between two SE2 states (Dubins or Euclidean)
    auto edgeCost = [&](const SE2State& a, const SE2State& b) -> Scalar {
        if (dubins) return dubins->distance(a, b);
        return space.distance(a, b);
    };

    // Steer from `from` towards `to`, respecting stepSize
    auto steerTo = [&](const SE2State& from,
                       const SE2State& to) -> SE2State {
        if (dubins) {
            auto path = dubins->shortestPath(from, to);
            if (!path) return from;
            if (path->totalLength() <= options_.stepSize) return to;
            Scalar t = options_.stepSize / path->totalLength();
            return dubins->interpolate(from, to, t);
        }
        return steer(from, to, space);
    };

    // Check if the path from `from` to `to` is collision-free
    auto isEdgeFree = [&](const SE2State& from,
                          const SE2State& to) -> bool {
        if (dubins) {
            auto path = dubins->shortestPath(from, to);
            if (!path) return false;
            auto pts = dubins->samplePath(
                from, *path, options_.dubinsCollisionStep);
            for (const auto& p : pts) {
                if (!point_check(p.position())) return false;
            }
            return true;
        }
        return point_check(to.position())
            && segment_check(from.position(), to.position());
    };

    int best_goal_node = -1;
    Scalar best_cost = constants::kInfinity;

    for (int iter = 0; iter < options_.maxIterations; ++iter) {
        // Check time limit
        auto elapsed = std::chrono::duration<double, std::milli>(
            Clock::now() - start_time).count();
        if (elapsed > options_.timeLimitMs) {
            result.status = SolveStatus::kTimeout;
            break;
        }

        // Sample random state (with goal bias)
        SE2State rand_state;
        std::uniform_real_distribution<Scalar> bias_dist(0, 1);
        if (bias_dist(gen_) < options_.goalBias) {
            rand_state = goal_state;
        } else {
            rand_state = space.sampleUniform();
        }

        // Find nearest node (SE2 Euclidean — fast lower bound for Dubins)
        int nearest_idx = nearest(rand_state, space);
        if (nearest_idx < 0) continue;

        // Steer towards sampled state
        SE2State new_state = steerTo(
            tree_[static_cast<std::size_t>(nearest_idx)].state, rand_state);

        // Collision-aware edge check
        if (!isEdgeFree(
                tree_[static_cast<std::size_t>(nearest_idx)].state,
                new_state)) {
            continue;
        }

        // Compute cost via actual edge metric
        Scalar new_cost =
            tree_[static_cast<std::size_t>(nearest_idx)].cost
            + edgeCost(
                  tree_[static_cast<std::size_t>(nearest_idx)].state,
                  new_state);

        // Find near neighbors for rewiring (SE2 distance for search)
        Scalar radius = options_.rewireRadius;
        if (options_.useKNearest) {
            auto n = static_cast<Scalar>(tree_.size());
            radius = options_.rewireRadius *
                     std::pow(std::log(n + 1) / (n + 1),
                              static_cast<Scalar>(1.0 / 3.0));
        }
        auto neighbors = nearNeighbors(new_state, space, radius);

        // Choose best parent among neighbors
        int best_parent = nearest_idx;
        Scalar best_new_cost = new_cost;
        for (int nb : neighbors) {
            Scalar nb_cost =
                tree_[static_cast<std::size_t>(nb)].cost
                + edgeCost(tree_[static_cast<std::size_t>(nb)].state,
                           new_state);
            if (nb_cost < best_new_cost
                && isEdgeFree(
                       tree_[static_cast<std::size_t>(nb)].state,
                       new_state)) {
                best_parent = nb;
                best_new_cost = nb_cost;
            }
        }

        // Add new node
        int new_idx = static_cast<int>(tree_.size());
        tree_.push_back(Node{new_state, best_parent, best_new_cost, {}});
        tree_[static_cast<std::size_t>(best_parent)].children.push_back(
            new_idx);

        // Rewire neighbors (with Dubins-aware collision check)
        for (int nb : neighbors) {
            if (nb == tree_[static_cast<std::size_t>(new_idx)].parent)
                continue;

            Scalar potential_cost =
                tree_[static_cast<std::size_t>(new_idx)].cost
                + edgeCost(
                      tree_[static_cast<std::size_t>(new_idx)].state,
                      tree_[static_cast<std::size_t>(nb)].state);

            if (potential_cost <
                tree_[static_cast<std::size_t>(nb)].cost) {
                if (!isEdgeFree(
                        tree_[static_cast<std::size_t>(new_idx)].state,
                        tree_[static_cast<std::size_t>(nb)].state))
                    continue;

                // Remove nb from old parent's children
                int old_parent =
                    tree_[static_cast<std::size_t>(nb)].parent;
                if (old_parent >= 0) {
                    auto& children =
                        tree_[static_cast<std::size_t>(old_parent)]
                            .children;
                    children.erase(
                        std::remove(children.begin(), children.end(), nb),
                        children.end());
                }

                // Rewire
                tree_[static_cast<std::size_t>(nb)].parent = new_idx;
                tree_[static_cast<std::size_t>(nb)].cost = potential_cost;
                tree_[static_cast<std::size_t>(new_idx)]
                    .children.push_back(nb);
            }
        }

        // Check if we reached the goal
        if (space.distance(new_state, goal_state) <
            problem.options.goalTolerance) {
            if (best_new_cost < best_cost) {
                best_goal_node = new_idx;
                best_cost = best_new_cost;
            }
        }
    }

    // ── Extract result ───────────────────────────────────────────────────
    auto end_time = Clock::now();
    result.solveTimeMs = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    result.iterations = static_cast<int>(tree_.size());

    if (best_goal_node >= 0) {
        result.status = SolveStatus::kSuccess;
        if (dubins) {
            result.trajectory =
                extractPathDubins(best_goal_node, *dubins);
        } else {
            result.trajectory = extractPath(best_goal_node);
        }
        result.cost = best_cost;
        result.pathLength = result.trajectory.pathLength();
    } else if (result.status != SolveStatus::kTimeout) {
        result.status = SolveStatus::kMaxIterations;
    }

    return result;
}

void RRTStar::reset() {
    tree_.clear();
}

std::size_t RRTStar::treeSize() const noexcept {
    return tree_.size();
}

int RRTStar::nearest(const SE2State& state, const SE2Space& space) const {
    if (tree_.empty()) return -1;
    int best = 0;
    Scalar best_dist = constants::kInfinity;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        Scalar d = space.distance(tree_[static_cast<std::size_t>(i)].state, state);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

std::vector<int> RRTStar::nearNeighbors(const SE2State& state,
                                          const SE2Space& space,
                                          Scalar radius) const {
    std::vector<int> result;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (space.distance(tree_[static_cast<std::size_t>(i)].state, state) < radius) {
            result.push_back(i);
        }
    }
    return result;
}

SE2State RRTStar::steer(const SE2State& from, const SE2State& to,
                          const SE2Space& space) const {
    Scalar d = space.distance(from, to);
    if (d <= options_.stepSize) return to;
    Scalar t = options_.stepSize / d;
    return space.interpolate(from, to, t);
}

void RRTStar::rewire(int new_node, const std::vector<int>& neighbors,
                       const SE2Space& space) {
    for (int nb : neighbors) {
        if (nb == tree_[static_cast<std::size_t>(new_node)].parent) continue;

        Scalar potential_cost = tree_[static_cast<std::size_t>(new_node)].cost
                                + space.distance(tree_[static_cast<std::size_t>(new_node)].state,
                                                 tree_[static_cast<std::size_t>(nb)].state);

        if (potential_cost < tree_[static_cast<std::size_t>(nb)].cost) {
            // Check collision for rewired edge
            Vec2 from_pos = tree_[static_cast<std::size_t>(new_node)].state.position();
            Vec2 to_pos = tree_[static_cast<std::size_t>(nb)].state.position();
            if (segment_check_ && !segment_check_(from_pos, to_pos)) continue;

            // Remove nb from old parent's children
            int old_parent = tree_[static_cast<std::size_t>(nb)].parent;
            if (old_parent >= 0) {
                auto& children = tree_[static_cast<std::size_t>(old_parent)].children;
                children.erase(std::remove(children.begin(), children.end(), nb),
                               children.end());
            }

            // Rewire
            tree_[static_cast<std::size_t>(nb)].parent = new_node;
            tree_[static_cast<std::size_t>(nb)].cost = potential_cost;
            tree_[static_cast<std::size_t>(new_node)].children.push_back(nb);
        }
    }
}

Trajectory2D RRTStar::extractPath(int goal_node) const {
    std::vector<Waypoint2D> waypoints;
    int node = goal_node;
    while (node >= 0) {
        const auto& s = tree_[static_cast<std::size_t>(node)].state;
        waypoints.push_back({s.x, s.y, s.theta, 0});
        node = tree_[static_cast<std::size_t>(node)].parent;
    }
    std::reverse(waypoints.begin(), waypoints.end());

    // Assign time stamps (proportional to distance)
    Scalar t = 0;
    if (!waypoints.empty()) {
        waypoints[0].t = 0;
        for (std::size_t i = 1; i < waypoints.size(); ++i) {
            Scalar dx = waypoints[i].x - waypoints[i - 1].x;
            Scalar dy = waypoints[i].y - waypoints[i - 1].y;
            t += std::sqrt(dx * dx + dy * dy);
            waypoints[i].t = t;
        }
    }
    return Trajectory2D(std::move(waypoints));
}

Trajectory2D RRTStar::extractPathDubins(
    int goal_node, const DubinsSpace& dubins) const {

    // Trace parent chain → node sequence (root → goal)
    std::vector<int> chain;
    int node = goal_node;
    while (node >= 0) {
        chain.push_back(node);
        node = tree_[static_cast<std::size_t>(node)].parent;
    }
    std::reverse(chain.begin(), chain.end());

    std::vector<Waypoint2D> waypoints;
    Scalar t = 0;

    for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
        const auto& from = tree_[static_cast<std::size_t>(chain[i])].state;
        const auto& to   = tree_[static_cast<std::size_t>(chain[i + 1])].state;
        auto path = dubins.shortestPath(from, to);
        if (!path) {
            // Fallback: straight-line
            if (i == 0)
                waypoints.push_back({from.x, from.y, from.theta, t});
            waypoints.push_back({to.x, to.y, to.theta, t + Scalar(0.1)});
            t += Scalar(0.1);
            continue;
        }
        auto pts = dubins.samplePath(from, *path, Scalar(0.1));
        std::size_t start_idx = (i == 0) ? 0 : 1;  // skip duplicate
        for (std::size_t j = start_idx; j < pts.size(); ++j) {
            waypoints.push_back({pts[j].x, pts[j].y, pts[j].theta, t});
            t += Scalar(0.1);
        }
    }

    // Handle single-node path
    if (chain.size() == 1) {
        const auto& s = tree_[static_cast<std::size_t>(chain[0])].state;
        waypoints.push_back({s.x, s.y, s.theta, 0});
    }

    return Trajectory2D(std::move(waypoints));
}

}  // namespace kinetra::planners
