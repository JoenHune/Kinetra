// SPDX-License-Identifier: BSD-3-Clause
// Kinetra Basic Planning Example
// Demonstrates: creating a scene, running RRT* and STOMP, exporting results.

#include <fstream>
#include <iostream>

#include "kinetra/kinetra.hpp"

int main() {
    using namespace kinetra;

    // ── Define environment ───────────────────────────────────────────────────
    PlanningProblem problem;
    problem.start = {-8, 0, 0, 0};
    problem.goal  = {8, 0, 0, 0};
    problem.environment.bounds_min = Vec2(-10, -10);
    problem.environment.bounds_max = Vec2(10, 10);
    problem.environment.obstacles.push_back(
        CircleObstacle{Vec2(0, 2), 1.5});
    problem.environment.obstacles.push_back(
        RectangleObstacle{Vec2(-1, -4), Vec2(1, -1)});
    problem.options.goalTolerance = 0.5;

    // ── Create occupancy grid from obstacles ─────────────────────────────────
    collision::OccupancyGrid2D grid(-10, -10, 10, 10, 0.1);
    grid.addCircleObstacle(Vec2(0, 2), 1.5);
    grid.addRectObstacle(Vec2(-1, -4), Vec2(1, -1));

    // ── Run RRT* ─────────────────────────────────────────────────────────────
    planners::RRTStarOptions rrt_opts;
    rrt_opts.maxIterations = 5000;
    rrt_opts.stepSize = 0.5;

    planners::RRTStar rrt(rrt_opts);
    rrt.setCollisionChecker(
        [&](const Vec2& p) { return grid.isFree(p); },
        [&](const Vec2& a, const Vec2& b) { return grid.isSegmentFree(a, b); }
    );

    auto rrt_result = rrt.solve(problem);
    std::cout << "RRT*: " << toString(rrt_result.status)
              << " | time=" << rrt_result.solveTimeMs << "ms"
              << " | path_length=" << rrt_result.pathLength
              << " | nodes=" << rrt.treeSize() << std::endl;

    // ── Run STOMP ────────────────────────────────────────────────────────────
    planners::STOMPOptions stomp_opts;
    stomp_opts.numTimesteps = 40;
    stomp_opts.maxIterations = 50;

    planners::STOMP stomp(stomp_opts);
    stomp.setCostFunction([&](const MatX& traj) -> VecX {
        VecX costs = VecX::Zero(traj.rows());
        for (int i = 0; i < traj.rows(); ++i) {
            Vec2 pos(traj(i, 0), traj(i, 1));
            Scalar d = grid.signedDistance(pos);
            if (d < 0.5) {  // Safety margin
                costs[i] = 100.0 * (0.5 - d) * (0.5 - d);
            }
        }
        return costs;
    });

    auto stomp_result = stomp.solve(problem);
    std::cout << "STOMP: " << toString(stomp_result.status)
              << " | time=" << stomp_result.solveTimeMs << "ms"
              << " | path_length=" << stomp_result.pathLength << std::endl;

    // ── Export visualization HTML ────────────────────────────────────────────
    if (rrt_result.success()) {
        std::ofstream ofs("rrt_result.html");
        ofs << io::generateVisualizationHTML(problem, rrt_result);
        std::cout << "Wrote rrt_result.html" << std::endl;
    }
    if (stomp_result.success()) {
        std::ofstream ofs("stomp_result.html");
        ofs << io::generateVisualizationHTML(problem, stomp_result);
        std::cout << "Wrote stomp_result.html" << std::endl;
    }

    return 0;
}
