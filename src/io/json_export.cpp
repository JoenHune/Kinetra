// SPDX-License-Identifier: BSD-3-Clause
#include "kinetra/io/json_export.hpp"

#include <iomanip>
#include <sstream>

namespace kinetra::io {

void toJSON(std::ostream& os, const Trajectory2D& trajectory,
            const std::string& label) {
    os << "{\"" << label << "\":[";
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        if (i > 0) os << ",";
        const auto& wp = trajectory[i];
        os << "{\"x\":" << wp.x
           << ",\"y\":" << wp.y
           << ",\"theta\":" << wp.theta
           << ",\"t\":" << wp.t << "}";
    }
    os << "]}";
}

void toJSON(std::ostream& os, const PlanningResult& result) {
    os << "{";
    os << "\"status\":\"" << toString(result.status) << "\",";
    os << "\"planner\":\"" << result.plannerName << "\",";
    os << "\"solve_time_ms\":" << result.solveTimeMs << ",";
    os << "\"iterations\":" << result.iterations << ",";
    os << "\"cost\":" << result.cost << ",";
    os << "\"path_length\":" << result.pathLength << ",";
    os << "\"max_curvature\":" << result.maxCurvature << ",";
    os << "\"min_clearance\":" << result.minClearance << ",";
    os << "\"smoothness\":" << result.smoothness << ",";
    os << "\"trajectory\":[";
    for (std::size_t i = 0; i < result.trajectory.size(); ++i) {
        if (i > 0) os << ",";
        const auto& wp = result.trajectory[i];
        os << "{\"x\":" << wp.x
           << ",\"y\":" << wp.y
           << ",\"theta\":" << wp.theta
           << ",\"t\":" << wp.t << "}";
    }
    os << "]}";
}

void toJSON(std::ostream& os, const Environment2D& env) {
    os << "{\"bounds\":{\"min\":[" << env.bounds_min.x() << ","
       << env.bounds_min.y() << "],\"max\":[" << env.bounds_max.x() << ","
       << env.bounds_max.y() << "]},\"obstacles\":[";
    for (std::size_t i = 0; i < env.obstacles.size(); ++i) {
        if (i > 0) os << ",";
        std::visit([&](auto&& obs) {
            using T = std::decay_t<decltype(obs)>;
            if constexpr (std::is_same_v<T, CircleObstacle>) {
                os << "{\"type\":\"circle\",\"center\":["
                   << obs.center.x() << "," << obs.center.y()
                   << "],\"radius\":" << obs.radius << "}";
            } else if constexpr (std::is_same_v<T, RectangleObstacle>) {
                os << "{\"type\":\"rectangle\",\"min\":["
                   << obs.min_corner.x() << "," << obs.min_corner.y()
                   << "],\"max\":[" << obs.max_corner.x() << ","
                   << obs.max_corner.y() << "]}";
            } else if constexpr (std::is_same_v<T, PolygonObstacle>) {
                os << "{\"type\":\"polygon\",\"vertices\":[";
                for (std::size_t j = 0; j < obs.vertices.size(); ++j) {
                    if (j > 0) os << ",";
                    os << "[" << obs.vertices[j].x() << ","
                       << obs.vertices[j].y() << "]";
                }
                os << "]}";
            }
        }, env.obstacles[i]);
    }
    os << "]}";
}

void toJSON(std::ostream& os, const PlanningProblem& problem,
            const PlanningResult& result) {
    os << "{\"problem\":{";
    os << "\"start\":{\"x\":" << problem.start.x
       << ",\"y\":" << problem.start.y
       << ",\"theta\":" << problem.start.theta << "},";
    os << "\"goal\":{\"x\":" << problem.goal.x
       << ",\"y\":" << problem.goal.y
       << ",\"theta\":" << problem.goal.theta << "},";
    os << "\"environment\":";
    toJSON(os, problem.environment);
    os << "},\"result\":";
    toJSON(os, result);
    os << "}";
}

void benchmarkToJSON(std::ostream& os,
                     const std::vector<PlanningResult>& results) {
    os << "[";
    for (std::size_t i = 0; i < results.size(); ++i) {
        if (i > 0) os << ",";
        toJSON(os, results[i]);
    }
    os << "]";
}

std::string generateVisualizationHTML(const PlanningProblem& problem,
                                       const PlanningResult& result) {
    std::ostringstream os;
    os << R"(<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Kinetra Trajectory Visualization</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script></head>
<body><div id="plot" style="width:800px;height:600px;"></div><script>
const data = )";

    // Embed JSON data
    toJSON(os, problem, result);

    os << R"(;
// Extract trajectory
const traj = data.result.trajectory;
const trace = {x: traj.map(p => p.x), y: traj.map(p => p.y),
    mode: 'lines+markers', name: data.result.planner, line: {width: 2}};

// Obstacles
const shapes = [];
if (data.problem.environment && data.problem.environment.obstacles) {
    data.problem.environment.obstacles.forEach(obs => {
        if (obs.type === 'circle') {
            shapes.push({type: 'circle', xref: 'x', yref: 'y',
                x0: obs.center[0]-obs.radius, y0: obs.center[1]-obs.radius,
                x1: obs.center[0]+obs.radius, y1: obs.center[1]+obs.radius,
                fillcolor: 'rgba(50,50,50,0.3)', line: {color: 'black'}});
        } else if (obs.type === 'rectangle') {
            shapes.push({type: 'rect', xref: 'x', yref: 'y',
                x0: obs.min[0], y0: obs.min[1], x1: obs.max[0], y1: obs.max[1],
                fillcolor: 'rgba(50,50,50,0.3)', line: {color: 'black'}});
        }
    });
}

// Start/Goal markers
const markers = {x: [data.problem.start.x, data.problem.goal.x],
    y: [data.problem.start.y, data.problem.goal.y],
    mode: 'markers', marker: {size: 12, color: ['green','red']},
    name: 'Start/Goal'};

Plotly.newPlot('plot', [trace, markers], {
    title: `${data.result.planner} — ${data.result.solve_time_ms.toFixed(1)}ms, cost=${data.result.cost.toFixed(3)}`,
    shapes: shapes,
    xaxis: {title: 'X (m)', scaleanchor: 'y'},
    yaxis: {title: 'Y (m)'}
});
</script></body></html>)";

    return os.str();
}

}  // namespace kinetra::io
