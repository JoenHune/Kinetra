// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// JSON export utilities for trajectories, benchmarks, and scenarios.
// Output feeds the GitHub Pages visualization dashboard.

#pragma once

#include <ostream>
#include <string>

#include "kinetra/core/result.hpp"
#include "kinetra/core/trajectory.hpp"
#include "kinetra/core/types.hpp"

namespace kinetra::io {

/// Export a Trajectory2D as JSON.
void toJSON(std::ostream& os, const Trajectory2D& trajectory,
            const std::string& label = "trajectory");

/// Export a PlanningResult as JSON (trajectory + metrics).
void toJSON(std::ostream& os, const PlanningResult& result);

/// Export an Environment2D as JSON (obstacles + bounds).
void toJSON(std::ostream& os, const Environment2D& env);

/// Export a complete scenario (problem + result) as JSON.
void toJSON(std::ostream& os, const PlanningProblem& problem,
            const PlanningResult& result);

/// Export benchmark comparison (multiple results) as JSON.
void benchmarkToJSON(std::ostream& os,
                     const std::vector<PlanningResult>& results);

/// Generate a self-contained HTML file with Plotly.js visualization.
/// Useful for quick debugging and sharing.
[[nodiscard]] std::string generateVisualizationHTML(
    const PlanningProblem& problem,
    const PlanningResult& result);

}  // namespace kinetra::io
