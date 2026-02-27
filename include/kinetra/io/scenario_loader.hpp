// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Scenario loader — reads YAML/JSON scenario definitions for test automation.

#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "kinetra/core/result.hpp"

namespace kinetra::io {

/// Load a planning problem from a JSON scenario file.
[[nodiscard]] PlanningProblem loadScenarioJSON(const std::filesystem::path& path);

/// Load all scenarios from a directory.
[[nodiscard]] std::vector<PlanningProblem> loadScenariosFromDir(
    const std::filesystem::path& dir);

/// Save a planning problem to a JSON scenario file.
void saveScenarioJSON(const std::filesystem::path& path,
                      const PlanningProblem& problem);

}  // namespace kinetra::io
