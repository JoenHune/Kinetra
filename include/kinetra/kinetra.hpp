// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// Kinetra — World-class standalone trajectory planning library.
// Umbrella header for convenient inclusion.

#pragma once

// Core
#include "kinetra/core/types.hpp"
#include "kinetra/core/concepts.hpp"
#include "kinetra/core/bounds.hpp"
#include "kinetra/core/trajectory.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/reference_path.hpp"

// State Spaces
#include "kinetra/spaces/se2.hpp"
#include "kinetra/spaces/real_vector.hpp"
#include "kinetra/spaces/dubins.hpp"

// Robot Models
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"

// Solvers
#include "kinetra/solvers/qp_admm.hpp"
#include "kinetra/solvers/lqr.hpp"

// Planners
#include "kinetra/planners/rrt_star.hpp"
#include "kinetra/planners/stomp.hpp"
#include "kinetra/planners/ilqr.hpp"
#include "kinetra/planners/mpcc.hpp"

// Optimization Framework
#include "kinetra/optimization/nlp_problem.hpp"

// Collision
#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/collision/polygon_obstacles.hpp"

// IO
#include "kinetra/io/json_export.hpp"
#include "kinetra/io/scenario_loader.hpp"
