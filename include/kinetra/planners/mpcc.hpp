// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 Kinetra Authors
//
// SE(2)-MPCC — Model Predictive Contouring Control for path tracking.
//
// Given a reference path from a global planner (RRT*, etc.), MPCC optimises
// a dynamically-feasible trajectory that tracks the path while maximising
// progress along it. The progress variable s_k is a decision variable —
// the optimiser simultaneously determines the robot motion AND which part
// of the reference path to track.
//
// Uses the existing SQP solver (BFGS + ADMM QP) via NLPProblem composition.
// Works with any LinearizableModel (DiffDrive, Ackermann, Omni).
//
// See journey/004_se2_mpcc.md for the full mathematical formulation.

#pragma once

#include <string_view>

#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/core/concepts.hpp"
#include "kinetra/core/reference_path.hpp"
#include "kinetra/core/result.hpp"
#include "kinetra/core/types.hpp"
#include "kinetra/optimization/nlp_problem.hpp"
#include "kinetra/solvers/sqp.hpp"

namespace kinetra::planners {

// ─── MPCC Options ────────────────────────────────────────────────────────────
struct MPCCOptions {
    // Horizon
    int horizon{50};
    Scalar dt{static_cast<Scalar>(0.05)};
    double timeLimitMs{5000.0};

    // Tracking weights
    Scalar wContour{static_cast<Scalar>(50.0)};    // Lateral error
    Scalar wLag{static_cast<Scalar>(20.0)};         // Longitudinal error
    Scalar wHeading{static_cast<Scalar>(30.0)};     // Heading error
    Scalar wProgress{static_cast<Scalar>(5.0)};     // λ: progress reward

    // Control weights
    Scalar wControl{static_cast<Scalar>(1.0)};      // Control effort

    // Obstacle avoidance
    Scalar safeDistance{static_cast<Scalar>(0.3)};   // d_safe
    Scalar wObstacle{static_cast<Scalar>(100.0)};    // Barrier weight

    // Progress constraints
    Scalar maxProgressRate{static_cast<Scalar>(1.5)};  // v̄_s
    Scalar maxLagError{static_cast<Scalar>(2.0)};      // ē_l

    // Reference path options
    ReferencePath::Options pathOptions;

    // SQP solver settings
    solvers::SQPSettings sqpSettings;

    MPCCOptions() {
        sqpSettings.maxIterations = 30;
        sqpSettings.tolerance = 1e-4;
        sqpSettings.constraintTolerance = 1e-4;
        sqpSettings.verbose = false;
        sqpSettings.timeLimitMs = 5000.0;
    }
};

// ─── MPCC Result (extended PlanningResult) ───────────────────────────────────
struct MPCCResult {
    PlanningResult planningResult;

    // MPCC-specific metrics
    std::vector<Scalar> progressValues;    // s_0 ... s_N
    std::vector<Scalar> contourErrors;     // e_c at each step
    std::vector<Scalar> lagErrors;         // e_l at each step
    std::vector<Scalar> headingErrors;     // e_θ at each step

    // Control inputs (per step)
    std::vector<VecX> controls;            // u_0 ... u_{N-1}

    // SQP solver info
    int sqpIterations{0};
    Scalar finalCost{0};
    Scalar constraintViolation{0};
};

/// SE(2)-MPCC — Model Predictive Contouring Control.
///
/// Template parameter Model must satisfy LinearizableModel concept.
///
/// Usage:
///   // 1. Get reference path from RRT* or other planner
///   ReferencePath refPath(rrtResult.trajectory);
///
///   // 2. Create MPCC
///   MPCC<DiffDriveAccel> mpcc(model, refPath, options);
///
///   // 3. Solve
///   auto result = mpcc.solve(problem);
///   // result.planningResult contains the optimised trajectory
///   // result.progressValues, contourErrors, etc. for analysis
template <typename Model>
    requires LinearizableModel<Model>
class MPCC {
public:
    static constexpr int kStateDim = Model::kStateDim;
    static constexpr int kControlDim = Model::kControlDim;
    using StateType = typename Model::StateType;
    using ControlType = typename Model::ControlType;

    MPCC(Model model, ReferencePath refPath, MPCCOptions options = {})
        : model_(std::move(model)),
          refPath_(std::move(refPath)),
          options_(std::move(options)) {}

    /// Solve MPCC trajectory optimisation.
    /// Uses start from problem, tracks the reference path.
    [[nodiscard]] MPCCResult solve(const PlanningProblem& problem);

    /// Solve MPCC with explicit obstacle grid (for SDF-based avoidance).
    [[nodiscard]] MPCCResult solve(const PlanningProblem& problem,
                                   const collision::OccupancyGrid2D& grid);

    [[nodiscard]] std::string_view name() const noexcept { return "MPCC"; }
    void reset();

    [[nodiscard]] const MPCCOptions& options() const noexcept { return options_; }
    MPCCOptions& options() noexcept { return options_; }

    [[nodiscard]] const ReferencePath& referencePath() const noexcept { return refPath_; }

private:
    Model model_;
    ReferencePath refPath_;
    MPCCOptions options_;

    // Build the NLP for the MPCC problem
    optimization::NLPProblem buildNLP(const StateType& x0,
                                      Scalar s_init,
                                      const collision::OccupancyGrid2D* grid) const;

    // Extract result from solved NLP
    MPCCResult extractResult(const optimization::NLPProblem& nlp,
                             const solvers::SQPResult& sqpResult,
                             double totalTimeMs) const;

    // Initialise decision variables (straight-line or warm-start)
    void initialiseVariables(optimization::NLPProblem& nlp,
                             const StateType& x0,
                             Scalar s_init) const;
};

}  // namespace kinetra::planners

// Template implementation
#include "kinetra/planners/mpcc_impl.hpp"
