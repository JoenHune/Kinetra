// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "kinetra/planners/mpcc.hpp"
#include "kinetra/robots/differential_drive.hpp"
#include "kinetra/robots/ackermann.hpp"
#include "kinetra/robots/omnidirectional.hpp"
#include "kinetra/collision/occupancy_grid.hpp"
#include "kinetra/core/reference_path.hpp"
#include "kinetra/core/result.hpp"

using namespace kinetra;
using namespace kinetra::planners;

// ═════════════════════════════════════════════════════════════════════════════
// Helper: build a straight-line reference path
// ═════════════════════════════════════════════════════════════════════════════
namespace {

ReferencePath makeStraightPath(Scalar length = 5.0, int nPts = 20) {
    std::vector<Waypoint2D> wps;
    for (int i = 0; i < nPts; ++i) {
        Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(nPts - 1);
        wps.push_back({t * length, 0, 0, t});
    }
    return ReferencePath(wps);
}

ReferencePath makeCurvedPath() {
    // Quarter circle: (0,0) → (1,1) with heading 0 → π/2
    std::vector<Waypoint2D> wps;
    int N = 20;
    for (int i = 0; i <= N; ++i) {
        Scalar t = static_cast<Scalar>(i) / static_cast<Scalar>(N);
        Scalar angle = t * M_PI / 2;
        wps.push_back({
            std::sin(angle),
            1.0 - std::cos(angle),
            angle,
            t
        });
    }
    return ReferencePath(wps);
}

PlanningProblem makeSimpleProblem() {
    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {5, 0, 0, 0};
    problem.environment.bounds_min = {-1, -3};
    problem.environment.bounds_max = {7, 3};
    return problem;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// MPCCOptions
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCCOptions, Defaults) {
    MPCCOptions opts;
    EXPECT_EQ(opts.horizon, 50);
    EXPECT_NEAR(opts.dt, 0.05, 1e-10);
    EXPECT_NEAR(opts.wContour, 50.0, 1e-10);
    EXPECT_NEAR(opts.wLag, 20.0, 1e-10);
    EXPECT_NEAR(opts.wHeading, 30.0, 1e-10);
    EXPECT_NEAR(opts.wProgress, 5.0, 1e-10);
    EXPECT_NEAR(opts.wControl, 1.0, 1e-10);
    EXPECT_EQ(opts.sqpSettings.maxIterations, 30);
}

// ═════════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_DiffDrive, ConstructionSimple) {
    DiffDriveSimple model;
    auto path = makeStraightPath();
    MPCC<DiffDriveSimple> mpcc(model, path);
    EXPECT_EQ(mpcc.name(), "MPCC");
    EXPECT_EQ(mpcc.referencePath().waypointCount(), 20u);
}

TEST(MPCC_DiffDrive, ConstructionWithOptions) {
    DiffDriveSimple model;
    auto path = makeStraightPath();
    MPCCOptions opts;
    opts.horizon = 30;
    opts.dt = 0.1;
    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    EXPECT_EQ(mpcc.options().horizon, 30);
    EXPECT_NEAR(mpcc.options().dt, 0.1, 1e-10);
}

TEST(MPCC_DiffDrive, OptionsModifiable) {
    DiffDriveSimple model;
    auto path = makeStraightPath();
    MPCC<DiffDriveSimple> mpcc(model, path);
    mpcc.options().horizon = 25;
    EXPECT_EQ(mpcc.options().horizon, 25);
}

// ═════════════════════════════════════════════════════════════════════════════
// Solving — DiffDriveSimple (3D state, 2D control)
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_DiffDrive, SolveStraightLine) {
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 15;
    opts.sqpSettings.verbose = false;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();
    problem.goal = {3, 0, 0, 0};

    auto result = mpcc.solve(problem);

    // Should produce a valid trajectory
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
    EXPECT_EQ(result.planningResult.plannerName, "MPCC");

    // Progress should be monotonically non-decreasing
    for (std::size_t i = 1; i < result.progressValues.size(); ++i) {
        EXPECT_GE(result.progressValues[i], result.progressValues[i - 1] - 1e-6);
    }

    // Should have controls
    EXPECT_EQ(result.controls.size(), static_cast<std::size_t>(opts.horizon));

    // Contour/lag/heading errors should have entries for each step
    EXPECT_EQ(result.contourErrors.size(), static_cast<std::size_t>(opts.horizon + 1));
    EXPECT_EQ(result.lagErrors.size(), static_cast<std::size_t>(opts.horizon + 1));
    EXPECT_EQ(result.headingErrors.size(), static_cast<std::size_t>(opts.horizon + 1));
}

TEST(MPCC_DiffDrive, SolveCurvedPath) {
    DiffDriveSimple model;
    auto path = makeCurvedPath();

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 15;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();

    auto result = mpcc.solve(problem);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);

    // Progress should advance
    EXPECT_GT(result.progressValues.back(), result.progressValues.front());
}

// ═════════════════════════════════════════════════════════════════════════════
// Solving — DiffDriveAccel (5D state, 2D control)
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_DiffDriveAccel, SolveStraightLine) {
    DiffDriveAccel model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 10;

    MPCC<DiffDriveAccel> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();

    auto result = mpcc.solve(problem);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
    EXPECT_EQ(result.controls.size(), static_cast<std::size_t>(opts.horizon));
}

// ═════════════════════════════════════════════════════════════════════════════
// Solving — OmniSimple (3D state, 2D control)
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_Omni, SolveStraightLine) {
    OmniSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 10;

    MPCC<OmniSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();

    auto result = mpcc.solve(problem);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// Solving — AckermannSimple
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_Ackermann, SolveStraightLine) {
    AckermannSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 10;

    MPCC<AckermannSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();

    auto result = mpcc.solve(problem);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// Obstacle avoidance
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_DiffDrive, SolveWithObstacles) {
    DiffDriveSimple model;
    auto path = makeStraightPath(4.0, 15);

    MPCCOptions opts;
    opts.horizon = 20;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 15;
    opts.wObstacle = 200.0;
    opts.safeDistance = 0.2;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);

    PlanningProblem problem;
    problem.start = {0, 0, 0, 0};
    problem.goal = {4, 0, 0, 0};
    problem.environment.bounds_min = {-1, -3};
    problem.environment.bounds_max = {5, 3};
    problem.environment.obstacles.push_back(
        CircleObstacle{{2.0, 0.0}, 0.5});

    auto result = mpcc.solve(problem);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

TEST(MPCC_DiffDrive, SolveWithExplicitGrid) {
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 10;

    MPCC<DiffDriveSimple> mpcc(model, path, opts);

    collision::OccupancyGrid2D grid(-1, -3, 5, 3, 0.1);
    grid.addCircleObstacle(Vec2(1.5, 0.0), 0.3);

    auto problem = makeSimpleProblem();
    auto result = mpcc.solve(problem, grid);
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// Weight sensitivity
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_DiffDrive, HighContourWeight) {
    DiffDriveSimple model;
    auto path = makeStraightPath(3.0, 15);

    MPCCOptions opts;
    opts.horizon = 15;
    opts.dt = 0.1;
    opts.sqpSettings.maxIterations = 10;
    opts.wContour = 500.0;      // Very high contour weight
    opts.wProgress = 0.1;       // Low progress reward

    MPCC<DiffDriveSimple> mpcc(model, path, opts);
    auto problem = makeSimpleProblem();
    auto result = mpcc.solve(problem);

    // Should still produce valid trajectory
    EXPECT_GT(result.planningResult.trajectory.size(), 0u);
}

// ═════════════════════════════════════════════════════════════════════════════
// Reset
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_DiffDrive, Reset) {
    DiffDriveSimple model;
    auto path = makeStraightPath();
    MPCC<DiffDriveSimple> mpcc(model, path);
    // Reset should not throw
    EXPECT_NO_THROW(mpcc.reset());
}

// ═════════════════════════════════════════════════════════════════════════════
// NLP Components — Direct unit tests
// ═════════════════════════════════════════════════════════════════════════════

TEST(MPCC_NLP, StateVarsBounds) {
    int N = 10;
    int nx = 3;
    VecX lo(3), hi(3);
    lo << -100, -100, -M_PI;
    hi <<  100,  100,  M_PI;
    planners::mpcc_nlp::MPCCStateVars vars(N, nx, lo, hi);

    EXPECT_EQ(vars.size(), (N + 1) * nx);
    auto lb = vars.lowerBounds();
    auto ub = vars.upperBounds();
    EXPECT_EQ(lb.size(), (N + 1) * nx);
    EXPECT_NEAR(lb[0], -100, 1e-10);
    EXPECT_NEAR(ub[2], M_PI, 1e-6);
}

TEST(MPCC_NLP, ControlVarsBounds) {
    int N = 10;
    int nu = 2;
    VecX lo(2), hi(2);
    lo << -1, -2;
    hi <<  1,  2;
    planners::mpcc_nlp::MPCCControlVars vars(N, nu, lo, hi);

    EXPECT_EQ(vars.size(), N * nu);
}

TEST(MPCC_NLP, ProgressVarsBounds) {
    int N = 10;
    planners::mpcc_nlp::MPCCProgressVars vars(N, 0.0, 5.0);

    EXPECT_EQ(vars.size(), N + 1);
    auto lb = vars.lowerBounds();
    auto ub = vars.upperBounds();
    EXPECT_NEAR(lb[0], 0.0, 1e-10);
    EXPECT_NEAR(ub[N], 5.0, 1e-10);
}

TEST(MPCC_NLP, ProgressConstraintMonotonicity) {
    int N = 5;
    Scalar maxRate = 2.0;
    Scalar dt = 0.1;

    auto progressVars = std::make_shared<planners::mpcc_nlp::MPCCProgressVars>(
        N, 0.0, 3.0);
    // Set monotonically increasing values
    VecX vals(N + 1);
    for (int k = 0; k <= N; ++k)
        vals[k] = 0.1 * k;
    progressVars->setValues(vals);

    planners::mpcc_nlp::ProgressConstraint con(N, maxRate, dt);
    con.linkVariables({progressVars});

    VecX g = con.evaluate();
    EXPECT_EQ(g.size(), N);
    for (int k = 0; k < N; ++k)
        EXPECT_NEAR(g[k], 0.1, 1e-10);  // s_{k+1} - s_k = 0.1

    VecX lb = con.lowerBound();
    VecX ub = con.upperBound();
    for (int k = 0; k < N; ++k) {
        EXPECT_GE(g[k], lb[k] - 1e-10);
        EXPECT_LE(g[k], ub[k] + 1e-10);
    }
}
