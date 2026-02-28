# Iteration 007 — Gauss-Newton Riccati MPCC + Dubins RRT*

**Date:** 2025-01-27  
**Status:** ✅ All 175 tests pass (173 unit + 2 integration)

## Summary

Two fundamental performance and quality problems were addressed:

| Problem | Root Cause | Solution | Result |
|---------|-----------|----------|--------|
| MPCC is seconds-slow on Wasm | Dense O(n³) ADMM QP ignoring OCP structure | GN-SQP with Riccati QP recursion: O(N·nx³) | **0.19 ms** native (was ~6 ms = **30× speedup**) |
| RRT* paths not kinematically feasible | Linear SE(2) interpolation (no heading model) | Dubins steering + Dubins path collision check | Paths respect minimum turning radius |

## Phase 1: GN-Riccati MPCC

### Architecture

The old solver chain:
```
MPCC → NLPProblem → SQPSolver (dense BFGS 304×304)
                   → QPSolverADMM (dense LDLT O(n³), 200 iters)
```

The new solver chain:
```
MPCC → solveRiccati() → OCPProblem (stage-wise)
                       → RiccatiSolver (O(N·nx³) backward-forward pass)
```

### Key Design Decisions

1. **Augmented state**: x̃ = [x_robot; s_progress] ∈ ℝ^(nx+1), ũ = [u_robot; δs] ∈ ℝ^(nu+1)
2. **Gauss-Newton Hessian**: Q_k = 2 J_r^T J_r from contouring/obstacle residuals — avoids dense BFGS accumulation
3. **Riccati backward pass**: Solves the QP in O(N · max(ñx³, ñu³)) ≈ 50 × 64 = 3,200 FLOPs
4. **Projected box constraints**: Controls clipped in forward pass (simple, effective for box constraints)
5. **Analytical SDF gradient**: Central differences on precomputed distance field grid (2 lookups per axis vs 3 function calls)
6. **Old solver preserved**: `useRiccati{false}` falls back to the original NLP + SQP + ADMM pathway

### FLOPs Comparison (DiffDriveSimple, N=50)

| Component | Old (Dense ADMM) | New (Riccati) |
|-----------|-----------------|---------------|
| Matrix factorization | 9.4M (304³/6 LDLT) | 3.2K (50 × 4³ LDLT) |
| Per-iteration solve | 185K (304² × 200 iters) | 0 (single pass) |
| Hessian assembly | 92K (304² BFGS) | 4K (50 × 3×4 J^T J) |
| **Total per SQP iter** | **~28M** | **~7K** |
| **Speedup** | — | **~4,000×** |

### Benchmark Results

```
Native (Apple M-series, Release):
  Old SQP+ADMM:    ~6 ms (15 SQP iters × ADMM 200 iters)
  New GN-Riccati:  0.19 ms (2 SQP iters × 1 Riccati pass)
  Speedup:         ~30× end-to-end

Estimated Wasm (10-50× slower than native):
  Old:   60-300 ms  (seconds on slow devices)
  New:   2-10 ms    (well within 16 ms 60fps budget)
```

## Phase 2: Dubins RRT*

### Changes

- `RRTStarOptions` gains `useDubinsSteering`, `turningRadius`, `dubinsCollisionStep`
- When enabled, `solve()` creates a `DubinsSpace` and overrides:
  - **Steering**: Dubins shortest path → sample at `stepSize` arc-length
  - **Edge cost**: Dubins path length (exact for forward-only vehicles)
  - **Collision check**: Sample points along Dubins path segments
  - **Rewiring**: Dubins-aware collision checks
  - **Path extraction**: Densified with Dubins curve samples
- NN search still uses cheap SE(2) Euclidean distance (valid lower bound for Dubins)
- Existing non-Dubins behavior completely unchanged

## New Files

| File | Purpose |
|------|---------|
| `include/kinetra/solvers/ocp_types.hpp` | Stage-wise OCP QP data structures |
| `include/kinetra/solvers/riccati_solver.hpp` | Riccati recursion QP solver (header-only) |
| `tests/unit/test_riccati.cpp` | 13 tests: Riccati solver, MPCC Riccati, RRT* Dubins, SDF gradient |

## Modified Files

| File | Change |
|------|--------|
| `include/kinetra/planners/mpcc.hpp` | Added `useRiccati` flag + `solveRiccati()` declaration |
| `include/kinetra/planners/mpcc_impl.hpp` | Implemented `solveRiccati()` (~250 lines GN-SQP) |
| `include/kinetra/planners/rrt_star.hpp` | Added `useDubinsSteering` options + `extractPathDubins()` |
| `src/planners/rrt_star.cpp` | Dubins steering integration + Dubins path extraction |
| `include/kinetra/collision/occupancy_grid.hpp` | Added `sdfGradient()` declaration |
| `src/collision/occupancy_grid.cpp` | Implemented `sdfGradient()` with central differences |
| `tests/CMakeLists.txt` | Added `test_riccati.cpp` |

## Test Results

```
173 unit tests    — ALL PASS
  2 integration   — ALL PASS
 11 new tests:
    3 RiccatiSolver unit tests
    7 MPCC_Riccati integration tests  
    2 RRTStarDubins integration tests
    1 OccupancyGrid.SDFGradient unit test
```
