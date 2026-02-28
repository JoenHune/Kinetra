# Iteration 006 — 2D Trajectory Optimization Survey & High-Performance Architecture Design

> **Date**: 2026-02-28
> **Scope**: Deep survey of academic and open-source work on 2D trajectory optimization;
> diagnosis of Kinetra's performance bottlenecks; architecture blueprint for high-performance
> trajectory optimization suitable for real-time (native) and interactive (Wasm) operation.

---

## Part I — Survey of 2D Trajectory Optimization

### 1. Problem Taxonomy

2D trajectory optimization for mobile robots can be decomposed along two axes:

| Axis | Categories |
|------|-----------|
| **Kinematic model** | Holonomic (Omni), Non-holonomic (DiffDrive, Ackermann/Bicycle), Dynamic (tire forces) |
| **Problem structure** | Point-to-point planning, Path tracking (NMPC/MPCC), Minimum-time racing, Corridor planning |

The fundamental mathematical form is an **Optimal Control Problem (OCP)**:

$$\min_{x_{0:N}, u_{0:N-1}} \sum_{k=0}^{N} \ell_k(x_k, u_k) \quad \text{s.t.} \quad x_{k+1} = f(x_k, u_k), \quad g(x_k, u_k) \le 0$$

where $x_k \in \mathbb{R}^{n_x}$ is state, $u_k \in \mathbb{R}^{n_u}$ is control, $f$ is discrete dynamics, and $g$
encodes collision avoidance / bounds.

---

### 2. Academic Literature — Key Contributions

#### 2.1 Model Predictive Contouring Control (MPCC)

**Liniger, Domahidi & Morari (2015)** — *"Optimization-based Autonomous Racing of 1:43 Scale RC Cars"*
([arXiv:1711.07300](https://arxiv.org/abs/1711.07300))

The foundational MPCC paper. Key ideas:
- Augment system state with **progress variable** $\theta$ along a reference path
- Decompose tracking error into **contouring error** $e_c$ (lateral) and **lag error** $e_l$ (longitudinal)
- **Maximize progress** $-\lambda \cdot \theta_N$ instead of tracking a time-parameterized reference
- Linearize the NLP around current solution → **time-varying QP** solved at each sample
- Use **HPIPM** (structure-exploiting QP solver) to solve the OCP-structured QP in sub-millisecond
- Real-time at **50 Hz** on embedded hardware (1:43 scale RC cars, >3 m/s, drifting regime)

**Critical insight**: The NLP is not solved generically — it is **linearized into a QP with OCP structure**
$(Q_k, R_k, A_k, B_k)$ and solved by a Riccati recursion, which is $O(N \cdot n_x^3)$ instead of $O((N \cdot n_x)^3)$.

**Vázquez, Brühlmeier, Liniger et al. (2020)** — *"Optimization-Based Hierarchical Motion Planning for Autonomous Racing"*
([arXiv:2003.04882](https://arxiv.org/abs/2003.04882))

Two-level architecture:
1. **High-level**: offline minimum-lap-time trajectory (large horizon, CasADi + Ipopt)
2. **Low-level**: real-time MPCC tracks the high-level trajectory with short horizon + terminal constraint

The terminal constraint (computed offline) allows the MPCC horizon to be shortened from N=50 to N=20
while maintaining safety — **2.5× faster** per solve. Won two international Formula Student competitions.

#### 2.2 Sequential Quadratic Programming (SQP) for OCP

**Standard SQP** (Boggs & Tolle, 1995):
- Full-step Newton on the KKT conditions of the NLP
- Requires Hessian of the Lagrangian (expensive, O(n²) storage)
- Quadratic convergence near the solution

**Real-Time Iteration (RTI)** (Diehl, Bock & Schlöder, 2002):
- Execute **exactly one SQP iteration** per control timestep
- Preparation phase: linearize + factorize (between timesteps)
- Feedback phase: incorporate new state measurement and solve QP (at control trigger)  
- Achieves **microsecond-level** feedback delay
- Relies on the **contraction property** of the Newton iteration near the solution manifold

**GN-SQP / Gauss-Newton SQP** (Bock & Plitt, 1984):
- For least-squares objectives $\ell(x) = \|r(x)\|^2$, approximate the Hessian as $H \approx J_r^T J_r$
- No second-order terms → **always positive semi-definite** (no need for regularization)
- Superlinear convergence for zero-residual problems
- **Preserves OCP structure**: $H$ is block-diagonal if residuals are stage-wise

#### 2.3 Iterative Linear Quadratic Regulator (iLQR / iLQG)

**Li & Todorov (2004)** — *"Iterative Linear-Quadratic Regulator Design for Nonlinear Biological Movement Systems"*

- Leverage the **Riccati recursion** to solve the LQ subproblem in $O(N \cdot n_x^3)$
- Backward pass: compute feedback gains $K_k$ and feedforward $k_k$ via Riccati
- Forward pass: simulate dynamics with $u_k = \bar{u}_k + K_k(x_k - \bar{x}_k) + \alpha \cdot k_k$
- Line search on $\alpha$ for globalization
- **Cannot handle inequality constraints** natively (only via augmented Lagrangian or penalty)

**ALTRO** (Howell, Jackson & Manchester, 2019):
- Augmented Lagrangian + iLQR for constrained trajectory optimization
- AL outer loop handles constraints; iLQR inner loop exploits Riccati
- Final projection step for constraint satisfaction

#### 2.4 Differential Dynamic Programming (DDP)

**Mayne (1966)**, modernized by **Tassa, Erez & Todorov (2012)**:
- Similar to iLQR but includes **second-order dynamics terms** ($f_{xx}$, $f_{xu}$, $f_{uu}$ tensors)
- Requires computing the tensor $\nabla^2_{xx} f$ — expensive for complex models
- iLQR = DDP with second-order dynamics terms dropped (Gauss-Newton approximation)
- In practice, iLQR and DDP have very similar convergence for smooth dynamics

#### 2.5 MPPI — Model Predictive Path Integral Control

**Williams, Aldrich & Theodorou (2017)** — *"Model Predictive Path Integral Control using Covariance Variable Importance Sampling"*

- **Sampling-based** trajectory optimizer — no gradients needed
- Sample K trajectories from a noise distribution, weight by cost, compute weighted mean
- Naturally handles discontinuous costs (collisions = infinite cost → zero weight)
- Parallelizable on GPU (CUDA)
- Used in **AutoRally** platform for aggressive off-road driving
([arXiv:1707.04540](https://arxiv.org/abs/1707.04540): *"Autonomous Racing with AutoRally Vehicles and Differential Games"*)
- Limitation: poor sample efficiency in high-dimensional spaces, requires many (1000+) rollouts

#### 2.6 RRT with Kinematic Steering

**LaValle & Kuffner (2001)** — RRT / RRT*:
- Standard RRT* uses **straight-line steering** — produces kinematically infeasible paths
- For non-holonomic robots, must use **Dubins paths** (forward-only, car-like) or **Reeds-Shepp paths**
  (forward+reverse) as the steering function
- **Dubins-RRT*** achieves asymptotic optimality with kinematically feasible paths
- Computational cost: Dubins path computation is $O(1)$ per query (closed-form for all 6 path types)
- **Hybrid A\*** (Dolgov et al., 2010): lattice expansion with Dubins/RS motions, used in Apollo/Autoware

#### 2.7 Corridor-Based Planning

**Corridor generation** (Liu et al., 2017) — *"Planning Dynamically Feasible Trajectories for Quadrotors using Safe Flight Corridors"*:
- Decompose free space into convex polyhedra (corridors)
- Formulate trajectory optimization as QP with linear corridor constraints
- **Guaranteed collision-free** within the corridor
- For 2D ground robots: safe corridors reduce obstacle avoidance to linear half-space constraints

#### 2.8 Optimization Through Contact

**Posa, Cantu & Tedrake (2014)** — Contact-implicit trajectory optimization:
- Include contact forces as decision variables
- Complementarity constraints model contact/no-contact switching
- Not directly applicable to 2D mobile robots but important for manipulation

---

### 3. Open-Source Implementations

| Project | Language | Solver | QP Backend | Structure Exploit | Speed |
|---------|----------|--------|------------|-------------------|-------|
| **[acados](https://github.com/acados/acados)** (1.2k★) | C + Python/MATLAB/Simulink | SQP, RTI | HPIPM | Full OCP structure | **~100μs** for N=40 |
| **[HPIPM](https://github.com/giaf/hpipm)** (667★) | C (BLASFEO) | Interior Point | N/A (is QP solver) | Riccati/band structure | **~50μs** for dense QP |
| **[OSQP](https://github.com/osqp/osqp)** (2.1k★) | C | ADMM | QDLDL sparse | General sparse | ~1ms for MPC-size |
| **[Liniger's MPCC](https://github.com/alexliniger/MPCC)** (1.8k★) | C++ / MATLAB | Lin.→QP | HPIPM/Yalmip | OCP structure via HPIPM | ~2ms per step |
| **[Control Toolbox](https://github.com/ethz-adrl/control-toolbox)** (1.7k★) | C++ | iLQR, GNMS, DMS | HPIPM/custom Riccati | Full OCP structure | Sub-ms iLQR |
| **[Ipopt](https://github.com/coin-or/Ipopt)** (1.7k★) | C++ | Interior Point | MA27/MA57/MUMPS | General sparse | ~10ms (large NLP) |
| **[CasADi](https://web.casadi.org/)** | C++ / Python / MATLAB | Wrapper for Ipopt/SNOPT/.. | Via backend | AD + code generation | Varies |
| **[Crocoddyl](https://github.com/loco-3d/crocoddyl)** | C++ / Python | DDP/FDDP | Custom Riccati | Full OCP structure | ~1ms |
| **[OCS2](https://github.com/leggedrobotics/ocs2)** | C++ / ROS | SLQ (iLQR variant) | Custom | Multi-thread Riccati | Real-time locomotion |

**Key takeaway**: Every high-performance implementation exploits **OCP structure** via Riccati recursion
or banded linear algebra. None uses dense unstructured BFGS + dense ADMM for the QP subproblem.

---

### 4. QP Solver Comparison for OCP

| Solver | Method | Structure | Sparsity | Factorization | Warm-start | Embedded |
|--------|--------|-----------|----------|---------------|------------|----------|
| **HPIPM** | IPM | OCP-aware | Riccati | $O(N n_x^3)$ | Partial | Yes (C) |
| **OSQP** | ADMM | General sparse | Sparse LDL | $O(\text{nnz}^{1.5})$ | Full | Yes (C) |
| **qpOASES** | Active-set | Dense | Dense QR/Chol | $O(n^3)$ | Hot-start | Yes (C) |
| **Kinetra ADMM** | ADMM | **None** | **Dense** | **$O(n^3)$** | Partial | Wasm |

Kinetra's QP solver is essentially a simplified OSQP reimplementation **without sparse factorization**.
The performance gap vs. a properly structured solver is approximately:

$$\frac{O(n^3)}{O(N \cdot n_x^3)} = \frac{O(304^3)}{O(50 \cdot 3^3)} \approx \frac{28M}{1.4K} \approx 20{,}000\times$$

This explains why acados solves in 100μs while Kinetra takes seconds.

---

## Part II — Kinetra Performance Diagnosis

### 5. Current Architecture & Bottlenecks

```
MPCC::solve()
 └─ Build NLPProblem
     ├─ DynamicsConstraint (150 eq)     ← analytical Jacobians, fast
     ├─ InitialStateConstraint (3 eq)   ← trivial
     ├─ ProgressConstraint (50 box)     ← trivial
     ├─ ContouringCost                  ← analytical gradient
     ├─ ControlCost                     ← analytical gradient
     └─ ObstacleCost                    ← NUMERICAL gradient (finite diff on SDF)
 └─ SQPSolver::solve()
     └─ For each SQP iteration (15–30):
         1. Evaluate cost, gradient, constraints, Jacobian
         2. BFGS update of H (304×304 dense)         ← O(n²)
         3. Form QP: Q=H, c=∇f, A=[J;I], l,u
         4. QPSolverADMM::setup()
            ├─ AᵀA = AT * A (dense 304×304)          ← O(n² · nnz/n)
            └─ LDLT(Q + σI + ρ·AᵀA) (dense 304×304)  ← O(n³) = 28M FLOPs ⬅ BOTTLENECK
         5. QPSolverADMM::solve() (200–400 iters)
            └─ Per iter: ldlt.solve(rhs) (dense)      ← O(n²) × 200 = 18M FLOPs
         6. Line search (merit function)
         7. Accept or reject step
```

### 6. Quantitative Breakdown (n=304, Wasm)

| Operation | FLOPs | Native (μs) | Wasm (μs) | Notes |
|-----------|-------|-------------|-----------|-------|
| Jacobian eval | ~5K | 5 | 30 | Analytical, fast |
| Cost + gradient | ~15K | 10 | 60 | + SDF finite diff |
| BFGS rank-2 update | ~185K | 50 | 300 | Dense H update |
| **AᵀA dense** | **~300K** | **100** | **600** | Destroys sparsity |
| **LDLT factor (304³/3)** | **~9.4M** | **1,000** | **10,000** | **DOMINANT** |
| **200 × LDLT solve** | **~18.5M** | **5,000** | **50,000** | **DOMINANT** |
| Line search (3 evals) | ~45K | 30 | 200 | |
| **Total per SQP iter** | **~28M** | **~6 ms** | **~60 ms** | |
| **Full solve (25 iters)** | **~700M** | **~150 ms** | **~1,500 ms** | Matches observation |

On Wasm: **1.5–5 seconds** for a full MPCC solve. This matches the user-reported "seconds per iteration."

### 7. RRT* Heading Problem

Current RRT* uses `SE2Space::interpolate()` which linearly interpolates $(x, y, \theta)$:
- Position: LERP
- Heading: angular SLERP (shortest path)

This produces paths where consecutive waypoints may have headings that **no physical robot can achieve**.
For an Ackermann robot with wheelbase $L$ and steering limit $\phi_{\max}$, the minimum turning radius is:

$$R_{\min} = \frac{L}{\tan \phi_{\max}}$$

The linear steering function completely ignores this constraint. The `DubinsSpace` implementation exists
in the codebase but is **not connected** to RRT*.

---

## Part III — High-Performance Architecture Design

### 8. Proposed Architecture: Gauss-Newton SQP with Riccati QP Solve

The key insight from the survey: **all fast trajectory optimizers exploit the temporal (OCP) structure**.
The optimal architecture for Kinetra is:

```
MPCC::solve()
 └─ For each GN-SQP iteration:
     1. Forward rollout: x(k+1) = f(x(k), u(k))        ← O(N · n_x)
     2. Linearize dynamics: A_k, B_k at each timestep   ← O(N · n_x²) [analytical]
     3. Quadratize cost:   Q_k, R_k, q_k, r_k          ← O(N · n_x²) [GN: J_r^T J_r]
     4. Solve structured QP via Riccati recursion:       ← O(N · n_x³)
        Backward:  P_k, K_k, k_k  ← Riccati
        Forward:   δx_k, δu_k     ← feedforward + feedback
     5. Line search + step acceptance
```

**Complexity comparison**:

| | Current (dense BFGS + ADMM) | Proposed (GN + Riccati) | Speedup |
|---|---|---|---|
| Hessian | Dense BFGS $O(n^2)$ | Stage-wise GN $O(N \cdot n_x^2)$ | $\frac{n^2}{N \cdot n_x^2} \approx 68\times$ |
| QP solve | Dense LDLT $O(n^3)$ + 200×$O(n^2)$ | Riccati $O(N \cdot n_x^3)$ | $\frac{n^3}{N \cdot n_x^3} \approx 20{,}000\times$ |
| Memory | $O(n^2) = 92\text{K}$ entries | $O(N \cdot n_x^2) = 450$ entries | $200\times$ |
| Total per iter | ~28M FLOPs | ~5K FLOPs | **~5{,}000×** |

### 9. Detailed Design

#### 9.1 Stage-wise Problem Formulation

Instead of assembling a single flat NLP, define the problem **per-stage**:

```cpp
struct Stage {
    VecX x;          // state at this stage (n_x)
    VecX u;          // control at this stage (n_u)
    MatX A, B;       // linearized dynamics: x(k+1) ≈ A·x(k) + B·u(k) + c
    VecX c;          // dynamics affine term
    MatX Q, R, S;    // cost Hessian blocks (GN approx)
    VecX q, r;       // cost gradient blocks
    VecX x_lb, x_ub; // state bounds
    VecX u_lb, u_ub; // control bounds
};
```

#### 9.2 Gauss-Newton Hessian for MPCC

The MPCC cost is a sum of squared residuals:

$$\ell_k = w_c e_c^2 + w_l e_l^2 + w_\theta e_\theta^2 + w_u \|u_k\|^2$$

Define residual vector:

$$r_k(x_k, s_k) = \begin{bmatrix} \sqrt{w_c} \cdot e_c \\ \sqrt{w_l} \cdot e_l \\ \sqrt{w_\theta} \cdot e_\theta \end{bmatrix} \in \mathbb{R}^3$$

The Gauss-Newton Hessian at stage $k$ is:

$$H_k \approx J_{r_k}^T J_{r_k} \in \mathbb{R}^{(n_x + 1) \times (n_x + 1)}$$

where $J_{r_k} = \partial r_k / \partial (x_k, s_k)$ is the $3 \times (n_x + 1)$ residual Jacobian.
This is a **tiny** $(n_x+1) \times (n_x+1)$ matrix per stage, vs. the current $n \times n$ dense BFGS.

#### 9.3 Riccati-Based QP Solver

The unconstrained OCP-QP has a closed-form solution via the **Riccati recursion**:

**Backward pass** ($k = N \to 0$):
$$Q_{xx} = Q_k + A_k^T P_{k+1} A_k$$
$$Q_{xu} = S_k + A_k^T P_{k+1} B_k$$
$$Q_{uu} = R_k + B_k^T P_{k+1} B_k$$
$$K_k = -Q_{uu}^{-1} Q_{xu}^T$$
$$k_k = -Q_{uu}^{-1} (r_k + B_k^T p_{k+1})$$
$$P_k = Q_{xx} + Q_{xu} K_k$$
$$p_k = q_k + A_k^T p_{k+1} + Q_{xu} k_k$$

**Forward pass** ($k = 0 \to N$):
$$\delta u_k = K_k \delta x_k + \alpha \cdot k_k$$
$$\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k + c_k$$

Cost per iteration: **each step** inverts $Q_{uu} \in \mathbb{R}^{n_u \times n_u}$ (2×2 for DiffDrive → trivial)
and does matrix multiplications in $\mathbb{R}^{n_x \times n_x}$ (3×3 for DiffDrive → 27 FLOPs).

Total: $N \times O(n_x^3 + n_x^2 n_u + n_u^3)$ = $50 \times O(27 + 18 + 8)$ ≈ **2,650 FLOPs**.

For comparison, the current dense LDLT alone is **9,400,000 FLOPs**.

#### 9.4 Handling Box Constraints

Pure Riccati handles only equality-constrained QP. For box constraints on states/controls, two approaches:

**Option A — Projected Riccati** (clamp at each forward step):
```
δu_k = clamp(K_k · δx_k + α · k_k, u_lb - ū_k, u_ub - ū_k)
```
Simple but doesn't guarantee optimality. Works well in practice for MPC/MPCC where bounds are
rarely active simultaneously across many stages.

**Option B — AL-iLQR (Augmented Lagrangian)**:
- Add penalty terms for bound violations to the cost
- Solve unconstrained Riccati
- Update Lagrange multipliers in outer loop
- Converges to the constrained optimum
- Used in ALTRO (Howell et al., 2019)

**Option C — HPIPM integration**:
- Use HPIPM as the QP backend for exact constrained OCP-QP
- HPIPM uses interior-point method with Riccati factorization
- Handles all box/polytopic constraints natively
- Complexity: $O(N \cdot n_x^3)$ per IPM iteration, ~5-10 IPM iters
- **Not suitable for Wasm** due to BLASFEO assembly dependency

**Recommended**: Option A (projected Riccati) for Kinetra, since:
- Zero external dependencies (keeps the header-only, Wasm-compatible design)
- Handles state/control bounds (the main constraints in MPCC)
- Obstacle avoidance remains as soft penalty (current design works)
- Complexity: $O(N \cdot n_x^3)$ per GN iteration, same as unconstrained

#### 9.5 Obstacle Avoidance

Current: soft penalty $w_\text{obs} \cdot \max(0, d_\text{safe} - \text{SDF})^2$ with **numerical** gradient.

Proposed improvements:
1. **Analytical SDF gradient**: precompute gradient grid $\nabla \text{SDF}(x, y)$ alongside the SDF.
   The EDT (exact distance transform) naturally provides the gradient direction as the Voronoi vector.
2. **Include obstacle cost in GN residual**: $r_\text{obs} = \sqrt{w_\text{obs}} \cdot \max(0, d_\text{safe} - \text{SDF})$,
   which gives a natural GN Hessian contribution.

#### 9.6 Progress Variable Treatment

The progress variable $s_k$ is currently a decision variable alongside states/controls.
In the stage-wise formulation, augment the state:

$$\tilde{x}_k = \begin{bmatrix} x_k \\ s_k \end{bmatrix} \in \mathbb{R}^{n_x + 1}$$

With augmented dynamics:
$$\tilde{f}(\tilde{x}_k, u_k, v_k) = \begin{bmatrix} f(x_k, u_k) \\ s_k + v_k \cdot dt \end{bmatrix}$$

where $v_k$ is the progress velocity (augmented control). This keeps the OCP structure intact
and the Riccati recursion applies directly on the augmented system.

### 10. RRT* with Dubins Steering

#### 10.1 Integration Plan

Replace `SE2Space::steer()` with `DubinsSpace::steer()` in RRT*:

```cpp
// Current (infeasible heading):
SE2State steer(from, to, stepSize) {
    return SE2Space::interpolate(from, to, stepSize / distance(from, to));
}

// Proposed (kinematically feasible):
SE2State steer(from, to, stepSize) {
    auto dubinsPath = DubinsSpace::shortestPath(from, to, turningRadius);
    return DubinsSpace::sample(dubinsPath, stepSize);
}
```

The Dubins distance replaces the weighted SE(2) distance for nearest-neighbor queries.

#### 10.2 Additional Improvements

1. **k-d tree for NN queries**: current $O(n)$ → $O(\log n)$ per query.
   For SE(2), use a modified k-d tree that wraps the angle dimension.
   Alternative: spatial grid hash for $(x, y)$ + angle bucketing.

2. **Robot footprint collision checking**: instead of point-robot,
   check the rectangular footprint swept along Dubins arcs.

3. **Subtree cost propagation**: after rewiring, BFS-propagate updated costs
   through all descendants.

### 11. Implementation Roadmap

```
Phase 1: GN-Riccati SQP core                    [High impact, ~3 days]
├── Stage-wise OCP data structure
├── GN Hessian assembly (stage-wise J_r^T J_r)
├── Riccati backward/forward pass (unconstrained)
├── Projected Riccati for box constraints
├── Line search with L1 merit function
└── MPCC adapter: build stages from MPCC formulation

Phase 2: MPCC performance polish                 [Medium impact, ~1 day]
├── Analytical SDF gradient (precomputed gradient grid)
├── Warm-starting from previous MPCC solution
├── Convergence: GN typically converges in 5-10 iters (vs. 15-30 for BFGS)
└── Wasm rebuild + benchmark on page

Phase 3: RRT* kinematic feasibility              [Medium impact, ~2 days]
├── Dubins steering function integration
├── Dubins distance metric for NN
├── k-d tree (or spatial hash) for nearest neighbor
└── Robot footprint swept-volume collision checking

Phase 4: Pipeline integration + polish           [Low-medium impact, ~1 day]
├── RRT* (Dubins) → smooth → MPCC pipeline
├── New scenarios demonstrating drivable paths
└── Page redesign with performance stats
```

### 12. Expected Performance After Optimization

| Metric | Current | Phase 1 (GN-Riccati) | Phase 1+2 (+ polish) |
|--------|---------|---------------------|----------------------|
| FLOPs per SQP iter (N=50, nx=3) | ~28M | ~5K | ~5K |
| SQP iterations to converge | 15–30 | 5–10 | 5–8 |
| Native solve time | ~150ms | **~0.2ms** | **~0.1ms** |
| Wasm solve time | **~1.5s** | **~5ms** | **~3ms** |
| Memory (QP solver) | 340 KB (dense) | 1.8 KB (staged) | 1.8 KB |

The GN-Riccati approach should deliver a **~300–500× speedup**, making interactive Wasm operation
feasible (<16ms = 60fps budget).

---

## Part IV — Summary

### What We Learned

1. **Every successful real-time trajectory optimizer exploits OCP structure** — Riccati recursion
   reduces $O(n^3)$ to $O(N \cdot n_x^3)$, which for typical mobile robot problems (nx=3-7, N=20-80)
   is the difference between milliseconds and microseconds.

2. **BFGS is the wrong Hessian strategy for trajectory optimization** — it produces dense, unstructured
   matrices that destroy the temporal decoupling. Gauss-Newton preserves block-diagonal structure
   and is naturally positive-definite for sum-of-squares costs.

3. **ADMM is the wrong QP method for structured OCPs** — first-order methods converge slowly and
   don't exploit the Riccati-solvable structure. An IPM with Riccati factorization (HPIPM) or
   direct Riccati solve (for box-constrained) is orders of magnitude faster.

4. **Liniger's MPCC** (the reference implementation) uses exactly this architecture: linearize → QP
   with OCP structure → HPIPM solver. Kinetra's current approach is a correct but **generic NLP solver**
   applied to a problem with exploitable structure.

5. **RRT\* with Dubins steering** is a well-established technique. The infrastructure (`DubinsSpace`)
   already exists in Kinetra — it just needs to be connected.

### References

| # | Paper | Key Contribution |
|---|-------|-----------------|
| 1 | Liniger et al. (2015) — Optimization-based Autonomous Racing | MPCC formulation + HPIPM-solved QP |
| 2 | Vázquez et al. (2020) — Hierarchical Motion Planning for Racing | MPCC + terminal constraint for short horizons |
| 3 | Frison & Diehl (2020) — HPIPM | Structure-exploiting IPM for OCP-QPs |
| 4 | Verschueren et al. (2020) — acados | Modular OCP solver framework (RTI, SQP, GN) |
| 5 | Giftthaler et al. (2018) — Control Toolbox | C++ iLQR/GNMS with HPIPM backend |
| 6 | Diehl, Bock & Schlöder (2002) — Real-Time Iteration | Single-iteration SQP for MPC |
| 7 | Li & Todorov (2004) — iLQR | Riccati-based trajectory optimization |
| 8 | Howell et al. (2019) — ALTRO | AL + iLQR for constrained trajectory optimization |
| 9 | Williams et al. (2017) — MPPI | Sampling-based MPC for aggressive driving |
| 10 | Stellato et al. (2020) — OSQP | ADMM-based QP solver |
| 11 | LaValle & Kuffner (2001) — RRT | Sampling-based motion planning |
| 12 | Dolgov et al. (2010) — Hybrid A* | Lattice search with analytic expansions |
