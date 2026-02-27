<div align="center">

# 🔀 Kinetra

**World-class trajectory planning for robotics — standalone, embeddable, blazing fast.**

[![CI](https://github.com/JoenHune/Kinetra/actions/workflows/ci.yml/badge.svg)](https://github.com/JoenHune/Kinetra/actions/workflows/ci.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-green.svg)](LICENSE)
[![ARMv7](https://img.shields.io/badge/ARMv7-ready-orange.svg)](#cross-compilation)

*Kinematic + Trajectory = Kinetra*

</div>

---

## Highlights

| Feature | Description |
|---------|-------------|
| **Standalone** | Zero external solver dependencies. Only Eigen (header-only, auto-fetched). |
| **C++20** | Concepts for zero-cost polymorphism, `constexpr` math, `std::span`. |
| **Embeddable** | Deploys to ARMv7 (NEON-optimized). Switchable `float`/`double` scalar. |
| **Multi-Model** | Differential drive, Ackermann, omnidirectional — velocity & accel orders. |
| **Composable NLP** | ifopt-inspired: build problems from VariableSets + Constraints + Costs. |
| **Test-Driven** | 50+ unit tests, integration tests, Google Benchmark suites. |
| **AI Agent Loop** | Automated research → design → implement → test → benchmark → reflect. |
| **Visualization** | GitHub Pages dashboard with Plotly.js (trajectory, benchmarks, agent log). |

## Quick Start

```bash
# Clone
git clone https://github.com/JoenHune/Kinetra.git
cd Kinetra

# Build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

# Test
ctest --test-dir build --output-on-failure

# Run example
./build/examples/example_basic_planning
```

## Architecture

```
kinetra/
├── include/kinetra/
│   ├── core/           # types, concepts, trajectory, result, bounds
│   ├── spaces/         # SE(2), Dubins, R^N state spaces
│   ├── robots/         # DiffDrive, Ackermann, Omni models
│   ├── optimization/   # Composable NLP problem formulation
│   ├── solvers/        # ADMM QP solver, LQR (Riccati)
│   ├── planners/       # RRT*, STOMP, iLQR
│   ├── collision/      # Occupancy grid + signed distance field
│   └── io/             # JSON export, HTML visualization
├── src/                # Implementations
├── tests/              # GoogleTest suite
├── benchmarks/         # Google Benchmark suite
├── examples/           # Working examples
├── agent/              # AI development agent (Python)
├── docs/               # GitHub Pages (Plotly.js dashboard)
└── cmake/toolchains/   # ARMv7 cross-compilation
```

## Robot Models

All models satisfy the `RobotModel` and `LinearizableModel` concepts, providing:
- `dynamics(state, control, dt)` → next state
- `jacobianState(state, control, dt)` → ∂f/∂x
- `jacobianControl(state, control, dt)` → ∂f/∂u

| Model | State | Control | Order |
|-------|-------|---------|-------|
| `DiffDriveSimple` | $(x, y, \theta)$ | $(v, \omega)$ | Velocity |
| `DiffDriveAccel` | $(x, y, \theta, v, \omega)$ | $(a, \alpha)$ | Acceleration |
| `AckermannSimple` | $(x, y, \theta)$ | $(v, \delta)$ | Velocity |
| `AckermannAccel` | $(x, y, \theta, v, \delta)$ | $(a, \dot\delta)$ | Acceleration |
| `OmniSimple` | $(x, y, \theta)$ | $(v_x, v_y, \omega)$ | Velocity |

## Planners

### RRT* (Sampling-based)

Asymptotically optimal rapidly-exploring random tree with lazy rewiring.

```cpp
planners::RRTStarOptions opts;
opts.maxIterations = 5000;
opts.stepSize = 0.5;
planners::RRTStar rrt(opts);
rrt.setCollisionChecker(pointCheck, segmentCheck);
auto result = rrt.solve(problem);
```

### STOMP (Gradient-free optimization)

Stochastic Trajectory Optimization for Motion Planning.

```cpp
planners::STOMPOptions opts;
opts.numTimesteps = 40;
opts.maxIterations = 50;
planners::STOMP stomp(opts);
stomp.setCostFunction(costFn);
auto result = stomp.solve(problem);
```

### iLQR (Optimal control)

Template on any `LinearizableModel` — supports all robot models.

```cpp
planners::iLQR<DiffDriveAccel> ilqr(model, horizon, dt);
auto result = ilqr.solve(x0, x_goal, u_init);
```

## Solvers

### ADMM QP Solver

OSQP-inspired standalone solver: $\min \frac{1}{2} x^T Q x + c^T x$ s.t. $lb \le Ax \le ub$

```cpp
solvers::QPADMMSolver qp;
qp.setup(Q, c, A, lb, ub);
auto result = qp.solve();
```

### LQR (Riccati Recursion)

Finite-horizon LQR used as the backward pass in iLQR.

## Concepts (Zero-Cost Polymorphism)

The library uses C++20 concepts instead of virtual dispatch — crucial for ARMv7 performance:

```cpp
template<typename T>
concept RobotModel = requires(T m) {
    { T::stateDim } -> std::convertible_to<int>;
    { T::controlDim } -> std::convertible_to<int>;
    { m.dynamics(state, control, dt) } -> std::same_as<StateVec>;
};
```

## Cross-Compilation

### ARMv7

```bash
cmake -B build-arm \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/armv7-linux-gnueabihf.cmake \
  -DKINETRA_USE_FLOAT=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-arm --parallel
```

The `KINETRA_USE_FLOAT` flag switches the global `Scalar` type from `double` to `float`, enabling NEON SIMD on ARM Cortex-A cores.

## AI Development Agent

The `agent/` directory contains a Python orchestrator that automates the development loop:

```
Research → Design → Implement → Test → Benchmark → Reflect → repeat
```

```bash
cd agent
pip install -r requirements.txt
python orchestrator.py --iterations 10
```

Each iteration:
1. Searches arXiv & GitHub for relevant algorithms
2. Builds the project and runs tests
3. Runs benchmarks and detects regressions (>5% threshold)
4. Logs learnings to an append-only knowledge base

## Visualization

The `docs/` directory serves as a GitHub Pages site with three views:

- **Trajectory Viewer** — Drop a Kinetra JSON result file to see the path, obstacles, and velocity profile
- **Benchmark Dashboard** — Visualize Google Benchmark JSON output
- **Agent Log** — Track iteration history with phase timing breakdown

## Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `KINETRA_BUILD_TESTS` | `ON` | Build GoogleTest suite |
| `KINETRA_BUILD_BENCHMARKS` | `ON` | Build Google Benchmark suite |
| `KINETRA_BUILD_EXAMPLES` | `ON` | Build examples |
| `KINETRA_USE_FLOAT` | `OFF` | Use `float` instead of `double` |
| `KINETRA_ENABLE_ASAN` | `OFF` | AddressSanitizer |
| `KINETRA_ENABLE_TSAN` | `OFF` | ThreadSanitizer |

## License

BSD 3-Clause. See [LICENSE](LICENSE) for details.

---

<div align="center">
<sub>Built with 🤖 by the Kinetra Development Agent</sub>
</div>
