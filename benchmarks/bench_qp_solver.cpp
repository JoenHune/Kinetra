// SPDX-License-Identifier: BSD-3-Clause
#include <benchmark/benchmark.h>
#include "kinetra/solvers/qp_admm.hpp"

using namespace kinetra;
using namespace kinetra::solvers;

static void BM_ADMM_QP_Small(benchmark::State& state) {
    int n = static_cast<int>(state.range(0));
    MatX Q = MatX::Identity(n, n) * 2;
    VecX c = VecX::Random(n);
    MatX A = MatX::Identity(n, n);
    VecX lb = VecX::Constant(n, -5.0);
    VecX ub = VecX::Constant(n, 5.0);

    for (auto _ : state) {
        QPSolverADMM solver;
        solver.setup(Q, c, A, lb, ub);
        auto result = solver.solve();
        benchmark::DoNotOptimize(result);
    }
    state.SetComplexityN(n);
}
BENCHMARK(BM_ADMM_QP_Small)->RangeMultiplier(2)->Range(4, 128)->Complexity();

static void BM_ADMM_QP_WarmStart(benchmark::State& state) {
    int n = 20;
    MatX Q = MatX::Identity(n, n) * 2;
    VecX c = VecX::Random(n);
    MatX A = MatX::Identity(n, n);
    VecX lb = VecX::Constant(n, -5.0);
    VecX ub = VecX::Constant(n, 5.0);

    QPSolverADMM solver;
    solver.setup(Q, c, A, lb, ub);
    auto first_result = solver.solve();

    for (auto _ : state) {
        // Slightly perturb cost and re-solve with warm start
        c = VecX::Random(n) * 0.01;  // Small perturbation
        solver.updateLinearCost(c);
        solver.warmStart(first_result.x, first_result.y);
        auto result = solver.solve();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_ADMM_QP_WarmStart);
