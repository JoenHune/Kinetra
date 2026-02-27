# ─── Benchmarks ───────────────────────────────────────────────────────────────
add_executable(kinetra_benchmarks
    bench_rrt_star.cpp
    bench_stomp.cpp
    bench_qp_solver.cpp
)

target_link_libraries(kinetra_benchmarks PRIVATE
    kinetra
    benchmark::benchmark
    benchmark::benchmark_main
)
