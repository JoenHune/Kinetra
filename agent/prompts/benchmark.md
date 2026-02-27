# Benchmark Phase

Run Google Benchmark suites and compare with the previous baseline.

## Metrics to Track

- **RRT***: planning time vs path quality (length, smoothness).
- **STOMP**: iteration time, convergence rate.
- **QP Solver**: solve time vs problem size, iteration count.
- **Collision**: query time for grid lookup and segment checks.

## Regression Detection

Flag any benchmark that is >{threshold}% slower than the baseline.
Investigate root cause: algorithmic change, memory allocation, cache misses.
