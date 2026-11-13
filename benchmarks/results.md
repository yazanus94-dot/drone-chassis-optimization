# Benchmark Results

Reproducible via `python benchmarks/benchmark.py` (seed 20261113, min of 5 runs).

| Benchmark | Result |
|---|---|
| DOE evaluation, 24 designs (full loop, seed 20261113) | 0.001 s |
| DOE sizing core, 24 designs (vectorized NumPy path) | 0.000 s |
| Beam FEM, 64 elements (assembly + solve) | 0.005 ms |
| Peak memory, full DOE loop vs sizing core | 0.0 MB vs 0.0 MB |

## Scaling

- Beam FEM assembly is O(n) elements with a dense O(n^2) solve; 8/16/32/64 elements measured locally.
- DOE is O(n) designs x O(1) per design; Pareto extraction is O(n^2) pairwise (n = 24 -> 576 comparisons, < 1 ms).
