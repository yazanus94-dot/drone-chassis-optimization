# Changelog

## 1.0.0 - 2026-11-13

- Initial release: full six-stage pipeline (sizing, CadQuery CAD, Gmsh meshes, structural analysis with CalculiX deck generation, validated drag model with OpenFOAM case generation, PyBaMM battery discharge).
- Coupled DOE optimization loop (Latin hypercube, seed 20261113) with constraint filtering and exact Pareto front extraction.
- Selected design: 194.5 mm x 10.9 x 1.41 mm Al-6061 arms, 4.2% lighter and 10.6% more flight time than the 225 x 16 x 1.5 mm baseline, SF 4.9, f1 73 Hz.
- 39 unit/property tests + 7 integration tests, ruff and mypy (strict) clean.
- Docker Compose environment and GitHub Actions CI.
