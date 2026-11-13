# Verification Strategy

Every physical model in this project is verified against an independent oracle before its numbers are allowed to influence the optimization loop.

## Oracle table

| Model | Oracle | Test |
|---|---|---|
| Rotor momentum theory | Hand calculation P = T^1.5 / sqrt(2 rho A) x corrections | `test_momentum_theory_power_matches_hand_calculation` |
| Tube section properties | Closed-form annulus area/inertia | `test_tube_section_properties` |
| Euler-Bernoulli FEM (tip load) | delta = FL^3/3EI at mesh-converged limit | `test_beam_fem_converges_to_theory` |
| FEM (distributed load) | delta = qL^4/8EI | `test_beam_fem_distributed_load_matches_udl_theory` |
| Mesh convergence | Monotone error decay, O(h^2) with consistent loads, superconvergence at nodes | `test_mesh_convergence_error_decreases`, plot in `results/structural/mesh_convergence.png` |
| First mode (Rayleigh-Ritz) | Omega = (1.875/L)^2 sqrt(EI/(rho A)) | `test_first_mode_matches_rayleigh_ritz` |
| Drag (sphere, Re 100-1e5) | Clift-Grace-Weber published Cd | `test_sphere_drag_matches_published_values`, `test_cfd_stage_script` |
| Drag (cylinder) | Szechenyi published Cd | `test_cylinder_drag_matches_published_values` |
| Energy budget | Flight time = usable Wh / cruise W | `test_flight_time_matches_energy_budget` |
| PyBaMM SPMe | Closed-form energy budget (+/- 10%) | `test_battery_stage_script` |
| Pareto extraction | Brute-force domination check | `test_extract_pareto_front_brute_force` |
| LHC stratification | One sample per stratum | `test_latin_hypercube_stratification` |
| CAD mass | Analytic tube volume x density | CAD stage JSON vs sizing budget |
| Determinism | Same seed -> identical DOE results | `test_full_design_loop_deterministic_with_seed` |

## Coverage

`pytest --cov` reports 90%+ line coverage of `src/drone_chassis` (unit + property + integration).

## Known deviations

- CalculiX `.inp` decks and OpenFOAM cases are generated and validated in structure, but the numeric results quoted in this repository come from the in-repo validated beam/drag models; the deck execution path requires a Linux/Docker solver install and is exercised in CI as an artefact check only.
- The modal constraint is band avoidance rather than the brief's literal "above 2x motor frequency" (see design-decisions D2).
