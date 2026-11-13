# Debugging Log

## BUG-001: Beam FEM tip deflection near zero

- **Symptom.** `beam_fem` returned deflections orders of magnitude too small (and negative) for a tip-loaded cantilever; the mesh-convergence study looked perfect but meaningless.
- **Root cause.** The element stiffness matrix row 2 was `[-12, -6L, 12, +6L]`; the correct Euler-Bernoulli row is `[-12, -6L, 12, -6L]`. The wrong sign broke the matrix's symmetry at assembled shared dofs, effectively pinning the tip node's rotation against the applied load.
- **Debugging approach.** Verified `K == K.T` failed; reduced to a single-element case where the analytic solution `FL^3/3EI` is exact; bisected the element matrix entry by entry against the standard textbook matrix.
- **Fix.** Corrected the sign in `src/drone_chassis/algorithms.py` and in the Rayleigh-Ritz assembly used for mode shapes in `simulation/structural/run_structural.py`.
- **Regression tests.** `test_beam_fem_converges_to_theory`, `test_mesh_convergence_error_decreases` (monotone error decay on the lumped-load path), `test_beam_fem_distributed_load_matches_udl_theory`.

## BUG-002: PyBaMM constant-power discharge fails to converge

- **Symptom.** `Discharge at 43 W until 3.0 V` raised `SolverError: IDA_CONV_FAIL` under both IDAKLU and CasADi (safe and fast modes) at the ~2.3C hover load.
- **Root cause.** Algebraic (voltage/power) constraints at every step make constant-power events stiff at higher C-rates; the default solver settings could not hold the power constraint through the voltage knee.
- **Debugging approach.** Reproduced with a minimal script, toggled solver modes, then replaced the load type: the brief permits "power or current profile" as the load.
- **Fix.** Pack load applied as the equivalent constant current `I = P / V_nom`, with the actual average power recomputed from the solution and reported next to the target.
- **Regression tests.** `test_battery_stage_script` asserts the solved average power lands within 3% of the requested electrical load.

## BUG-003: Flight time 5x too long (97 min instead of ~19 min)

- **Symptom.** The battery stage reported 97.5 min of flight at 172.7 W hover power with a 74 Wh pack.
- **Root cause.** The per-cell current was derived as `P / (n_cells * V_nom)` while `V_nom` already includes the series count -- a double division, silently halving (per stack, quartering) the discharge current.
- **Debugging approach.** Ah-balance check: 5 Ah capacity cannot supply 11.7 A for 97 min; recomputed the applied current inside the solver call and compared against the requested one.
- **Fix.** `current_a = power_w_total / req.pack_nominal_voltage`.
- **Regression tests.** `test_battery_stage_script` bounds the resulting flight time between 1 and 120 min and asserts the temperature sweep ordering (colder = shorter).

## BUG-004: Gmsh flow-mesh boolean crash

- **Symptom.** `occ.cut` raised `BOPAlgo_AlertTooFewArguments` when called with an empty object list.
- **Root cause.** A leftover boolean-removal line with no tool objects; the far-field box needs no subtraction at this fidelity level (the refinement field handles near-body resolution).
- **Fix.** Removed the boolean; kept the Distance/Threshold refinement field.
- **Regression tests.** `test_cfd_stage_script` (flow artefacts) plus the mesh stage inside the integration suite.

## BUG-005: CadQuery chained-selector failure

- **Symptom.** `ValueError: No pending wires present` while building the plate pocket and motor mount.
- **Root cause.** Chained `.faces(">Z").workplane()` selectors after boolean cuts left the workplane stack in an inconsistent state across CadQuery 2.8.
- **Fix.** Rebuilt the geometry with explicit primitives and boolean cuts (`box().cut()`, `circle().extrude()`), avoiding fragile selector chains.
- **Regression tests.** `cad_model.py` runs inside `run_all.py`; the CAD stage is exercised by the demo-asset generator in CI.
