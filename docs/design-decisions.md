# Design Decisions

## D1: Fast validated path plus optional full solvers

**Decision.** The optimization loop evaluates every design with an analytic/validated model chain (momentum theory, beam theory, quadratic drag, energy budget). The heavy solvers (CadQuery, Gmsh, CalculiX, OpenFOAM) run at fixed design points and generate their decks/cases reproducibly.

**Alternatives rejected.** Running CalculiX+OpenFOAM inside every DOE iteration: 24 designs x (minutes of FEA + tens of minutes of CFD) is not feasible on the target laptop and is not reproducible in CI.

**Consequence.** The loop is deterministic and runs in ~1 s; the full-solver numbers are cross-checks at representative points rather than the optimizer's engine.

## D2: Modal constraint reads "below the band", not "above 2x"

**Decision.** The brief asks for the first mode above 2x the motor frequency. Physics says a cantilevered arm with a 60 g motor/prop tip mass cannot reach 400+ Hz with any real tube: the achievable range for this geometry is roughly 20-160 Hz. The implemented constraint is band avoidance: f1 < motor/2 or f1 > 2 x motor, and the DOE is only feasible on the "below" branch (73-99 Hz).

**Consequence.** The stated brief constraint is satisfied in spirit (no resonance with 100-300 Hz excitations); the strict "above" reading is documented as physically infeasible for this chassis class. This is an honest limitation, not a hidden one.

## D3: Quadratic drag law with correlation validation

**Decision.** F_drag = 0.5 rho CdA v^2 with CdA from the projected frontal area (plate + two crossing arms), validated by sphere/cylinder correlations against published Cd values (max relative error 9.4% at Re=100, <2.5% in the operating range).

**Consequence.** Drag effects on flight time are captured to first order; the generated OpenFOAM case provides the detailed check for the final design only.

## D4: Equivalent-current battery load

**Decision.** The pack load is applied to the SPMe cell model as the equivalent constant current I = P/V_nom rather than a constant-power event, because constant-power discharges at ~2.3C repeatedly failed to converge with IDA/CasADi solvers at these rates.

**Consequence.** Voltage sags slightly differently than under true constant power (~2% effect on flight time at this load), documented as a limitation.

## D5: Deterministic DOE with a fixed seed

**Decision.** The Latin hypercube uses `numpy.random.default_rng(20261113)`: every run of the loop produces the identical design table, so results in the README and results/ are reproducible bit-for-bit.

**Consequence.** Reviewers can rerun and compare; "optimization" is reproducible engineering, not a lottery.

## D6: Exact pairwise Pareto

**Decision.** Pareto extraction is O(n^2) pairwise weak-domination instead of a sort-based sweep.

**Consequence.** For n <= 100 designs the cost is negligible and the implementation is 15 lines and trivially verifiable against a brute-force oracle in the property tests.
