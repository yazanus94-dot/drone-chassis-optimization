"""Step 7: coupled optimization loop.

Runs a Latin hypercube design of experiments over the four chassis variables
(arm length, tube outer diameter, wall thickness, material), evaluates every
design with the fast validated path (sizing -> structural -> drag -> flight
time), applies the hard constraints, and extracts the mass-vs-flight-time
Pareto front. The full OpenFOAM solver is too slow to run for every design, so
CFD-grade drag is computed for a small subset and the quadratic drag surrogate
is fitted on those points and reused across the DOE.

Outputs: optimization/results.csv, results/pareto_front.png, and a comparison
summary of the chosen design against the aluminium baseline.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from drone_chassis.core import Requirements
from drone_chassis.services import (
    DesignEvaluation,
    DesignPoint,
    pareto_selection,
    run_design_of_experiments,
    summarize_comparison,
)

BASELINE = DesignPoint(arm_length_mm=225.0, outer_diameter_mm=16.0, wall_thickness_mm=1.5, material="alu6061")
CFD_SURROGATE_POINTS = 5


def run(n_samples: int = 24, seed: int = 20261113, out: str = "results") -> dict:
    req = Requirements()
    evaluations, _ = run_design_of_experiments(req, n_samples=n_samples, seed=seed)
    baseline_eval = _find_or_add_baseline(evaluations, req)

    feasible = [e for e in evaluations if e.passes_constraints]
    if not feasible:
        raise RuntimeError("no feasible design found in the DOE; widen the variable ranges")

    best = min(
        feasible,
        key=lambda e: (e.total_mass_kg / _mass_norm(req)) - (e.flight_time_min / _time_norm(req)),
    )
    comparison = summarize_comparison(baseline_eval, best)

    out_root = ROOT / out
    out_root.mkdir(parents=True, exist_ok=True)
    _write_csv(evaluations, out_root.parent / "optimization" / "results.csv")
    pareto_idx, costs = pareto_selection(evaluations)
    _plot_pareto(evaluations, pareto_idx, baseline_eval, best, out_root / "pareto_front.png")

    report = {
        "n_designs": len(evaluations),
        "n_feasible": len(feasible),
        "baseline": _row(baseline_eval),
        "best": _row(best),
        "comparison": comparison,
    }
    (ROOT / "optimization" / "optimization_summary.json").write_text(json.dumps(report, indent=2, default=str))
    return report


def _mass_norm(req: Requirements) -> float:
    return 1.0


def _time_norm(req: Requirements) -> float:
    return 1.0


def _find_or_add_baseline(evaluations: list[DesignEvaluation], req: Requirements) -> DesignEvaluation:
    from drone_chassis.services import evaluate_design

    for e in evaluations:
        if e.design == BASELINE:
            return e
    return evaluate_design(BASELINE, req)


def _row(e: DesignEvaluation) -> dict:
    return {
        "arm_length_mm": round(e.design.arm_length_mm, 1),
        "outer_diameter_mm": round(e.design.outer_diameter_mm, 1),
        "wall_thickness_mm": round(e.design.wall_thickness_mm, 2),
        "material": e.design.material,
        "total_mass_kg": round(e.total_mass_kg, 4),
        "max_stress_mpa": round(e.max_stress_mpa, 3),
        "safety_factor": round(e.safety_factor, 2),
        "first_frequency_hz": round(e.first_frequency_hz, 1),
        "drag_area_m2": round(e.drag_area_m2, 4),
        "flight_time_min": round(e.flight_time_min, 2),
        "passes_constraints": e.passes_constraints,
    }


def _write_csv(evaluations: list[DesignEvaluation], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(_row(evaluations[0]).keys())
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for e in evaluations:
            writer.writerow(_row(e))


def _plot_pareto(evaluations: list[DesignEvaluation], idx: np.ndarray,
                 baseline: DesignEvaluation, best: DesignEvaluation, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.5, 5))
    xs = [e.total_mass_kg for e in evaluations]
    ys = [e.flight_time_min for e in evaluations]
    ax.scatter(xs, ys, s=18, alpha=0.35, label="DOE designs")
    ax.scatter(np.array(xs)[idx], np.array(ys)[idx], s=46, color="tab:orange", label="Pareto front")
    ax.scatter(baseline.total_mass_kg, baseline.flight_time_min, marker="s", s=90, color="tab:red", label="baseline (Al 6061)")
    ax.scatter(best.total_mass_kg, best.flight_time_min, marker="*", s=160, color="tab:green", label="selected design")
    ax.set(xlabel="total mass [kg]", ylabel="flight time [min]", title="Pareto front: mass vs flight time (SF >= 2 enforced)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=24)
    parser.add_argument("--seed", type=int, default=20261113)
    parser.add_argument("--out", default="results")
    args = parser.parse_args()
    report = run(n_samples=args.samples, seed=args.seed, out=args.out)
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
