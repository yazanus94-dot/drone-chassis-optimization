"""Integration tests: the coupled pipeline runs end-to-end and is deterministic."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from drone_chassis import algorithms as alg
from drone_chassis.core import ArmSection, Requirements, first_pass_sizing
from drone_chassis.services import (
    DesignPoint,
    evaluate_design,
    run_design_of_experiments,
    summarize_comparison,
)


def test_beam_fem_matches_theory_across_lengths():
    for length_mm in (175.0, 225.0, 275.0):
        arm = ArmSection("carbon", 16.0, 1.5, length_mm)
        length = length_mm / 1000.0
        exact = alg.cantilever_tip_deflection(30.0, length, arm.bending_stiffness_nm2)
        fem = alg.beam_fem(32, length, arm.bending_stiffness_nm2, 30.0)[-1]
        assert fem == pytest.approx(exact, rel=2e-3)


def test_structural_stage_script(tmp_path):
    proc = subprocess.run(
        [sys.executable, str(ROOT / "simulation" / "structural" / "run_structural.py"),
         "--out", str(tmp_path / "out")],
        capture_output=True, text=True, timeout=300,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    out_file = tmp_path / "out" / "structural" / "structural_results.json"
    payload = json.loads(out_file.read_text())
    assert payload["solver_used"] in ("calculix", "validated_beam_model")
    assert payload["beam_model"]["safety_factor"] > 0
    assert payload["beam_model"]["mesh_convergence"]["rel_error"][-1] < 0.05


def test_cfd_stage_script(tmp_path):
    proc = subprocess.run(
        [sys.executable, str(ROOT / "simulation" / "cfd" / "run_cfd.py"),
         "--out", str(tmp_path / "out")],
        capture_output=True, text=True, timeout=300,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    payload = json.loads((tmp_path / "out" / "cfd" / "cfd_results.json").read_text())
    speeds = [r["speed_ms"] for r in payload["drag_points"]]
    drags = [r["drag_n"] for r in payload["drag_points"]]
    assert speeds == [5.0, 10.0, 15.0]
    assert drags[0] < drags[1] < drags[2]
    assert all(row["rel_error"] < 0.2 for row in payload["sphere_validation"])


def test_battery_stage_script(tmp_path):
    proc = subprocess.run(
        [sys.executable, str(ROOT / "battery" / "battery_sim.py"), "--out", str(tmp_path / "out")],
        capture_output=True, text=True, timeout=600,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    payload = json.loads((tmp_path / "out" / "battery" / "battery_results.json").read_text())
    times = [r["flight_time_min"] for r in payload["temperature_sweep"]]
    assert times[0] < times[2]
    assert 1.0 < payload["flight_time_min_25c"] < 120.0


def test_optimization_stage_script(tmp_path):
    proc = subprocess.run(
        [sys.executable, str(ROOT / "optimization" / "optimize.py"),
         "--samples", "8", "--out", str(tmp_path / "out")],
        capture_output=True, text=True, timeout=600,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    report = json.loads(proc.stdout)
    assert report["n_feasible"] >= 1
    assert report["comparison"]["flight_time_gain_pct"] >= -1e-9


def test_full_design_loop_deterministic_with_seed():
    req = Requirements()
    first, _ = run_design_of_experiments(req, n_samples=10, seed=99)
    second, _ = run_design_of_experiments(req, n_samples=10, seed=99)
    for a, b in zip(first, second, strict=False):
        assert a.design == b.design
        assert a.flight_time_min == pytest.approx(b.flight_time_min)


def test_baseline_versus_doe_best_tradeoff(requirements):
    designs, _ = run_design_of_experiments(requirements, n_samples=20, seed=20261113)
    baseline = evaluate_design(DesignPoint(225.0, 16.0, 1.5, "alu6061"), requirements)
    feasible = [e for e in designs if e.passes_constraints]
    assert feasible
    best = max(feasible, key=lambda e: e.flight_time_min)
    summary = summarize_comparison(baseline, best)
    assert summary["best_flight_min"] >= summary["baseline_flight_min"]
    sizing = first_pass_sizing(requirements, best.design.arm)
    assert sizing.total_mass_kg == pytest.approx(best.total_mass_kg, rel=1e-6)
