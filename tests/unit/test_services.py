"""Unit tests for the pipeline services (design evaluation and constraints)."""

import pytest

from drone_chassis.exceptions import DesignConstraintError, SizingError
from drone_chassis.services import (
    DesignPoint,
    enforce_or_raise,
    evaluate_design,
    pareto_selection,
    run_design_of_experiments,
    summarize_comparison,
)


def test_evaluate_design_baseline_feasible(requirements, baseline_design):
    result = evaluate_design(baseline_design, requirements)
    assert result.total_mass_kg > 0
    assert result.safety_factor > 1.0
    assert result.first_frequency_hz > 0
    assert result.flight_time_min > 0


def test_safety_factor_improves_with_material_strength(requirements):
    alu = evaluate_design(DesignPoint(225.0, 16.0, 1.5, "alu6061"), requirements)
    carbon = evaluate_design(DesignPoint(225.0, 16.0, 1.5, "carbon"), requirements)
    assert carbon.safety_factor > alu.safety_factor


def test_petg_design_is_rejected(requirements):
    result = evaluate_design(DesignPoint(225.0, 16.0, 1.5, "petg"), requirements)
    assert not result.passes_constraints
    assert any("safety factor" in r for r in result.rejections)


def test_enforce_or_raise_raises_on_infeasible(requirements):
    weak = evaluate_design(DesignPoint(275.0, 10.0, 1.0, "petg"), requirements)
    with pytest.raises(DesignConstraintError):
        enforce_or_raise(weak)


def test_first_frequency_band_logic(requirements):
    in_band = evaluate_design(DesignPoint(225.0, 16.0, 1.5, "alu6061"), requirements)
    assert 100.0 < in_band.first_frequency_hz < 300.0
    assert not in_band.constraint_mode_separated
    below_band = evaluate_design(DesignPoint(275.0, 16.0, 1.5, "alu6061"), requirements)
    assert below_band.first_frequency_hz < 100.0
    assert below_band.constraint_mode_separated


def test_heavier_design_has_less_flight_time(requirements):
    light = evaluate_design(DesignPoint(175.0, 10.0, 1.0, "carbon"), requirements)
    heavy = evaluate_design(DesignPoint(275.0, 20.0, 2.5, "alu6061"), requirements)
    assert light.flight_time_min > heavy.flight_time_min


def test_doe_returns_requested_designs(requirements):
    designs, lhs = run_design_of_experiments(requirements, n_samples=12, seed=1)
    assert len(designs) == 12
    assert lhs.shape == (12, 4)
    lengths = {d.design.arm_length_mm for d in designs}
    assert len(lengths) == 12


def test_doe_deterministic(requirements):
    d1, _ = run_design_of_experiments(requirements, n_samples=6, seed=5)
    d2, _ = run_design_of_experiments(requirements, n_samples=6, seed=5)
    assert d1[0].design == d2[0].design
    assert d1[0].flight_time_min == pytest.approx(d2[0].flight_time_min)


def test_pareto_selection_prefers_lighter_longer(requirements):
    designs, _ = run_design_of_experiments(requirements, n_samples=15, seed=2)
    idx, costs = pareto_selection(designs)
    assert len(idx) >= 1
    assert costs.shape == (len(designs), 2)
    best = designs[int(idx[0])]
    worst = max(designs, key=lambda e: e.total_mass_kg)
    assert best.total_mass_kg <= worst.total_mass_kg


def test_summary_comparison_percentages(requirements):
    d1, _ = run_design_of_experiments(requirements, n_samples=10, seed=3)
    baseline = d1[0]
    best = max(d1, key=lambda e: e.flight_time_min)
    summary = summarize_comparison(baseline, best)
    assert -100.0 <= summary["mass_saved_pct"] <= 100.0
    assert summary["flight_time_gain_pct"] >= 0.0


def test_invalid_doe_size_raises(requirements):
    with pytest.raises(SizingError):
        run_design_of_experiments(requirements, n_samples=1)
