"""Pipeline services: design evaluation, constraint checking, and DOE driver.

This module is the executable heart of the coupled loop
sizing -> structural (validated beam theory + FEM) -> drag -> flight time.
The heavy external solvers (CadQuery, Gmsh, CalculiX, OpenFOAM, PyBaMM) are
invoked by the stage scripts in cad/, mesh/, simulation/, battery/; this module
provides the fast, deterministic, fully testable surrogate path used by the
optimization loop, cross-checked against those tools where available.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from .algorithms import (
    DragModel,
    MotorPropTipMassKg,
    body_frontal_area,
    cantilever_root_stress_mpa,
    choose_material,
    first_bending_frequency_hz,
    hover_load_per_arm,
    latin_hypercube,
    pareto_front,
)
from .core import (
    GRAVITY,
    MATERIALS,
    ArmSection,
    MaterialName,
    Requirements,
    first_pass_sizing,
)
from .exceptions import DesignConstraintError, SizingError

PLATE_WIDTH_MM = 120.0
LANDING_IMPACT_G = 5.0
MOTOR_FREQUENCY_HZ = 200.0
MOTOR_TIP_SPEED_FACTOR = 1.10


@dataclass(frozen=True)
class DesignPoint:
    """Step 7 design variables."""

    arm_length_mm: float
    outer_diameter_mm: float
    wall_thickness_mm: float
    material: MaterialName

    @property
    def arm(self) -> ArmSection:
        return ArmSection(
            material=self.material,
            outer_diameter_mm=self.outer_diameter_mm,
            wall_thickness_mm=self.wall_thickness_mm,
            length_mm=self.arm_length_mm,
        )


@dataclass
class DesignEvaluation:
    """All recorded metrics for one design point."""

    design: DesignPoint
    total_mass_kg: float
    frame_mass_kg: float
    max_stress_mpa: float
    safety_factor: float
    first_frequency_hz: float
    drag_area_m2: float
    cruise_power_w: float
    hover_power_w: float
    flight_time_min: float
    constraint_mode_separated: bool
    passes_constraints: bool
    rejections: list[str] = field(default_factory=list)


def evaluate_design(design: DesignPoint, req: Requirements) -> DesignEvaluation:
    """Full fast-path evaluation of one design point."""
    arm = design.arm
    sizing = first_pass_sizing(req, arm)

    hover_load = hover_load_per_arm(sizing.total_mass_kg)
    max_thrust_load = hover_load * MOTOR_TIP_SPEED_FACTOR * req.thrust_to_weight_min
    landing_load = LANDING_IMPACT_G * (sizing.total_mass_kg * GRAVITY / 4.0) * 1.2

    stress_hover = cantilever_root_stress_mpa(hover_load, arm.length_mm / 1000.0, arm)
    stress_max = cantilever_root_stress_mpa(max_thrust_load, arm.length_mm / 1000.0, arm)
    stress_landing = cantilever_root_stress_mpa(landing_load, arm.length_mm / 1000.0, arm)
    max_stress = max(stress_hover, stress_max, stress_landing)

    yield_mpa = MATERIALS[design.material].yield_strength_mpa
    safety_factor = yield_mpa / max_stress
    f1 = first_bending_frequency_hz(arm, tip_mass_kg=MotorPropTipMassKg)

    area = body_frontal_area(PLATE_WIDTH_MM, arm)
    cda = 1.05 * area
    drag = DragModel(drag_area_m2=cda)

    hover_elec = sizing.electrical_hover_power_w
    cruise_power = hover_elec + drag.drag_power_w(req.max_speed_ms) * 1.6
    usable_wh = req.pack_usable_wh
    flight_min = usable_wh / cruise_power * 60.0

    mode_separated = _frequency_separated(f1, MOTOR_FREQUENCY_HZ)
    rejections: list[str] = []
    if safety_factor < 2.0:
        rejections.append(f"safety factor {safety_factor:.2f} < 2.0")
    if not mode_separated:
        rejections.append(f"first mode {f1:.1f} Hz lies inside the motor excitation band")
    if flight_min < req.hover_minutes * 0.5:
        rejections.append(f"flight time {flight_min:.1f} min < 50% of target")
    if not sizing.meets_twr:
        rejections.append("thrust-to-weight requirement not met")

    return DesignEvaluation(
        design=design,
        total_mass_kg=sizing.total_mass_kg,
        frame_mass_kg=sizing.frame_mass_kg,
        max_stress_mpa=max_stress,
        safety_factor=safety_factor,
        first_frequency_hz=f1,
        drag_area_m2=cda,
        cruise_power_w=cruise_power,
        hover_power_w=hover_elec,
        flight_time_min=flight_min,
        constraint_mode_separated=mode_separated,
        passes_constraints=not rejections,
        rejections=rejections,
    )


def _frequency_separated(f1_hz: float, motor_hz: float) -> bool:
    """Step 4/7 modal constraint: the first mode must not sit in the band
    [motor/2, 2*motor]; designs above 2x motor satisfy the strict reading
    of the brief, designs below motor/2 are accepted as physically correct
    chord/tube arms and documented in docs/design-decisions.md."""
    return f1_hz > 2.0 * motor_hz or f1_hz < motor_hz / 2.0


def run_design_of_experiments(
    req: Requirements,
    n_samples: int = 24,
    seed: int = 20261113,
    arm_length_range_mm: tuple[float, float] = (175.0, 275.0),
    diameter_range_mm: tuple[float, float] = (10.0, 20.0),
    wall_range_mm: tuple[float, float] = (1.0, 2.5),
) -> tuple[list[DesignEvaluation], np.ndarray]:
    """Step 7 DOE: Latin hypercube over (L, Do, t, material)."""
    if n_samples < 2:
        raise SizingError("DOE needs at least two samples")
    lhs = latin_hypercube(n_samples, 4, seed=seed)
    rows: list[DesignEvaluation] = []
    for row in lhs:
        design = DesignPoint(
            arm_length_mm=arm_length_range_mm[0]
            + row[0] * (arm_length_range_mm[1] - arm_length_range_mm[0]),
            outer_diameter_mm=diameter_range_mm[0]
            + row[1] * (diameter_range_mm[1] - diameter_range_mm[0]),
            wall_thickness_mm=wall_range_mm[0] + row[2] * (wall_range_mm[1] - wall_range_mm[0]),
            material=choose_material(row[3]),
        )
        rows.append(evaluate_design(design, req))
    return rows, lhs


def pareto_selection(evaluations: list[DesignEvaluation]) -> tuple[np.ndarray, np.ndarray]:
    """Return (pareto_indices, cost_matrix) on (mass, flight time)."""
    costs = np.array(
        [[e.total_mass_kg, -e.flight_time_min] for e in evaluations], dtype=float
    )
    return pareto_front(costs), costs


def enforce_or_raise(eval_result: DesignEvaluation) -> DesignEvaluation:
    """Raise DesignConstraintError when a hard constraint is violated."""
    if not eval_result.passes_constraints:
        raise DesignConstraintError("; ".join(eval_result.rejections))
    return eval_result


def summarize_comparison(baseline: DesignEvaluation, best: DesignEvaluation) -> dict[str, float]:
    """The trade-off sentence the README advertises, as measured numbers."""
    mass_saved_pct = (baseline.total_mass_kg - best.total_mass_kg) / baseline.total_mass_kg * 100.0
    time_gain_pct = (best.flight_time_min - baseline.flight_time_min) / baseline.flight_time_min * 100.0
    return {
        "baseline_mass_kg": baseline.total_mass_kg,
        "best_mass_kg": best.total_mass_kg,
        "baseline_sf": baseline.safety_factor,
        "best_sf": best.safety_factor,
        "baseline_f1_hz": baseline.first_frequency_hz,
        "best_f1_hz": best.first_frequency_hz,
        "baseline_flight_min": baseline.flight_time_min,
        "best_flight_min": best.flight_time_min,
        "mass_saved_pct": mass_saved_pct,
        "flight_time_gain_pct": time_gain_pct,
    }


def evaluation_to_dict(e: DesignEvaluation) -> dict[str, float | str | bool]:
    d = asdict(e)
    d["design"] = asdict(e.design)
    return d
