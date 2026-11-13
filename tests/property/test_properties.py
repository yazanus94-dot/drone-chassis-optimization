"""Property-based tests (Hypothesis) with independent oracles."""

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from drone_chassis import algorithms as alg
from drone_chassis.core import (
    MATERIALS,
    ArmSection,
    Requirements,
    hover_power_induced,
    prop_disc_area,
)
from drone_chassis.services import DesignPoint, evaluate_design

positive = st.floats(min_value=0.05, max_value=80.0)
arm_lengths = st.floats(min_value=150.0, max_value=300.0)
diameters = st.floats(min_value=10.0, max_value=22.0)
walls = st.floats(min_value=1.0, max_value=2.5)
materials = st.sampled_from(["alu6061", "carbon", "petg"])


@given(diameter=st.floats(min_value=0.01, max_value=0.06), thrust=st.floats(min_value=5.0, max_value=60.0))
@settings(max_examples=50, deadline=None)
def test_momentum_power_monotonic_in_area(diameter, thrust):
    small = hover_power_induced(thrust, prop_disc_area(diameter))
    large = hover_power_induced(thrust, prop_disc_area(diameter * 1.5))
    assert small > large


@given(l1=arm_lengths, frac=st.floats(min_value=0.3, max_value=0.95), d=diameters, t=walls, m=materials)
@settings(max_examples=50, deadline=None)
def test_evaluation_mass_monotonic_in_arm_length(l1, frac, d, t, m):
    req = Requirements()
    bigger = evaluate_design(DesignPoint(l1, d, t, m), req)
    smaller = evaluate_design(DesignPoint(l1 * frac, d, t, m), req)
    assert smaller.total_mass_kg <= bigger.total_mass_kg + 1e-9


@given(force=st.floats(min_value=1.0, max_value=200.0), length_m=st.floats(min_value=0.1, max_value=1.5),
       stiffness=st.floats(min_value=1e5, max_value=5e7))
@settings(max_examples=50, deadline=None)
def test_cantilever_deflection_matches_oracle(force, length_m, stiffness):
    exact = alg.cantilever_tip_deflection(force, length_m, stiffness)
    assert exact == pytest.approx(force * length_m**3 / (3.0 * stiffness), rel=1e-12)


@given(do=st.floats(min_value=10.0, max_value=25.0), t=st.floats(min_value=1.0, max_value=3.0))
@settings(max_examples=40, deadline=None)
def test_tube_second_moment_grows_with_diameter(do, t):
    if 2 * t >= do:
        return
    arm = ArmSection("carbon", do, t, 225.0)
    fatter = ArmSection("carbon", do * 1.2, t, 225.0)
    assert fatter.second_moment_m4 > arm.second_moment_m4


@given(materials)
def test_materials_have_consistent_table(materials):
    mat = MATERIALS[materials]
    assert mat.density > 0 and mat.young_modulus_gpa > 0 and mat.yield_strength_mpa > 0


@given(points=st.lists(st.tuples(st.floats(min_value=0.1, max_value=10.0), st.floats(min_value=0.1, max_value=10.0)), min_size=1, max_size=25))
@settings(max_examples=40, deadline=None)
def test_pareto_front_has_no_dominated_members(points):
    costs = np.array(sorted(points))
    idx = set(alg.pareto_front(costs).tolist())
    for i in idx:
        c1, c2 = costs[i]
        for j, (d1, d2) in enumerate(costs):
            if j in idx or j == i:
                continue
            assert not (d1 <= c1 and d2 <= c2 and (d1 < c1 or d2 < c2))


@given(speeds=st.lists(st.floats(min_value=10.0, max_value=500.0), min_size=3, max_size=6))
@settings(max_examples=30, deadline=None)
def test_drag_surrogate_positive_and_monotone(speeds):
    speeds = sorted(set(speeds))
    if len(speeds) < 2:
        return
    cda = 0.03
    forces = [0.5 * 1.225 * cda * v**2 for v in speeds]
    model = alg.fit_drag_surrogate(speeds, forces)
    assert model.drag_area_m2 > 0
    for a, b in zip(speeds, speeds[1:], strict=False):
        assert model.drag_force_n(b) >= model.drag_force_n(a)
