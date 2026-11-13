"""Unit tests for the validated algorithm library."""

import numpy as np
import pytest

from drone_chassis import algorithms as alg
from drone_chassis.core import ArmSection


@pytest.fixture
def carbon_arm() -> ArmSection:
    return ArmSection("carbon", 16.0, 1.5, 225.0)


def test_cantilever_deflection_matches_beam_theory(carbon_arm):
    force, length = 30.0, carbon_arm.length_mm / 1000.0
    exact = force * length**3 / (3.0 * carbon_arm.bending_stiffness_nm2)
    assert alg.cantilever_tip_deflection(force, length, carbon_arm.bending_stiffness_nm2) == pytest.approx(exact)


def test_beam_fem_converges_to_theory(carbon_arm):
    length = carbon_arm.length_mm / 1000.0
    force = 30.0
    exact = alg.cantilever_tip_deflection(force, length, carbon_arm.bending_stiffness_nm2)
    fem_64 = alg.beam_fem(64, length, carbon_arm.bending_stiffness_nm2, tip_load_n=force)[-1]
    assert fem_64 == pytest.approx(exact, rel=2e-3)


def test_beam_fem_distributed_load_matches_udl_theory(carbon_arm):
    length = carbon_arm.length_mm / 1000.0
    q = (carbon_arm.mass_kg * 9.81) / length
    exact = q * length**4 / (8.0 * carbon_arm.bending_stiffness_nm2)
    fem_64 = alg.beam_fem(64, length, carbon_arm.bending_stiffness_nm2, distributed_load_n_per_m=q)[-1]
    assert fem_64 == pytest.approx(exact, rel=1e-3)


def test_mesh_convergence_error_decreases(carbon_arm):
    conv = alg.mesh_convergence(carbon_arm, tip_load_n=30.0)
    errors = conv["rel_error"]
    assert errors[-1] < errors[0]
    assert all(err < 0.05 for err in errors[2:])


def test_sphere_correlation_published_values():
    assert alg.sphere_drag_coefficient(1e3) == pytest.approx(0.47, abs=0.05)
    assert alg.sphere_drag_coefficient(1e4) == pytest.approx(0.42, abs=0.06)


def test_cylinder_newtonian_plateau():
    assert alg.cylinder_drag_coefficient(1e4) == pytest.approx(1.2)
    assert alg.cylinder_drag_coefficient(1e5) == pytest.approx(1.2)


def test_drag_surrogate_fit_recovers_quadratic():
    speeds = [5.0, 10.0, 15.0]
    cda = 0.02
    forces = [0.5 * 1.225 * cda * v**2 for v in speeds]
    model = alg.fit_drag_surrogate(speeds, forces)
    assert model.drag_area_m2 == pytest.approx(cda, rel=1e-9)
    assert model.drag_force_n(10.0) == pytest.approx(forces[1])


def test_latin_hypercube_stratified_and_deterministic():
    d1 = alg.latin_hypercube(10, 3, seed=7)
    d2 = alg.latin_hypercube(10, 3, seed=7)
    assert np.allclose(d1, d2)
    assert d1.shape == (10, 3)
    assert np.all((d1 >= 0) & (d1 < 1))
    for dim in range(3):
        strata = np.floor(d1[:, dim] * 10).astype(int)
        assert sorted(strata) == list(range(10))


def test_pareto_front_exact_small_case():
    costs = np.array([[1.0, 5.0], [2.0, 4.0], [3.0, 1.0], [4.0, 5.0]])
    idx = alg.pareto_front(costs)
    assert set(idx.tolist()) == {0, 1, 2}


def test_pareto_dominates_are_removed():
    costs = np.array([[1.0, 1.0], [1.0, 2.0], [2.0, 1.0], [3.0, 3.0]])
    idx = alg.pareto_front(costs)
    assert list(idx) == [0]


def test_choose_material_maps_correctly():
    assert alg.choose_material(0.0) == "alu6061"
    assert alg.choose_material(0.5) == "carbon"
    assert alg.choose_material(0.99) == "petg"
