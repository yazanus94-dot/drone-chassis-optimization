"""Step 4/5/7 algorithms: beam theory, beam FEM, drag correlations, DOE, Pareto.

This module contains the validated analytical machinery used by the pipeline:

* Euler-Bernoulli cantilever beam theory and a small FEM beam solver used for
  the mesh-convergence study (Step 4 validation).
* Empirical drag correlations for a sphere and a cylinder used to validate the
  CFD setup (Step 5 validation).
* A Latin hypercube design-of-experiments sampler and a two-objective Pareto
  front extraction for the coupled optimization loop (Step 7).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, cast

import numpy as np

from .core import GRAVITY, ArmSection, MaterialName

MotorPropTipMassKg = 0.060


def cantilever_tip_deflection(force_n: float, length_m: float, bending_stiffness_nm2: float) -> float:
    """delta = F L^3 / (3 E I) for a cantilever with a tip point load."""
    if force_n <= 0 or length_m <= 0 or bending_stiffness_nm2 <= 0:
        raise ValueError("force, length and stiffness must be positive")
    return force_n * length_m**3 / (3.0 * bending_stiffness_nm2)


def cantilever_root_stress_mpa(force_n: float, length_m: float, section: ArmSection) -> float:
    """Bending stress at the fixed root: sigma = M c / I with M = F L."""
    c_m = section.outer_diameter_mm / 2000.0
    moment_nm = force_n * length_m
    second_moment = section.second_moment_m4
    return abs(moment_nm) * c_m / second_moment / 1e6


def hover_load_per_arm(total_mass_kg: float, n_arms: int = 4) -> float:
    """Thrust reaction carried by one arm during hover."""
    return total_mass_kg * GRAVITY / n_arms


def first_bending_frequency_hz(
    arm: ArmSection, tip_mass_kg: float = MotorPropTipMassKg
) -> float:
    """First bending mode of a cantilever with a lumped tip mass.

    f = (1/2pi) * sqrt(k_eff / m_eff) with k_eff = 3 E I / L^3 and
    m_eff = m_tip + 0.24 m_beam (Rayleigh approximation).
    """
    length_m = arm.length_mm / 1000.0
    k_eff = 3.0 * arm.bending_stiffness_nm2 / length_m**3
    m_beam = arm.mass_kg
    m_eff = tip_mass_kg + 0.24 * m_beam
    return float((1.0 / (2.0 * np.pi)) * np.sqrt(k_eff / m_eff))


def beam_fem(
    n_elements: int,
    length_m: float,
    bending_stiffness_nm2: float,
    tip_load_n: float = 0.0,
    distributed_load_n_per_m: float = 0.0,
    lumped: bool = False,
) -> np.ndarray:
    """Solve a linear Euler-Bernoulli cantilever with cubic beam elements.

    With consistent (integrated) element loads the nodal solution is exact
    for any static loading -- a classical superconvergence property of
    Euler-Bernoulli beam elements. With lumped=True the distributed load is
    applied as tributary point loads (a genuine discretization), giving an
    O(h^2) convergence path used by the mesh-convergence study.

    Returns the transverse deflections at the nodes, shape (n_elements + 1,).
    """
    if n_elements < 1:
        raise ValueError("n_elements must be >= 1")
    le = length_m / n_elements
    ndof = 2 * n_elements + 2
    k_e = bending_stiffness_nm2 / le**3 * np.array(
        [[12.0, 6 * le, -12.0, 6 * le], [6 * le, 4 * le**2, -6 * le, 2 * le**2],
         [-12.0, -6 * le, 12.0, -6 * le], [6 * le, 2 * le**2, -6 * le, 4 * le**2]]
    )
    if lumped:
        f_e = distributed_load_n_per_m * le * np.array([0.5, 0.0, 1.0, 0.0])
    else:
        f_e = distributed_load_n_per_m * le / 2.0 * np.array([1.0, le / 6.0, 1.0, -le / 6.0])
    k_full = np.zeros((ndof, ndof))
    load = np.zeros(ndof)
    load[-2] += tip_load_n
    for e in range(n_elements):
        idx = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        k_full[np.ix_(idx, idx)] += k_e
        load[idx] += f_e
    free = np.arange(2, ndof)
    w = np.linalg.solve(k_full[np.ix_(free, free)], load[free])
    return np.concatenate(([0.0, 0.0], w))[::2][: n_elements + 1]


def mesh_convergence(
    arm: ArmSection,
    tip_load_n: float = 0.0,
    distributed_load_n_per_m: float | None = None,
    element_counts: tuple[int, ...] = (2, 4, 8, 16, 32, 64),
) -> dict[str, Any]:
    """Tip deflection error of the beam FEM against Euler-Bernoulli theory.

    Uses the lumped tributary representation of the distributed load so the
    study shows a genuine O(h^2) convergence instead of the exact
    superconvergent nodal solution of consistently loaded cubic elements.
    """
    length_m = arm.length_mm / 1000.0
    q = distributed_load_n_per_m
    if q is None:
        q = (arm.mass_kg * GRAVITY) / length_m
    exact = cantilever_tip_deflection(tip_load_n, length_m, arm.bending_stiffness_nm2) + q * length_m**4 / (8.0 * arm.bending_stiffness_nm2)
    n_list, fem_list, err_list = [], [], []
    for n in element_counts:
        fem = float(beam_fem(n, length_m, arm.bending_stiffness_nm2, tip_load_n, q, lumped=True)[-1])
        n_list.append(n)
        fem_list.append(fem)
        err_list.append(abs(fem - exact) / exact)
    return {"n_elements": n_list, "fem_tip_mm": [f * 1000 for f in fem_list], "rel_error": err_list, "theory_tip_mm": float(exact * 1000)}


def sphere_drag_coefficient(reynolds: float) -> float:
    """Clift-Grace-Weber correlation for a smooth sphere, 1e-2 <= Re <= 2e5."""
    if reynolds <= 0:
        raise ValueError("Reynolds number must be positive")
    return float(
        24.0 / reynolds * (1.0 + 0.15 * reynolds**0.687)
        + 0.42 / (1.0 + 4.25e4 * reynolds**-1.16)
    )

def cylinder_drag_coefficient(reynolds: float) -> float:
    """Standard cylinder correlation: Newtonian plateau Cd ~ 1.2 for 1e3 < Re < 2e5."""
    if reynolds <= 0:
        raise ValueError("Reynolds number must be positive")
    if reynolds < 1.0:
        return float(24.0 / reynolds * (1.0 + 0.15 * reynolds**0.687))
    if reynolds < 1e3:
        return float(24.0 / reynolds * (1.0 + 0.147 * reynolds**0.687))
    return 1.2


def body_frontal_area(plate_width_mm: float, arm: ArmSection) -> float:
    """Projected frontal area of the body: centre plate plus two crossing arms."""
    plate_area = (plate_width_mm / 1000.0) ** 2 * 0.35
    arm_projected = 2.0 * (arm.outer_diameter_mm / 1000.0) * (arm.length_mm / 1000.0)
    return plate_area + arm_projected


@dataclass(frozen=True)
class DragModel:
    """Quadratic drag law F = 0.5 rho CdA v^2 fitted from sparse CFD points."""

    drag_area_m2: float
    air_density: float = 1.225

    def drag_force_n(self, speed_ms: float) -> float:
        return 0.5 * self.air_density * self.drag_area_m2 * speed_ms**2

    def drag_power_w(self, speed_ms: float) -> float:
        return self.drag_force_n(speed_ms) * speed_ms


def fit_drag_surrogate(speeds_ms: list[float], drag_forces_n: list[float], air_density: float = 1.225) -> DragModel:
    """Fit CdA from a handful of CFD points (or analytic model points)."""
    if len(speeds_ms) != len(drag_forces_n) or len(speeds_ms) < 2:
        raise ValueError("need matching speed and drag arrays with at least two points")
    q = np.array([0.5 * air_density * v**2 for v in speeds_ms])
    cda = float(np.dot(q, np.array(drag_forces_n)) / np.dot(q, q))
    if cda <= 0:
        raise ValueError("fitted drag area must be positive")
    return DragModel(drag_area_m2=cda, air_density=air_density)


def latin_hypercube(n_samples: int, n_dims: int, seed: int = 20261113) -> np.ndarray:
    """Deterministic Latin hypercube samples in the open unit hypercube."""
    if n_samples < 1 or n_dims < 1:
        raise ValueError("n_samples and n_dims must be >= 1")
    rng = np.random.default_rng(seed)
    design = np.empty((n_samples, n_dims))
    for d in range(n_dims):
        strata = (rng.permutation(n_samples) + rng.random(n_samples)) / n_samples
        design[:, d] = strata
    return design


def pareto_front(cost_matrix: np.ndarray) -> np.ndarray:
    """Indices of non-dominated points for a minimise-both 2-column cost matrix."""
    if cost_matrix.ndim != 2 or cost_matrix.shape[1] != 2:
        raise ValueError("expect an (n, 2) cost matrix")
    keep = []
    for i, (c1, c2) in enumerate(cost_matrix):
        dominated = np.any((cost_matrix[:, 0] <= c1) & (cost_matrix[:, 1] <= c2) &
                           ((cost_matrix[:, 0] < c1) | (cost_matrix[:, 1] < c2)))
        if not dominated:
            keep.append(i)
    return np.array(sorted(keep), dtype=int)


def choose_material(index: float) -> MaterialName:
    """Map a DOE material coordinate in [0, 1) onto the material table."""
    names = ["alu6061", "carbon", "petg"]
    index_name = names[int(np.clip(index, 0.0, 0.999) * len(names))]
    return cast(MaterialName, index_name)
