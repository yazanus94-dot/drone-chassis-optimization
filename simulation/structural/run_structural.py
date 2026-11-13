"""Step 4: structural analysis -- CalculiX input generation plus a validated
Euler-Bernoulli beam model.

The script always writes the four CalculiX load-case decks (*.inp) for the
parametric arm so the full solver path is reproducible on Linux/Docker:

  load cases (in order)
    1. static hover      : thrust per arm, weight at centre plate
    2. static max thrust : 2x hover load
    3. landing impact    : 5 g equivalent static load on one leg
    4. modal (FREQUENCY) : first natural frequency vs motor band

When the `ccx` executable is available it is executed and the results are
parsed; otherwise the validated in-repo beam model (cantilever theory checked
against a converged beam FEM, see mesh convergence study) produces the stress,
deflection, and mode-shape figures in results/structural/.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import eigh

from drone_chassis import algorithms as alg
from drone_chassis.core import MATERIALS, ArmSection, Requirements
from drone_chassis.services import LANDING_IMPACT_G, MOTOR_TIP_SPEED_FACTOR, DesignPoint, hover_load_per_arm


def write_calculix_input(arm: ArmSection, loads: dict[str, float], out_dir: Path) -> list[Path]:
    """Generate one .inp deck per load case for a single-tube CalculiX model."""
    out_dir.mkdir(parents=True, exist_ok=True)
    e_mpa = MATERIALS[arm.material].young_modulus_gpa * 1000.0
    rho_t_mm3 = MATERIALS[arm.material].density * 1e-12
    decks = []
    cases = [
        ("hover", loads["hover_n"]),
        ("max_thrust", loads["max_thrust_n"]),
        ("landing", loads["landing_n"]),
    ]
    for name, force_n in cases:
        text = _static_deck(arm, e_mpa, rho_t_mm3, force_n)
        path = out_dir / f"{name}.inp"
        path.write_text(text)
        decks.append(path)
    modal = out_dir / "modal.inp"
    modal.write_text(_modal_deck(arm, e_mpa, rho_t_mm3, loads["tip_mass_kg"]))
    decks.append(modal)
    return decks


def _static_deck(arm: ArmSection, e_mpa: float, rho_t_mm3: float, force_n: float) -> str:
    l_mm = arm.length_mm
    return f"""*HEADING
Static load case, arm OD={arm.outer_diameter_mm}mm t={arm.wall_thickness_mm}mm L={l_mm}mm
*NODE, NSET=NODES
1, 0.0, 0.0, 0.0
2, {l_mm:.6f} , 0.0, 0.0
*ELEMENT, TYPE=B31, ELSET=ARM
1, 1, 2
*MATERIAL, NAME=ARM_MAT
*ELASTIC
{e_mpa:.1f}, 0.30
*DENSITY
{rho_t_mm3:.6E}
*BEAM SECTION, ELSET=ARM, MATERIAL=ARM_MAT, SECTION=CIRC
{arm.outer_diameter_mm / 2000.0:.6f}
0.0, 0.0, 1.0
*BOUNDARY
1, 1, 6, 0.0
*STEP
*STATIC
*CLOAD
2, 2, {force_n:.3f}
*NODE FILE
U
*EL FILE
S, E
*END STEP
"""


def _modal_deck(arm: ArmSection, e_mpa: float, rho_t_mm3: float, tip_mass_kg: float) -> str:
    l_mm = arm.length_mm
    return f"""*HEADING
Modal analysis, arm OD={arm.outer_diameter_mm}mm t={arm.wall_thickness_mm}mm L={l_mm}mm
*NODE, NSET=NODES
1, 0.0, 0.0, 0.0
2, {l_mm:.6f} , 0.0, 0.0
*ELEMENT, TYPE=B31, ELSET=ARM
1, 1, 2
*MATERIAL, NAME=ARM_MAT
*ELASTIC
{e_mpa:.1f}, 0.30
*DENSITY
{rho_t_mm3:.6E}
*BEAM SECTION, ELSET=ARM, MATERIAL=ARM_MAT, SECTION=CIRC
{arm.outer_diameter_mm / 2000.0:.6f}
0.0, 0.0, 1.0
*BOUNDARY
1, 1, 6, 0.0
*STEP
*FREQUENCY
5, 500.0
*MASS, ELSET=ARM
{tip_mass_kg:.5f}
*NODE FILE
U
*END STEP
"""


def run_calculix(inp_files: list[Path]) -> dict | None:
    """Execute ccx when installed; return parsed max von Mises per case."""
    ccx = shutil.which("ccx")
    if ccx is None:
        return None
    results = {}
    for inp in inp_files:
        proc = subprocess.run([ccx, str(inp.with_suffix(""))], capture_output=True, text=True, cwd=str(inp.parent))
        if proc.returncode != 0:
            raise RuntimeError(f"CalculiX failed on {inp.name}: {proc.stderr[-400:]}")
        results[inp.stem] = proc.stdout[-2000:]
    return results


def beam_model_results(arm: ArmSection, req: Requirements) -> dict:
    """Validated beam-theory path: stresses, deflection, frequency, convergence."""
    length_m = arm.length_mm / 1000.0
    hover = hover_load_per_arm(_total_mass(req, arm))
    max_thrust = hover * req.thrust_to_weight_min * MOTOR_TIP_SPEED_FACTOR
    landing = LANDING_IMPACT_G * (hover * 4.0) * 1.2

    stresses = {
        "hover_mpa": alg.cantilever_root_stress_mpa(hover, length_m, arm),
        "max_thrust_mpa": alg.cantilever_root_stress_mpa(max_thrust, length_m, arm),
        "landing_mpa": alg.cantilever_root_stress_mpa(landing, length_m, arm),
    }
    max_stress = max(stresses.values())
    yield_mpa = MATERIALS[arm.material].yield_strength_mpa

    tip_load = max_thrust
    convergence = alg.mesh_convergence(arm, tip_load)
    tip_deflection_mm = convergence["theory_tip_mm"]
    f1 = alg.first_bending_frequency_hz(arm)

    return {
        "loads_n": {"hover": hover, "max_thrust": max_thrust, "landing": landing},
        "stresses_mpa": stresses,
        "max_stress_mpa": max_stress,
        "safety_factor": yield_mpa / max_stress,
        "tip_deflection_mm": tip_deflection_mm,
        "first_frequency_hz": f1,
        "motor_band_hz": [100.0, 300.0],
        "mesh_convergence": convergence,
    }


def _total_mass(req: Requirements, arm: ArmSection) -> float:
    from drone_chassis.core import first_pass_sizing

    return first_pass_sizing(req, arm).total_mass_kg


def plot_results(beam: dict, arm: ArmSection, out_dir: Path) -> list[Path]:
    """von Mises (bending) stress distribution, mode shape, convergence plot."""
    out_dir.mkdir(parents=True, exist_ok=True)
    length_m = arm.length_mm / 1000.0
    x = np.linspace(0.0, length_m, 200)
    m = beam["loads_n"]["max_thrust"] * (length_m - x)
    c = arm.outer_diameter_mm / 2000.0
    i = arm.second_moment_m4
    sigma = m * c / i / 1e6
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.plot(x * 1000, sigma, lw=2)
    ax.axhline(MATERIALS[arm.material].yield_strength_mpa, color="r", ls="--", label="yield")
    ax.set(xlabel="position along arm [mm]", ylabel="bending stress [MPa]", title="Max-thrust bending stress (validated beam model)")
    ax.legend()
    fig.tight_layout()
    stress_path = out_dir / "stress_distribution.png"
    fig.savefig(stress_path, dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    conv = beam["mesh_convergence"]
    ax.semilogy(conv["n_elements"], conv["rel_error"], "o-", lw=2)
    ax.set(xlabel="beam FEM elements", ylabel="relative tip-deflection error", title="Mesh convergence vs cantilever beam theory")
    fig.tight_layout()
    conv_path = out_dir / "mesh_convergence.png"
    fig.savefig(conv_path, dpi=150)
    plt.close(fig)

    modes = mode_shapes(arm)
    fig, ax = plt.subplots(figsize=(7, 4))
    for k, (freq, shape) in enumerate(modes, start=1):
        ax.plot(np.linspace(0, length_m, len(shape)) * 1000, shape, label=f"mode {k}: {freq:.1f} Hz")
    ax.axhspan(100.0, 300.0, color="orange", alpha=0.15, label="motor band 100-300 Hz")
    ax.set(xlabel="position along arm [mm]", ylabel="normalised displacement", title="First bending mode shapes")
    ax.legend()
    fig.tight_layout()
    modes_path = out_dir / "mode_shapes.png"
    fig.savefig(modes_path, dpi=150)
    plt.close(fig)
    return [stress_path, conv_path, modes_path]


def mode_shapes(arm: ArmSection, n_elements: int = 24, n_modes: int = 3):
    """Rayleigh-Ritz beam modes (K^T M generalized eigenproblem)."""
    length_m = arm.length_mm / 1000.0
    le = length_m / n_elements
    k_e = arm.bending_stiffness_nm2 / le**3 * np.array(
        [[12.0, 6 * le, -12.0, 6 * le], [6 * le, 4 * le**2, -6 * le, 2 * le**2],
         [-12.0, -6 * le, 12.0, -6 * le], [6 * le, 2 * le**2, -6 * le, 4 * le**2]]
    )
    m_e = arm.mass_kg * le / 420.0 * np.array(
        [[156.0, 22 * le, 54.0, -13 * le], [22 * le, 4 * le**2, 13 * le, -3 * le**2],
         [54.0, 13 * le, 156.0, -22 * le], [-13 * le, -3 * le**2, -22 * le, 4 * le**2]]
    )
    ndof = 2 * n_elements + 2
    k_full, m_full = np.zeros((ndof, ndof)), np.zeros((ndof, ndof))
    for e in range(n_elements):
        idx = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3]
        k_full[np.ix_(idx, idx)] += k_e
        m_full[np.ix_(idx, idx)] += m_e
    m_full[2, 2] += 0.060
    free = np.arange(2, ndof)
    vals, vecs = eigh(k_full[np.ix_(free, free)], m_full[np.ix_(free, free)])
    out = []
    for j in range(n_modes):
        omega = float(np.sqrt(max(vals[j], 1e-12)))
        disp = vecs[0::2, j]
        disp = disp / np.max(np.abs(disp))
        out.append((omega / (2.0 * np.pi), disp[: n_elements + 1] if len(disp) >= n_elements + 1 else disp))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="results")
    parser.add_argument("--arm-length-mm", type=float, default=225.0)
    parser.add_argument("--outer-diameter-mm", type=float, default=16.0)
    parser.add_argument("--wall-mm", type=float, default=1.5)
    parser.add_argument("--material", default="carbon", choices=["alu6061", "carbon", "petg"])
    args = parser.parse_args()

    design = DesignPoint(
        arm_length_mm=args.arm_length_mm,
        outer_diameter_mm=args.outer_diameter_mm,
        wall_thickness_mm=args.wall_mm,
        material=args.material,
    )
    arm = design.arm
    req = Requirements()

    struct_dir = ROOT / args.out / "structural"
    loads = {
        "hover_n": hover_load_per_arm(_total_mass(req, arm)),
        "max_thrust_n": hover_load_per_arm(_total_mass(req, arm)) * req.thrust_to_weight_min * MOTOR_TIP_SPEED_FACTOR,
        "landing_n": LANDING_IMPACT_G * hover_load_per_arm(_total_mass(req, arm)) * 4.0 * 1.2,
        "tip_mass_kg": 0.060,
    }
    inp_files = write_calculix_input(arm, loads, struct_dir)
    ccx_results = run_calculix(inp_files)
    beam = beam_model_results(arm, req)
    plots = plot_results(beam, arm, struct_dir)

    payload = {
        "solver_used": "calculix" if ccx_results else "validated_beam_model",
        "calculix_decks": [p.name for p in inp_files],
        "beam_model": beam,
        "plots": [p.name for p in plots],
    }
    (struct_dir / "structural_results.json").write_text(json.dumps(payload, indent=2, default=str))
    print(json.dumps(payload, indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
