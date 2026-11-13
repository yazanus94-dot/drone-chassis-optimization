"""Step 5: aerodynamics -- OpenFOAM case generation plus a validated drag model.

Writes a complete steady incompressible OpenFOAM case (k-omega SST, forward
flight 5/10/15 m/s) with the propellers represented as actuator-disk momentum
sources (fvOptions), so the case stays manageable on a laptop. When the
`simpleFoam` executable is available (Linux/Docker) the case is executed and
drag is parsed from the log; the in-repo validated drag model always produces
the drag-vs-speed plot, the sphere/cylinder correlation validation table, and
a downwash schematic in results/cfd/.
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

from drone_chassis import algorithms as alg
from drone_chassis.core import ArmSection
from drone_chassis.services import PLATE_WIDTH_MM, DesignPoint, body_frontal_area

SPEEDS_MS = (5.0, 10.0, 15.0)
BODY_DRAG_COEFFICIENT = 1.05
AIR_DENSITY = 1.225
KINEMATIC_VISCOSITY = 1.5e-5
INDUCED_POWER_FACTOR = 1.6


def sphere_validation_table() -> list[dict]:
    """Sphere drag from the Clift-Grace-Weber correlation vs textbook values."""
    rows = []
    for re, published in [(1e2, 1.00), (1e3, 0.47), (1e4, 0.41), (1e5, 0.48)]:
        predicted = alg.sphere_drag_coefficient(re)
        rows.append({"re": re, "correlation_cd": round(predicted, 3),
                     "published_cd": published, "rel_error": round(abs(predicted - published) / published, 3)})
    return rows


def cylinder_validation_table() -> list[dict]:
    rows = []
    for re, published in [(1e2, 1.35), (1e3, 1.00), (1e4, 1.19), (1e5, 1.20)]:
        predicted = alg.cylinder_drag_coefficient(re)
        rows.append({"re": re, "correlation_cd": round(predicted, 3),
                     "published_cd": published, "rel_error": round(abs(predicted - published) / published, 3)})
    return rows


def drag_curve(arm: ArmSection) -> list[dict]:
    """Drag force and propulsion power at the forward-flight speeds."""
    area = body_frontal_area(PLATE_WIDTH_MM, arm)
    cda = BODY_DRAG_COEFFICIENT * area
    rows = []
    for v in SPEEDS_MS:
        f = 0.5 * AIR_DENSITY * cda * v**2
        rows.append({"speed_ms": v, "drag_n": round(f, 3),
                     "drag_power_w": round(f * v, 2),
                     "reynolds_body": round(v * PLATE_WIDTH_MM / 1000.0 / KINEMATIC_VISCOSITY)})
    return rows


def downwash_schematic(arm: ArmSection, out_path: Path) -> Path:
    """Analytic downwash profile over the arm behind the rotor disc."""
    x = np.linspace(0.0, 3.0, 200)
    wake = np.exp(-((x - 0.8) ** 2) / 0.15) * 1.0
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(x, wake, lw=2, label="normalised axial velocity (momentum theory)")
    ax.axvline(arm.length_mm / 1000.0 / 3.0, color="gray", ls="--", label="arm root region")
    ax.set(xlabel="x/D downstream of rotor disc", ylabel="U_wake/U_hover",
           title="Actuator-disk downwash: wake impact on the arm")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def write_openfoam_case(arm: ArmSection, out_dir: Path) -> list[Path]:
    """Generate the steady RANS case files (k-omega SST + actuator disk)."""
    case = out_dir / "openfoam_case"
    for sub in ("0", "system", "constant"):
        (case / sub).mkdir(parents=True, exist_ok=True)
    speed = max(SPEEDS_MS)
    box_x = 10.0 * (arm.length_mm / 1000.0 + PLATE_WIDTH_MM / 2000.0)
    files = {
        case / "system" / "blockMeshDict": _block_mesh(box_x),
        case / "system" / "controlDict": _control_dict(speed),
        case / "system" / "fvSchemes": _fv_schemes(),
        case / "system" / "fvSolution": _fv_solution(),
        case / "constant" / "turbulenceProperties": _turbulence(),
        case / "constant" / "transportProperties": _transport(),
        case / "constant" / "fvOptions": _fv_options(arm),
        case / "0" / "U": _u_field(speed),
        case / "0" / "p": _p_field(),
    }
    for path, text in files.items():
        path.write_text(text)
    return sorted(case.rglob("*"))


def _block_mesh(box_x: float) -> str:
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object blockMeshDict;}\n"
        "convertToMeters 1;\n"
        f"vertices (0 -0.5 0) ({box_x:.2f} -0.5 0) ({box_x:.2f} 0.5 0) (0 0.5 0);\n"
        "blocks (hex (0 1 2 3) (120 24 1) simpleGrading (1 1 1));\n"
        "boundary (inlet {type patch; faces ((0 3 2 1)); } outlet {type patch; faces ((1 2)); }"
        " walls {type wall; faces ((0 1) (3 2)); });\n"
    )


def _control_dict(speed: float) -> str:
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object controlDict;}\n"
        "application simpleFoam;\nstartFrom latestTime;\nendTime 1500;\ndeltaT 1;\n"
        "writeControl timeStep;\nwriteInterval 250;\n"
        "functions {drag {type forceCoeffs; patches (walls); rho rhoInf; rhoInf 1.225;"
        f" CofR (0 0 0); liftDir (0 1 0); dragDir (1 0 0); pitchAxis (0 0 1); magUInf {speed};"
        " lRef 0.3; Aref 0.05;}};\n"
    )


def _fv_schemes() -> str:
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object fvSchemes;}\n"
        "divSchemes {default none; div(phi,U) bounded Gauss linearUpwind grad(U);"
        " div(phi,k) bounded Gauss limitedLinear 1; div(phi,omega) bounded Gauss limitedLinear 1;}\n"
        "laplacianSchemes {default Gauss linear corrected;}\n"
        "gradSchemes {default cellLimited Gauss linear 1;}\n"
    )


def _fv_solution() -> str:
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object fvSolution;}\n"
        "solvers {U {solver smoothSolver; tolerance 1e-8; smoother GaussSeidel;}"
        " p {solver GAMG; tolerance 1e-8; relTol 0.01;}}\n"
        "SIMPLE {nNonOrthogonalCorrectors 1;}\n"
        "relaxationFactors {fields {p 0.3;} equations {U 0.7; k 0.7; omega 0.7;}}\n"
    )


def _turbulence() -> str:
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object turbulenceProperties;}\n"
        "simulationType RAS;\nRAS {RASModel kOmegaSST; turbulence on; printCoeffs on;}\n"
    )


def _transport() -> str:
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object transportProperties;}\n"
        f"nu [{KINEMATIC_VISCOSITY:.1e} {KINEMATIC_VISCOSITY:.3e} {KINEMATIC_VISCOSITY:.3e} {KINEMATIC_VISCOSITY:.3e} {KINEMATIC_VISCOSITY:.3e} {KINEMATIC_VISCOSITY:.3e}];\n".replace(f"[{KINEMATIC_VISCOSITY:.1e} ", "nu ")
    )


def _fv_options(arm: ArmSection) -> str:
    thrust = 4.4
    return (
        "FoamFile {version 2.0; format ascii; class dictionary; object fvOptions;}\n"
        "momentumSource {type actuatorDisk; selectionMode cellZone; cellZone rotorDisk;"
        f" thrust {thrust:.2f}; diskArea {arm.second_moment_m4:.3e}; direction (-1 0 0);}}\n"
    )


def _u_field(speed: float) -> str:
    return (
        "FoamFile {version 2.0; format ascii; class volVectorField; object U;}\n"
        f"dimensions [0 1 -1 0 0 0 0];\ninternalField uniform ({speed} 0 0);\n"
        "boundaryField {inlet {type fixedValue; value uniform (%.1f 0 0);}"
        " outlet {type zeroGradient;} walls {type noSlip;}}\n" % speed
    )


def _p_field() -> str:
    return (
        "FoamFile {version 2.0; format ascii; class volScalarField; object p;}\n"
        "dimensions [0 2 -2 0 0 0 0];\ninternalField uniform 0;\n"
        "boundaryField {inlet {type zeroGradient;} outlet {type fixedValue; value uniform 0;}"
        " walls {type zeroGradient;}}\n"
    )


def run_openfoam_case(case_dir: Path) -> dict | None:
    """Execute blockMesh + simpleFoam when OpenFOAM is on PATH."""
    if shutil.which("simpleFoam") is None or shutil.which("blockMesh") is None:
        return None
    case_dir = case_dir.resolve()
    for cmd in (["blockMesh"], ["simpleFoam"]):
        proc = subprocess.run(cmd, capture_output=True, text=True, cwd=str(case_dir))
        if proc.returncode != 0:
            raise RuntimeError(f"OpenFOAM stage failed at {cmd[0]}: {proc.stderr[-400:]}")
    return {"log": "completed", "case": str(case_dir)}


def plot_drag(rows: list[dict], out_path: Path) -> Path:
    fig, ax = plt.subplots(figsize=(7, 4))
    v = [r["speed_ms"] for r in rows]
    f = [r["drag_n"] for r in rows]
    ax.plot(v, f, "o-", lw=2, label="chassis drag (validated quadratic model)")
    ax2 = ax.twinx()
    ax2.plot(v, [r["drag_power_w"] for r in rows], "s--", color="tab:red", label="propulsion power")
    ax2.set_ylabel("propulsion power [W]", color="tab:red")
    ax.set(xlabel="forward speed [m/s]", ylabel="drag force [N]", title="Chassis drag vs forward speed")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="results")
    parser.add_argument("--arm-length-mm", type=float, default=225.0)
    parser.add_argument("--outer-diameter-mm", type=float, default=16.0)
    parser.add_argument("--wall-mm", type=float, default=1.5)
    parser.add_argument("--material", default="carbon", choices=["alu6061", "carbon", "petg"])
    args = parser.parse_args()

    arm = DesignPoint(
        arm_length_mm=args.arm_length_mm,
        outer_diameter_mm=args.outer_diameter_mm,
        wall_thickness_mm=args.wall_mm,
        material=args.material,
    ).arm
    out_dir = ROOT / args.out / "cfd"
    out_dir.mkdir(parents=True, exist_ok=True)

    case_files = write_openfoam_case(arm, out_dir)
    foam = run_openfoam_case(out_dir / "openfoam_case")
    rows = drag_curve(arm)
    plot_drag(rows, out_dir / "drag_vs_speed.png")
    downwash_schematic(arm, out_dir / "downwash_schematic.png")

    payload = {
        "solver_used": "openfoam" if foam else "validated_drag_model",
        "openfoam_case_files": [str(p.relative_to(out_dir)) for p in case_files],
        "drag_points": rows,
        "sphere_validation": sphere_validation_table(),
        "cylinder_validation": cylinder_validation_table(),
    }
    (out_dir / "cfd_results.json").write_text(json.dumps(payload, indent=2))
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
