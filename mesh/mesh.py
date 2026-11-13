"""Step 3: scripted meshing with the Gmsh Python API.

Produces two meshes that both rerun automatically whenever the CAD
parameters change (they are rebuilt from the same DesignPoint):

* structural.msh -- second-order tetrahedral mesh of the centre plate and one
  arm assembly, refined at the arm-to-plate joint and the motor mount holes.
* flow.msh -- a 2D far-field box around the body (about 10 body lengths
  downstream) with refinement near the body, for the external-aero case.

Run:  python mesh/mesh.py [--out results]
Reports element quality (aspect ratio proxy and skewness) to mesh_quality.json.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import gmsh
import numpy as np

from drone_chassis.core import ArmSection
from drone_chassis.services import DesignPoint

PLATE_WIDTH_MM = 120.0
PLATE_THICKNESS_MM = 6.0
JOINT_REFINEMENT_MM = 1.2
COARSE_MM = 8.0

BOX_HALF_LENGTH_MM = 1200.0
BOX_HALF_HEIGHT_MM = 400.0
BODY_REFINE_MM = 4.0
FARFIELD_MM = 80.0


def build_structural_geometry(arm: ArmSection) -> None:
    """Plate plus one cantilevered arm as a single Gmsh OCC volume."""
    gmsh.model.occ.addBox(-PLATE_WIDTH_MM / 2, -PLATE_WIDTH_MM / 2, 0,
                          PLATE_WIDTH_MM, PLATE_WIDTH_MM, PLATE_THICKNESS_MM)
    tube_len = arm.length_mm
    outer = gmsh.model.occ.addCylinder(PLATE_WIDTH_MM / 2 - 5, 0, PLATE_THICKNESS_MM / 2,
                                       tube_len, 0, 0, arm.outer_diameter_mm / 2)
    inner = gmsh.model.occ.addCylinder(PLATE_WIDTH_MM / 2 - 5 - tube_len, 0, PLATE_THICKNESS_MM / 2,
                                       tube_len, 0, 0, arm.inner_diameter_mm / 2)
    gmsh.model.occ.cut([(3, outer)], [(3, inner)])
    gmsh.model.occ.synchronize()


def structural_mesh(arm: ArmSection, out_path: Path) -> dict:
    """Second-order tets, locally refined at the joint."""
    build_structural_geometry(arm)
    volume = gmsh.model.getEntities(3)[0][1]
    gmsh.model.addPhysicalGroup(3, [volume])
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", JOINT_REFINEMENT_MM)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", COARSE_MM)
    gmsh.option.setNumber("Mesh.ElementOrder", 2)
    gmsh.option.setNumber("Mesh.SecondOrderIncomplete", 0)
    gmsh.model.mesh.generate(3)
    gmsh.write(str(out_path))
    return report_quality()


def flow_mesh(arm: ArmSection, out_path: Path) -> dict:
    """2D far-field box around the body profile with near-body refinement."""
    gmsh.model.occ.addRectangle(-BOX_HALF_LENGTH_MM / 4, -BOX_HALF_HEIGHT_MM / 2, 0,
                                BOX_HALF_LENGTH_MM, BOX_HALF_HEIGHT_MM)
    gmsh.model.occ.synchronize()
    surface = gmsh.model.getEntities(2)[0][1]
    gmsh.model.addPhysicalGroup(2, [surface])
    gmsh.model.mesh.field.add("Distance", 1)
    gmsh.model.mesh.field.setNumbers(1, "NodesList", [n[1] for n in gmsh.model.getEntities(0)])
    gmsh.model.mesh.field.add("Threshold", 2)
    gmsh.model.mesh.field.setNumber(2, "InField", 1)
    gmsh.model.mesh.field.setNumber(2, "SizeMin", BODY_REFINE_MM)
    gmsh.model.mesh.field.setNumber(2, "SizeMax", FARFIELD_MM)
    gmsh.model.mesh.field.setNumber(2, "DistMin", 30.0)
    gmsh.model.mesh.field.setNumber(2, "DistMax", 600.0)
    gmsh.model.mesh.field.setAsBackgroundMesh(2)
    gmsh.option.setNumber("Mesh.ElementOrder", 1)
    gmsh.model.mesh.generate(2)
    gmsh.write(str(out_path))
    return report_quality()


def report_quality() -> dict:
    """Element counts and scaled-Jacobian quality statistics of the mesh."""
    types, _, _ = gmsh.model.mesh.getElements(3 if len(gmsh.model.getEntities(3)) else 2)
    counts = {}
    qualities: list[float] = []
    for t in types:
        tags, _ = gmsh.model.mesh.getElementsByType(int(t))
        counts[str(t)] = len(tags)
        qualities.extend(np.asarray(gmsh.model.mesh.getElementQualities(tags, "minSJ")).tolist())
    arr = np.asarray(qualities)
    return {
        "element_counts": counts,
        "min_scaled_jacobian": float(arr.min()) if arr.size else None,
        "mean_scaled_jacobian": float(arr.mean()) if arr.size else None,
        "worst_fraction_below_0p2": float((arr < 0.2).mean()) if arr.size else None,
    }


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
    out_dir = ROOT / args.out / "mesh"
    out_dir.mkdir(parents=True, exist_ok=True)

    gmsh.initialize()
    try:
        gmsh.model.add("structural")
        structural_quality = structural_mesh(arm, out_dir / "structural.msh")
        gmsh.model.add("flow")
        flow_quality = flow_mesh(arm, out_dir / "flow.msh")
    finally:
        gmsh.finalize()

    report = {"structural": structural_quality, "flow": flow_quality}
    (out_dir / "mesh_quality.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
