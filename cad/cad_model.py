"""Step 2: parametric CadQuery model of the X-quad chassis.

Builds the centre plate, four arms, motor mounts, and landing legs from a
single parameter set, exports STEP (meshing) and STL (printing), and reports
mass and centre of gravity computed from the CAD solids so that Step 1 sizing
can be corrected with the true geometry.

Run:  python cad/cad_model.py [--out results] [--arm-length-mm 225]
The generated artefacts land in results/cad/.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import cadquery as cq
import numpy as np

from drone_chassis.core import GRAVITY, MATERIALS, ArmSection
from drone_chassis.services import DesignPoint

PLATE_WIDTH_MM = 120.0
PLATE_THICKNESS_MM = 6.0
PLATE_HOLE_DIA_MM = 4.0
PLATE_HOLE_OFFSET_MM = 45.0
MOUNT_DIA_MM = 22.0
MOUNT_HEIGHT_MM = 8.0
LEG_LENGTH_MM = 90.0
LEG_DIAMETER_MM = 8.0
LEG_ANGLE_DEG = 30.0


def build_chassis(arm: ArmSection, n_arms: int = 4):
    """Return the parametric assembly solids: (plate, arms, mounts, legs)."""
    arms: list[cq.Workplane] = []
    mounts: list[cq.Workplane] = []
    legs: list[cq.Workplane] = []
    half = PLATE_WIDTH_MM / 2.0

    plate = cq.Workplane("XY").box(
        PLATE_WIDTH_MM, PLATE_WIDTH_MM, PLATE_THICKNESS_MM, centered=(True, True, False)
    )
    pocket = cq.Workplane("XY").box(
        PLATE_WIDTH_MM * 0.45, PLATE_WIDTH_MM * 0.45, PLATE_THICKNESS_MM,
        centered=(True, True, False),
    )
    plate = plate.cut(pocket)
    motor_wire_hole = (
        cq.Workplane("XY").moveTo(0.0, PLATE_HOLE_OFFSET_MM)
        .circle(PLATE_HOLE_DIA_MM / 2.0).extrude(PLATE_THICKNESS_MM)
    )
    plate = plate.cut(motor_wire_hole)

    tube = (
        cq.Workplane("XY").circle(arm.outer_diameter_mm / 2.0)
        .circle(arm.inner_diameter_mm / 2.0).extrude(arm.length_mm)
        .rotate((0, 0, 0), (0, 1, 0), 90.0)
    )
    mount = (
        cq.Workplane("XY").circle(MOUNT_DIA_MM / 2.0).extrude(MOUNT_HEIGHT_MM)
        .faces(">Z").workplane().hole(3.0)
    )
    leg = (
        cq.Workplane("XY").circle(LEG_DIAMETER_MM / 2.0).extrude(LEG_LENGTH_MM)
        .rotate((0, 0, 0), (1, 0, 0), LEG_ANGLE_DEG)
    )

    for i in range(n_arms):
        angle = 45.0 + i * 90.0
        direction = cq.Vector(np.cos(np.radians(angle)), np.sin(np.radians(angle)), 0.0)
        pos = direction * (arm.length_mm / 2.0 + half * 0.4)
        arms.append(tube.translate(cq.Vector(pos.x, pos.y, PLATE_THICKNESS_MM)))
        mounts.append(mount.translate(
            cq.Vector(pos.x, pos.y, PLATE_THICKNESS_MM + arm.outer_diameter_mm / 2.0 + MOUNT_HEIGHT_MM / 2.0)
        ))
        legs.append(leg.translate(cq.Vector(pos.x * 0.6, pos.y * 0.6, 0.0)))

    return plate, arms, mounts, legs


def assembly_mass_and_cog(plate, arms, mounts, legs, arm: ArmSection):
    """Mass (kg) and CoG (mm) from CAD solids: arms use the arm material,
    plate/mounts/legs use aluminium 6061."""
    fixed_density = MATERIALS["alu6061"].density * 1e-9
    arm_density = MATERIALS[arm.material].density * 1e-9
    total_mass = 0.0
    cog_weighted = cq.Vector(0, 0, 0)
    for s in arms:
        v = s.val().Volume()
        total_mass += v * arm_density
        cog_weighted += s.val().Center() * (v * arm_density)
    for s in [plate] + mounts + legs:
        v = s.val().Volume()
        total_mass += v * fixed_density
        cog_weighted += s.val().Center() * (v * fixed_density)
    cog_mm = cog_weighted * (1.0 / sum(s.val().Volume() for s in arms + [plate] + mounts + legs))
    return total_mass, (cog_mm.x, cog_mm.y, cog_mm.z)


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

    out_dir = ROOT / args.out / "cad"
    out_dir.mkdir(parents=True, exist_ok=True)

    plate, arms, mounts, legs = build_chassis(arm)
    assembly = plate
    for part in arms + mounts + legs:
        assembly = assembly.union(part)

    cq.exporters.export(assembly, str(out_dir / "drone.step"))
    cq.exporters.export(assembly, str(out_dir / "drone.stl"))
    cq.exporters.export(assembly, str(out_dir / "render.svg"),
                        opt={"width": 640, "height": 480, "marginLeft": 20, "marginTop": 20})

    mass_kg, cog = assembly_mass_and_cog(plate, arms, mounts, legs, arm)
    payload = {
        "arm": {"material": arm.material, "outer_diameter_mm": arm.outer_diameter_mm,
                "wall_thickness_mm": arm.wall_thickness_mm, "length_mm": arm.length_mm},
        "cad_mass_kg": round(mass_kg, 5),
        "cad_center_of_gravity_mm": [round(c, 2) for c in cog],
        "gravity_accel_ms2": GRAVITY,
    }
    (out_dir / "cad_mass.json").write_text(json.dumps(payload, indent=2))
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
