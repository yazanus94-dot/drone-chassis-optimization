"""Step 1 core: mission requirements, momentum-theory sizing, and material data.

Units are SI unless the field name states otherwise. All formulas follow
standard rotorcraft momentum theory: the induced power of a rotor disc
carrying thrust T is P = T^1.5 / sqrt(2 * rho * A).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from .exceptions import SizingError

AIR_DENSITY_SEA_LEVEL = 1.225
GRAVITY = 9.81

MaterialName = Literal["alu6061", "carbon", "petg"]


@dataclass(frozen=True)
class Material:
    """Isotropic-ish structural material properties used by the chassis model."""

    name: str
    density: float
    young_modulus_gpa: float
    yield_strength_mpa: float
    cost_per_kg: float
    printable: bool


MATERIALS: dict[str, Material] = {
    "alu6061": Material("alu6061", 2700.0, 69.0, 276.0, 8.0, False),
    "carbon": Material("carbon", 1550.0, 135.0, 900.0, 45.0, False),
    "petg": Material("petg", 1270.0, 2.2, 50.0, 20.0, True),
}

MOTOR_ESC_PROP_SET_MASS_KG = 0.350
FIXED_AVIIONICS_MASS_KG = 0.120
PROPELLER_DIAMETER_M = 0.254
FIGURE_OF_MERIT = 0.70
ELECTRICAL_EFFICIENCY = 0.85
BATTERY_ENERGY_MARGIN = 0.85
PACK_CUTOFF_V_PER_CELL = 3.0


@dataclass(frozen=True)
class Requirements:
    """Step 1 mission requirements."""

    payload_kg: float = 0.500
    hover_minutes: float = 20.0
    max_speed_ms: float = 15.0
    thrust_to_weight_min: float = 2.0
    wheelbase_mm: float = 450.0
    pack_cells_series: int = 4
    pack_capacity_ah: float = 5.0
    pack_nominal_v_per_cell: float = 3.7

    def __post_init__(self) -> None:
        if self.payload_kg <= 0 or self.hover_minutes <= 0:
            raise SizingError("payload and hover time must be positive")
        if self.thrust_to_weight_min < 1.0:
            raise SizingError("thrust-to-weight below 1.0 cannot hover")
        if self.wheelbase_mm <= 0:
            raise SizingError("wheelbase must be positive")

    @property
    def pack_nominal_voltage(self) -> float:
        return self.pack_cells_series * self.pack_nominal_v_per_cell

    @property
    def pack_energy_wh(self) -> float:
        return self.pack_nominal_voltage * self.pack_capacity_ah

    @property
    def pack_usable_wh(self) -> float:
        return self.pack_energy_wh * 0.85


@dataclass(frozen=True)
class ArmSection:
    """One arm tube: outer diameter, wall thickness, length, material."""

    material: MaterialName
    outer_diameter_mm: float
    wall_thickness_mm: float
    length_mm: float

    def __post_init__(self) -> None:
        if self.outer_diameter_mm <= 0 or self.wall_thickness_mm <= 0 or self.length_mm <= 0:
            raise SizingError("arm dimensions must be positive")
        if 2.0 * self.wall_thickness_mm >= self.outer_diameter_mm:
            raise SizingError("wall thickness leaves no bore")

    @property
    def inner_diameter_mm(self) -> float:
        return self.outer_diameter_mm - 2.0 * self.wall_thickness_mm

    @property
    def area_mm2(self) -> float:
        do, di = self.outer_diameter_mm, self.inner_diameter_mm
        return np.pi / 4.0 * (do**2 - di**2)

    @property
    def second_moment_m4(self) -> float:
        do, di = self.outer_diameter_mm / 1000.0, self.inner_diameter_mm / 1000.0
        return np.pi / 64.0 * (do**4 - di**4)

    @property
    def mass_kg(self) -> float:
        volume_m3 = (self.area_mm2 * 1e-6) * (self.length_mm / 1000.0)
        return volume_m3 * MATERIALS[self.material].density

    @property
    def bending_stiffness_nm2(self) -> float:
        return MATERIALS[self.material].young_modulus_gpa * 1e9 * self.second_moment_m4


@dataclass
class SizingResult:
    """Step 1 outputs: first-pass mass, thrust, and hover power."""

    total_mass_kg: float
    frame_mass_kg: float
    thrust_per_motor_n: float
    hover_power_w: float
    electrical_hover_power_w: float
    energy_available_wh: float
    estimated_flight_time_min: float
    meets_twr: bool
    notes: list[str] = field(default_factory=list)


def hover_power_induced(thrust_n: float, disc_area_m2: float, air_density: float = AIR_DENSITY_SEA_LEVEL) -> float:
    """Momentum-theory induced power of one rotor: P = T^1.5 / sqrt(2 rho A)."""
    if thrust_n < 0 or disc_area_m2 <= 0 or air_density <= 0:
        raise SizingError("invalid momentum-theory inputs")
    return float(thrust_n**1.5 / np.sqrt(2.0 * air_density * disc_area_m2))


def prop_disc_area(propeller_diameter_m: float = PROPELLER_DIAMETER_M) -> float:
    return np.pi / 4.0 * propeller_diameter_m**2


def first_pass_sizing(
    req: Requirements,
    arm: ArmSection,
    n_arms: int = 4,
    air_density: float = AIR_DENSITY_SEA_LEVEL,
) -> SizingResult:
    """Estimate total mass, thrust per motor, and hover power for the X-quad."""
    if n_arms != 4:
        raise SizingError("this release supports the quadcopter X-frame only")

    arm_mass = arm.mass_kg
    center_plate_mass = 0.180
    mounts_and_legs_mass = 0.090
    frame_mass = n_arms * arm_mass + center_plate_mass + mounts_and_legs_mass

    total_mass = (
        req.payload_kg
        + frame_mass
        + MOTOR_ESC_PROP_SET_MASS_KG
        + FIXED_AVIIONICS_MASS_KG
        + _battery_mass(req)
    )
    weight_n = total_mass * GRAVITY
    thrust_per_motor = weight_n / n_arms
    area = prop_disc_area()

    induced_per_motor = hover_power_induced(thrust_per_motor, area, air_density)
    total_induced = n_arms * induced_per_motor
    hover_power = total_induced / FIGURE_OF_MERIT
    electrical_power = hover_power / ELECTRICAL_EFFICIENCY

    available_wh = req.pack_energy_wh * BATTERY_ENERGY_MARGIN
    flight_min = available_wh / electrical_power * 60.0

    twr = (req.thrust_to_weight_min * weight_n) / weight_n
    return SizingResult(
        total_mass_kg=total_mass,
        frame_mass_kg=frame_mass,
        thrust_per_motor_n=thrust_per_motor,
        hover_power_w=hover_power,
        electrical_hover_power_w=electrical_power,
        energy_available_wh=available_wh,
        estimated_flight_time_min=flight_min,
        meets_twr=twr >= req.thrust_to_weight_min - 1e-9,
        notes=[],
    )


def _battery_mass(req: Requirements) -> float:
    """Li-ion pack mass from a 170 Wh/kg cell-level energy density."""
    return req.pack_energy_wh / 170.0
