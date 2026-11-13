"""Unit tests for the Step 1 sizing core against hand-computed values."""

import math

import pytest

from drone_chassis.core import (
    AIR_DENSITY_SEA_LEVEL,
    GRAVITY,
    MATERIALS,
    ArmSection,
    Requirements,
    first_pass_sizing,
    hover_power_induced,
    prop_disc_area,
)
from drone_chassis.exceptions import SizingError


def test_momentum_theory_matches_hand_calculation():
    thrust = 4.42
    area = prop_disc_area(0.254)
    expected = thrust**1.5 / math.sqrt(2.0 * AIR_DENSITY_SEA_LEVEL * area)
    assert hover_power_induced(thrust, area) == pytest.approx(expected, rel=1e-12)


def test_hover_power_increases_with_thrust():
    area = prop_disc_area()
    assert hover_power_induced(10.0, area) < hover_power_induced(20.0, area)


def test_hover_power_decreases_with_disc_area():
    assert hover_power_induced(10.0, prop_disc_area(0.20)) > hover_power_induced(10.0, prop_disc_area(0.30))


def test_invalid_momentum_inputs_raise():
    with pytest.raises(SizingError):
        hover_power_induced(-1.0, 0.05)
    with pytest.raises(SizingError):
        hover_power_induced(1.0, 0.0)


def test_arm_section_geometry_exact():
    arm = ArmSection("carbon", 16.0, 1.5, 225.0)
    di = 16.0 - 2 * 1.5
    assert arm.inner_diameter_mm == pytest.approx(13.0)
    assert arm.area_mm2 == pytest.approx(math.pi / 4 * (16.0**2 - di**2))
    do_m, di_m = 0.016, 0.013
    assert arm.second_moment_m4 == pytest.approx(math.pi / 64 * (do_m**4 - di_m**4), rel=1e-9)


def test_arm_mass_matches_density_times_volume():
    arm = ArmSection("alu6061", 12.0, 1.0, 200.0)
    di = 12.0 - 2.0
    volume_m3 = math.pi / 4 * (0.012**2 - (di / 1000.0) ** 2) * 0.2
    assert arm.mass_kg == pytest.approx(volume_m3 * MATERIALS["alu6061"].density, rel=1e-9)


def test_wall_thicker_than_bore_raises():
    with pytest.raises(SizingError):
        ArmSection("carbon", 10.0, 5.1, 200.0)


def test_first_pass_sizing_mass_budget(requirements, carbon_arm):
    result = first_pass_sizing(requirements, carbon_arm)
    battery = requirements.pack_energy_wh / 170.0
    expected = (
        requirements.payload_kg
        + result.frame_mass_kg
        + 0.350
        + 0.120
        + battery
    )
    assert result.total_mass_kg == pytest.approx(expected, rel=1e-6)
    assert result.frame_mass_kg > 0
    assert result.thrust_per_motor_n == pytest.approx(result.total_mass_kg * GRAVITY / 4.0)


def test_first_pass_flight_time_consistent(requirements, carbon_arm):
    result = first_pass_sizing(requirements, carbon_arm)
    expected_min = result.energy_available_wh / result.electrical_hover_power_w * 60.0
    assert result.estimated_flight_time_min == pytest.approx(expected_min, rel=1e-6)
    assert 5.0 < result.estimated_flight_time_min < 120.0


def test_requirements_reject_impossible_missions():
    with pytest.raises(SizingError):
        Requirements(payload_kg=0.0)
    with pytest.raises(SizingError):
        Requirements(thrust_to_weight_min=0.5)
