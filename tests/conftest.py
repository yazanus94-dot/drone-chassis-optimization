"""Shared pytest configuration: src layout import path and common fixtures."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from drone_chassis.core import ArmSection, Requirements  # noqa: E402
from drone_chassis.services import DesignPoint  # noqa: E402


@pytest.fixture
def requirements() -> Requirements:
    return Requirements()


@pytest.fixture
def baseline_design() -> DesignPoint:
    return DesignPoint(arm_length_mm=225.0, outer_diameter_mm=16.0, wall_thickness_mm=1.5, material="alu6061")


@pytest.fixture
def carbon_arm() -> ArmSection:
    return ArmSection("carbon", 16.0, 1.5, 225.0)
