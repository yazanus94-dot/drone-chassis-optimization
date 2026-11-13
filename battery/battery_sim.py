"""Step 6: battery and flight time with PyBaMM (SPMe).

Uses the hover power from Step 1 (corrected by the final mass) and the
forward-flight power from Step 5 (hover + drag). The 4S 5000 mAh Li-ion pack
is simulated per cell with the SPMe model and a constant-power discharge to
the 3.0 V/cell cut-off, at several ambient temperatures.

Outputs: discharge curves, and a flight_time(power_w, temperature) function.
A closed-form energy/power fallback is used when PyBaMM is unavailable, so the
pipeline and its tests stay runnable everywhere.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np

from drone_chassis.core import (
    Requirements,
)

NOMINAL_CAPACITY_AH = 5.0
NOMINAL_CAPACITY_AS = NOMINAL_CAPACITY_AH * 3600.0
CELL_CUTOFF_V = 3.0
AMBIENT_TEMPS_C = (-10.0, 0.0, 25.0, 40.0)

try:
    import pybamm
except ImportError:
    pybamm = None


def simulate_discharge(power_w_total: float, req: Requirements, ambient_temp_c: float = 25.0):
    """SPMe discharge with the pack load applied as a per-cell current profile.

    I = P / (n_cells * V_nom) per cell, per the brief's "power or current
    profile" option; the discharge runs to the 3.0 V/cell cut-off.
    Returns (times_s, voltages_v, average_power_w) or None.
    """
    if power_w_total <= 0:
        raise ValueError("power must be positive")
    if pybamm is None:
        return None
    current_a = power_w_total / req.pack_nominal_voltage
    model = pybamm.lithium_ion.SPMe()
    params = pybamm.ParameterValues("Chen2020")
    params["Lower voltage cut-off [V]"] = CELL_CUTOFF_V
    params["Ambient temperature [K]"] = ambient_temp_c + 273.15
    experiment = pybamm.Experiment([f"Discharge at {current_a:.3f} A until 3.0 V"])
    sim = pybamm.Simulation(model, parameter_values=params, experiment=experiment)
    solution = sim.solve()
    t = solution["Time [s]"].entries
    v = solution["Terminal voltage [V]"].entries
    i = solution["Current [A]"].entries
    avg_power = float(np.trapezoid(i * v, t) / t[-1]) * req.pack_cells_series
    return t, v, avg_power


def flight_time_fallback(power_w: float, req: Requirements, ambient_temp_c: float = 25.0) -> float:
    """Closed-form flight time from pack energy with a temperature derating."""
    temp_factor = 1.0
    if ambient_temp_c <= 0.0:
        temp_factor = 0.75
    elif ambient_temp_c < 15.0:
        temp_factor = 0.9
    usable_wh = req.pack_usable_wh * temp_factor
    return usable_wh / power_w * 3600.0


def flight_time(power_w: float, req: Requirements | None = None,
                ambient_temp_c: float = 25.0) -> float:
    """Flight time [s] at a given electrical power and ambient temperature."""
    if req is None:
        req = Requirements()
    if pybamm is not None:
        result = simulate_discharge(power_w, req, ambient_temp_c)
        if result is not None:
            return float(result[0][-1])
    return flight_time_fallback(power_w, req, ambient_temp_c)


def temperature_sweep(power_w: float, req: Requirements | None = None) -> list[dict]:
    if req is None:
        req = Requirements()
    rows = []
    for temp in AMBIENT_TEMPS_C:
        t = flight_time(power_w, req, temp)
        rows.append({"ambient_temp_c": temp, "flight_time_min": round(t / 60.0, 2)})
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="results")
    parser.add_argument("--hover-power-w", type=float, default=None,
                        help="electrical hover power; default recomputes from sizing")
    args = parser.parse_args()

    req = Requirements()
    from drone_chassis.core import first_pass_sizing
    from drone_chassis.services import DesignPoint

    baseline = DesignPoint(225.0, 16.0, 1.5, "carbon")
    power = args.hover_power_w
    if power is None:
        power = first_pass_sizing(req, baseline.arm).electrical_hover_power_w

    out_dir = ROOT / args.out / "battery"
    out_dir.mkdir(parents=True, exist_ok=True)

    sweep = temperature_sweep(power, req)
    direct = simulate_discharge(power, req, 25.0) if pybamm is not None else None
    curves = {}
    if direct is not None:
        t, v, avg_power = direct
        curves["time_s"] = [round(float(x), 1) for x in t[:: max(1, len(t) // 200)]]
        curves["voltage_v"] = [round(float(x), 4) for x in v[:: max(1, len(v) // 200)]]
        _plot_curves(curves, avg_power, out_dir / "discharge_curves.png")

    payload = {
        "model": "PyBaMM SPMe (Chen2020)" if pybamm is not None else "closed-form energy fallback",
        "hover_power_w": round(power, 2),
        "flight_time_min_25c": round(flight_time(power, req) / 60.0, 2),
        "temperature_sweep": sweep,
    }
    (out_dir / "battery_results.json").write_text(json.dumps(payload, indent=2))
    print(json.dumps(payload, indent=2))
    return 0


def _plot_curves(curves: dict, power: float, out_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4))
    t = np.asarray(curves["time_s"]) / 60.0
    ax.plot(t, curves["voltage_v"], lw=2)
    ax.axhline(CELL_CUTOFF_V, color="r", ls="--", label="cut-off 3.0 V/cell")
    ax.set(xlabel="time [min]", ylabel="cell voltage [V]", title=f"SPMe discharge at {power:.0f} W total")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
