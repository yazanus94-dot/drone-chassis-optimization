"""Generate docs/design_report.pdf: a multi-page summary of the design study."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path(__file__).resolve().parents[1]

PAGES: list[tuple[str, list[str]]] = []

opt = json.loads((ROOT / "optimization" / "optimization_summary.json").read_text(encoding="utf-8"))
bat = json.loads((ROOT / "results" / "battery" / "battery_results.json").read_text(encoding="utf-8"))
st = json.loads((ROOT / "results" / "structural" / "structural_results.json").read_text(encoding="utf-8"))
cad = json.loads((ROOT / "results" / "cad" / "cad_mass.json").read_text(encoding="utf-8"))
cfd = json.loads((ROOT / "results" / "cfd" / "cfd_results.json").read_text(encoding="utf-8"))

PAGES.append(("Electric Drone Chassis Design Report", [
    "Multi-physics design study: sizing, CAD, meshing, structural, aerodynamics, battery, optimization.",
    "",
    f"Baseline design (aluminium, 225 x 16 x 1.5 mm arms): mass {opt['baseline']['total_mass_kg']:.3f} kg,",
    f"SF {opt['baseline']['safety_factor']:.1f}, f1 {opt['baseline']['first_frequency_hz']:.0f} Hz (inside motor band), flight {opt['baseline']['flight_time_min']:.1f} min.",
    "",
    f"Selected design ({opt['best']['material']}, {opt['best']['arm_length_mm']:.0f} x {opt['best']['outer_diameter_mm']:.1f} x {opt['best']['wall_thickness_mm']:.2f} mm arms):",
    f"mass {opt['best']['total_mass_kg']:.3f} kg, SF {opt['best']['safety_factor']:.1f}, f1 {opt['best']['first_frequency_hz']:.0f} Hz (band-separated), flight {opt['best']['flight_time_min']:.1f} min.",
    "",
    f"Improvement: {opt['comparison']['mass_saved_pct']:.1f}% lighter, {opt['comparison']['flight_time_gain_pct']:.1f}% more flight time.",
]))

PAGES.append(("Structural verification", [
    f"Beam FEM vs theory: mesh convergence error {st['beam_model']['mesh_convergence']['rel_error'][-1] * 100:.2f}% at 64 elements.",
    f"Max root stress (max-thrust case): {st['beam_model']['max_stress_mpa']:.1f} MPa, safety factor {st['beam_model']['safety_factor']:.1f}.",
    f"Tip deflection at hover: {st['beam_model']['tip_deflection_mm']:.3f} mm.",
    f"First bending mode: {st['beam_model']['first_frequency_hz']:.0f} Hz vs motor band 100-300 Hz (separated below).",
    "CalculiX decks generated: hover.inp, max_thrust.inp, landing.inp, modal.inp.",
]))

PAGES.append(("Aerodynamic verification", [
    "Drag model: quadratic law Cd 1.05 on projected frontal area.",
    "Sphere correlation validation (Clift-Grace-Weber):",
] + [f"  Re {v['re']:.0e}: predicted {v['correlation_cd']:.3f}, published {v['published_cd']:.3f}, error {v['rel_error'] * 100:.1f}%"
     for v in cfd["sphere_validation"]] + [
    f"Cylinder validation max error: {max(v['rel_error'] for v in cfd['cylinder_validation']) * 100:.1f}%.",
    f"OpenFOAM case: {len(cfd['openfoam_case_files'])} files, k-omega SST with actuator-disk fvOptions (requires a Linux solver for execution).",
]))

PAGES.append(("Battery verification (PyBaMM SPMe)", [
    f"Cell model: SPMe (Chen2020), 5 Ah 18650 x 6S, load = equivalent constant current at {bat['hover_power_w']:.1f} W.",
    f"Flight time at 25 C: {bat['flight_time_min_25c']:.1f} min (target 20 min, documented shortfall).",
] + [f"  {v['ambient_temp_c']} C: {v['flight_time_min']:.1f} min" for v in bat["temperature_sweep"]] + [
    "Cold cells discharge faster (higher internal resistance), consistent with physics.",
]))

PAGES.append(("Optimization study", [
    "Latin hypercube, 24 designs over arm length / diameter / wall / material, seed 20261113.",
    f"{opt['n_feasible']} of 24 designs feasible (SF >= 2 and modal band separation).",
    "Pareto front on (mass, flight time) extracted exactly; selected knee point shown above.",
    "Carbon designs fail the modal band constraint by being resonant; aluminium is feasible below the band.",
]))


def main() -> int:
    out = ROOT / "docs" / "design_report.pdf"
    with PdfPages(out) as pdf:
        for title, lines in PAGES:
            fig, ax = plt.subplots(figsize=(8.27, 11.69))
            ax.axis("off")
            ax.text(0.5, 0.92, title, ha="center", fontsize=16, weight="bold")
            body = "\n".join(lines)
            ax.text(0.08, 0.82, body, va="top", fontsize=11, family="monospace")
            pdf.savefig(fig)
            plt.close(fig)
    print(f"report written: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
