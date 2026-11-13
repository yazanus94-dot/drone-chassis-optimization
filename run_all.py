"""Full pipeline runner: CAD -> mesh -> structural -> CFD -> battery -> optimization.

Executes every stage as a subprocess with the same design parameters so the
generated artefacts (STEP/STL, meshes, structural and CFD results, battery
curves, Pareto analysis) stay consistent, then assembles the demo GIF.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PY = sys.executable

STAGES = [
    ("cad", [str(ROOT / "cad" / "cad_model.py")]),
    ("mesh", [str(ROOT / "mesh" / "mesh.py")]),
    ("structural", [str(ROOT / "simulation" / "structural" / "run_structural.py")]),
    ("cfd", [str(ROOT / "simulation" / "cfd" / "run_cfd.py")]),
    ("battery", [str(ROOT / "battery" / "battery_sim.py")]),
    ("optimization", [str(ROOT / "optimization" / "optimize.py")]),
]


def run_all(out: str = "results") -> int:
    failures = []
    for name, script in STAGES:
        start = time.time()
        print(f"== stage: {name} ==")
        proc = subprocess.run([PY, *script, "--out", out], cwd=str(ROOT))
        elapsed = time.time() - start
        print(f"== stage {name} {'ok' if proc.returncode == 0 else 'FAILED'} in {elapsed:.1f}s ==")
        if proc.returncode != 0:
            failures.append(name)
    if failures:
        print(f"pipeline finished with failures: {failures}")
        return 1
    print("pipeline finished successfully")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="results")
    args = parser.parse_args()
    return run_all(args.out)


if __name__ == "__main__":
    raise SystemExit(main())
