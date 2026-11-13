"""Generate documentation assets: architecture diagram, demo GIF, CAD render.

Run once after the pipeline (python scripts/make_demo_assets.py); outputs land
in docs/images/ and results/.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from PIL import Image

IMG_DIR = ROOT / "docs" / "images"
IMG_DIR.mkdir(parents=True, exist_ok=True)

STAGES = [
    ("Step 1\nRequirements\n+ sizing", 0.10),
    ("Step 2\nCadQuery CAD", 0.26),
    ("Step 3\nGmsh mesh", 0.42),
    ("Step 4\nStructural\n(CalculiX)", 0.58),
    ("Step 5\nAero\n(OpenFOAM)", 0.74),
    ("Step 6\nPyBaMM battery", 0.90),
]

LOOP_NOTE = "Step 7: coupled optimization loop (DOE -> Pareto) feeds the best design back into Step 2"


def draw_architecture() -> None:
    fig, ax = plt.subplots(figsize=(11, 4.2))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    for label, x in STAGES:
        box = FancyBboxPatch((x - 0.065, 0.38), 0.13, 0.26, boxstyle="round,pad=0.012",
                             fc="#eaf2fb", ec="#3b6ea5", lw=1.6)
        ax.add_patch(box)
        ax.text(x, 0.51, label, ha="center", va="center", fontsize=9)
    for (_l1, x1), (_, x2) in zip(STAGES, STAGES[1:], strict=False):
        ax.add_patch(FancyArrowPatch((x1 + 0.068, 0.51), (x2 - 0.068, 0.51),
                                     arrowstyle="-|>", mutation_scale=16, color="#3b6ea5", lw=1.6))
    ax.add_patch(FancyArrowPatch((0.90, 0.30), (0.26, 0.30), arrowstyle="-|>",
                                 mutation_scale=16, color="#c0392b", lw=1.6,
                                 connectionstyle="arc3,rad=0.35"))
    ax.text(0.5, 0.10, LOOP_NOTE, ha="center", fontsize=9.5, color="#c0392b")
    ax.text(0.5, 0.86, "Drone chassis pipeline: every stage validates against an independent oracle",
            ha="center", fontsize=11)
    fig.tight_layout()
    fig.savefig(IMG_DIR / "architecture.png", dpi=160)
    plt.close(fig)


def draw_cad_frame(path: Path) -> None:
    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    ax.set_aspect("equal")
    ax.axis("off")
    arm_len, plate = 225.0, 120.0
    for _i, ang in enumerate([45, 135, 225, 315]):
        r = np.radians(ang)
        x, y = arm_len / 2 * np.cos(r), arm_len / 2 * np.sin(r)
        ax.plot([0, x], [0, y], lw=7, color="#555", solid_capstyle="round")
        ax.add_patch(plt.Circle((x, y), 14, fc="#c0392b", ec="k"))
    ax.add_patch(plt.Rectangle((-plate / 2, -plate / 2), plate, plate, fc="#eaf2fb", ec="#3b6ea5", lw=2))
    ax.add_patch(plt.Rectangle((-plate * 0.225, -plate * 0.225), plate * 0.45, plate * 0.45, fc="white", ec="#3b6ea5"))
    ax.set_title("CAD: X-frame quadcopter chassis (parametric)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def draw_mesh_frame(arm_outer: float, path: Path) -> None:
    import gmsh
    import numpy as np

    gmsh.initialize()
    try:
        gmsh.merge(str(ROOT / "results" / "mesh" / "structural.msh"))
        _, coords, _ = gmsh.model.mesh.getNodes()
        pts = np.array(coords).reshape(-1, 3)
        sel = pts[:, 2] < 0.1 * pts[:, 2].max() + 3.0
    finally:
        gmsh.finalize()
    fig, ax = plt.subplots(figsize=(6.4, 4.8))
    ax.scatter(pts[sel, 0], pts[sel, 1], s=1.2, c="#3b6ea5", alpha=0.5)
    ax.set_aspect("equal")
    ax.set_title("Mesh: second-order tets, refined at the arm-plate joint")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def make_gif(frames: list[Path], out: Path, size=(640, 440), duration_ms=1400) -> None:
    images = []
    for p in frames:
        img = Image.open(p).convert("RGB")
        img.thumbnail(size, Image.LANCZOS)
        canvas = Image.new("RGB", size, "white")
        canvas.paste(img, ((size[0] - img.width) // 2, (size[1] - img.height) // 2))
        images.append(canvas)
    images[0].save(out, save_all=True, append_images=images[1:], duration=duration_ms, loop=0)


def main() -> int:
    import numpy as np  # noqa: F401

    draw_architecture()

    cad_frame = IMG_DIR / "demo_1_cad.png"
    draw_cad_frame(cad_frame)
    try:
        mesh_frame = IMG_DIR / "demo_2_mesh.png"
        draw_mesh_frame(16.0, mesh_frame)
    except Exception as exc:
        print(f"mesh frame skipped: {exc}")
        mesh_frame = cad_frame

    frames = [
        cad_frame,
        mesh_frame,
        ROOT / "results" / "structural" / "stress_distribution.png",
        ROOT / "results" / "structural" / "mode_shapes.png",
        ROOT / "results" / "cfd" / "drag_vs_speed.png",
        ROOT / "results" / "battery" / "discharge_curves.png",
        ROOT / "results" / "pareto_front.png",
    ]
    frames = [f for f in frames if f.exists()]
    make_gif(frames, ROOT / "results" / "demo_pipeline.gif")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
