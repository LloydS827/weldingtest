from __future__ import annotations

import argparse
from pathlib import Path

from .io import save_json
from .models import Point3D


def _frange(start: float, stop: float, step: float) -> list[float]:
    values = []
    current = start
    while current <= stop + 1e-9:
        values.append(round(current, 4))
        current += step
    return values


def build_shipyard_demo_scene() -> tuple[list[Point3D], list[Point3D]]:
    background = []

    for x in _frange(-0.3, 1.3, 0.1):
        for y in _frange(-0.3, 1.3, 0.1):
            background.append(Point3D(x, y, -0.35))

    for y in _frange(-0.2, 1.2, 0.12):
        for z in _frange(-0.2, 1.0, 0.12):
            background.append(Point3D(1.28, y, z))

    scene = list(background)

    for x in _frange(0.0, 0.75, 0.06):
        for y in _frange(0.05, 1.0, 0.06):
            scene.append(Point3D(x, y, 0.0))

    for y in _frange(0.05, 1.0, 0.06):
        for z in _frange(0.0, 0.75, 0.06):
            scene.append(Point3D(0.0, y, z))

    for y in _frange(0.08, 0.95, 0.05):
        scene.append(Point3D(0.025, y, 0.02))
        scene.append(Point3D(0.018, y, 0.03))

    for dx in _frange(0.65, 0.9, 0.05):
        for dy in _frange(0.65, 0.9, 0.05):
            for dz in _frange(0.35, 0.6, 0.05):
                scene.append(Point3D(dx, dy, dz))

    return scene, background


def write_demo_files(output_dir: str | Path) -> tuple[Path, Path]:
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    scene, background = build_shipyard_demo_scene()

    scene_path = destination / "scene.json"
    background_path = destination / "background.json"
    save_json(scene_path, {"points": [point.to_dict() for point in scene]})
    save_json(background_path, {"points": [point.to_dict() for point in background]})
    return scene_path, background_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate demo point clouds for the welding vision pipeline.")
    parser.add_argument(
        "--output-dir",
        default="examples",
        help="Directory where scene.json and background.json will be created.",
    )
    args = parser.parse_args(argv)

    scene_path, background_path = write_demo_files(args.output_dir)
    print(f"scene: {scene_path}")
    print(f"background: {background_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
