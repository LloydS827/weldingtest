from __future__ import annotations

import csv
import json
from pathlib import Path

from .models import Point3D


def load_points(path: str | Path) -> list[Point3D]:
    source = Path(path)
    suffix = source.suffix.lower()
    if suffix == ".json":
        return _load_json_points(source)
    if suffix == ".csv":
        return _load_csv_points(source)
    raise ValueError(f"Unsupported point cloud format: {source.suffix}")


def save_json(path: str | Path, payload: dict[str, object]) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, indent=2))


def _load_json_points(path: Path) -> list[Point3D]:
    payload = json.loads(path.read_text())
    if isinstance(payload, dict):
        items = payload.get("points", [])
    elif isinstance(payload, list):
        items = payload
    else:
        raise ValueError("JSON point cloud must be an object with 'points' or a list.")

    points = []
    for item in items:
        points.append(Point3D(x=float(item["x"]), y=float(item["y"]), z=float(item["z"])))
    return points


def _load_csv_points(path: Path) -> list[Point3D]:
    points = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            points.append(Point3D(x=float(row["x"]), y=float(row["y"]), z=float(row["z"])))
    return points
