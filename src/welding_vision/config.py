from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path


@dataclass(frozen=True)
class ROIBox:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    def contains(self, point: tuple[float, float, float]) -> bool:
        x, y, z = point
        return (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and self.min_z <= z <= self.max_z
        )

    def center(self) -> tuple[float, float, float]:
        return (
            (self.min_x + self.max_x) / 2.0,
            (self.min_y + self.max_y) / 2.0,
            (self.min_z + self.max_z) / 2.0,
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "min_x": self.min_x,
            "max_x": self.max_x,
            "min_y": self.min_y,
            "max_y": self.max_y,
            "min_z": self.min_z,
            "max_z": self.max_z,
        }


@dataclass(frozen=True)
class PipelineConfig:
    roi: ROIBox
    voxel_size: float = 0.04
    outlier_radius: float = 0.10
    outlier_min_neighbors: int = 2
    background_tolerance: float = 0.05
    plane_distance_threshold: float = 0.025
    plane_ransac_iterations: int = 180
    environment_plane_min_inliers: int = 30
    environment_plane_min_span: float = 0.8
    environment_boundary_margin: float = 0.08
    max_environment_planes: int = 3
    cluster_tolerance: float = 0.10
    cluster_min_points: int = 25
    cluster_keep_top_k: int = 2
    plane_extraction_min_inliers: int = 25
    max_workpiece_planes: int = 6
    seam_line_distance_threshold: float = 0.05
    seam_segment_gap: float = 0.12
    seam_min_support_points: int = 12
    seam_min_length: float = 0.35
    expected_height_range: tuple[float, float] | None = None
    preferred_approach_vector: tuple[float, float, float] | None = None
    final_score_threshold: float = 0.58

    @classmethod
    def from_dict(cls, payload: dict[str, object]) -> "PipelineConfig":
        roi_payload = payload.get("roi")
        if not isinstance(roi_payload, dict):
            raise ValueError("Config must include roi bounds.")

        expected_height_range = payload.get("expected_height_range")
        if isinstance(expected_height_range, list):
            expected_height_range = tuple(float(v) for v in expected_height_range)
        elif expected_height_range is not None:
            expected_height_range = tuple(expected_height_range)

        preferred_approach_vector = payload.get("preferred_approach_vector")
        if isinstance(preferred_approach_vector, list):
            preferred_approach_vector = tuple(float(v) for v in preferred_approach_vector)
        elif preferred_approach_vector is not None:
            preferred_approach_vector = tuple(preferred_approach_vector)

        return cls(
            roi=ROIBox(**{k: float(v) for k, v in roi_payload.items()}),
            voxel_size=float(payload.get("voxel_size", 0.04)),
            outlier_radius=float(payload.get("outlier_radius", 0.10)),
            outlier_min_neighbors=int(payload.get("outlier_min_neighbors", 2)),
            background_tolerance=float(payload.get("background_tolerance", 0.05)),
            plane_distance_threshold=float(payload.get("plane_distance_threshold", 0.025)),
            plane_ransac_iterations=int(payload.get("plane_ransac_iterations", 180)),
            environment_plane_min_inliers=int(payload.get("environment_plane_min_inliers", 30)),
            environment_plane_min_span=float(payload.get("environment_plane_min_span", 0.8)),
            environment_boundary_margin=float(payload.get("environment_boundary_margin", 0.08)),
            max_environment_planes=int(payload.get("max_environment_planes", 3)),
            cluster_tolerance=float(payload.get("cluster_tolerance", 0.10)),
            cluster_min_points=int(payload.get("cluster_min_points", 25)),
            cluster_keep_top_k=int(payload.get("cluster_keep_top_k", 2)),
            plane_extraction_min_inliers=int(payload.get("plane_extraction_min_inliers", 25)),
            max_workpiece_planes=int(payload.get("max_workpiece_planes", 6)),
            seam_line_distance_threshold=float(payload.get("seam_line_distance_threshold", 0.05)),
            seam_segment_gap=float(payload.get("seam_segment_gap", 0.12)),
            seam_min_support_points=int(payload.get("seam_min_support_points", 12)),
            seam_min_length=float(payload.get("seam_min_length", 0.35)),
            expected_height_range=expected_height_range,
            preferred_approach_vector=preferred_approach_vector,
            final_score_threshold=float(payload.get("final_score_threshold", 0.58)),
        )

    @classmethod
    def from_json_file(cls, path: str | Path) -> "PipelineConfig":
        return cls.from_dict(json.loads(Path(path).read_text()))

