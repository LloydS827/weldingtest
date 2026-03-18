from __future__ import annotations

import unittest

from welding_vision.config import PipelineConfig, ROIBox
from welding_vision.models import Point3D
from welding_vision.pipeline import PointCloudPipeline


def _frange(start: float, stop: float, step: float) -> list[float]:
    values = []
    current = start
    while current <= stop + 1e-9:
        values.append(round(current, 4))
        current += step
    return values


def make_scene(include_workpiece: bool = True) -> tuple[list[Point3D], list[Point3D]]:
    background = []

    for x in _frange(-0.3, 1.3, 0.1):
        for y in _frange(-0.3, 1.3, 0.1):
            background.append(Point3D(x, y, -0.35))

    for y in _frange(-0.2, 1.2, 0.12):
        for z in _frange(-0.2, 1.0, 0.12):
            background.append(Point3D(1.28, y, z))

    scene = list(background)
    if include_workpiece:
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


def make_config() -> PipelineConfig:
    return PipelineConfig(
        roi=ROIBox(min_x=-0.3, max_x=1.35, min_y=-0.3, max_y=1.35, min_z=-0.4, max_z=1.1),
        voxel_size=0.04,
        outlier_radius=0.11,
        outlier_min_neighbors=2,
        background_tolerance=0.045,
        plane_distance_threshold=0.025,
        plane_ransac_iterations=220,
        environment_plane_min_inliers=25,
        environment_plane_min_span=0.9,
        environment_boundary_margin=0.08,
        max_environment_planes=2,
        cluster_tolerance=0.1,
        cluster_min_points=20,
        cluster_keep_top_k=2,
        plane_extraction_min_inliers=22,
        max_workpiece_planes=4,
        seam_line_distance_threshold=0.05,
        seam_segment_gap=0.12,
        seam_min_support_points=10,
        seam_min_length=0.4,
        expected_height_range=(-0.05, 0.85),
        preferred_approach_vector=(1.0, 0.0, 1.0),
        final_score_threshold=0.55,
    )


class PipelineTests(unittest.TestCase):
    def test_background_subtraction_keeps_workpiece(self) -> None:
        scene, background = make_scene(include_workpiece=True)
        pipeline = PointCloudPipeline(make_config(), seed=11)
        result = pipeline.run(scene, background_points=background)

        self.assertGreater(result.metrics["after_background_subtraction"], 150)
        self.assertLess(result.metrics["after_background_subtraction"], result.metrics["after_outlier_filter"])
        self.assertGreater(result.metrics["after_clustering"], 120)

    def test_pipeline_detects_primary_seam(self) -> None:
        scene, background = make_scene(include_workpiece=True)
        pipeline = PointCloudPipeline(make_config(), seed=11)
        result = pipeline.run(scene, background_points=background)

        self.assertGreaterEqual(result.metrics["final_seam_count"], 1)
        best = result.final_seams[0]
        self.assertGreater(best.length, 0.7)
        self.assertGreater(best.confidence, 0.6)
        self.assertLess(abs(best.start.x), 0.08)
        self.assertLess(abs(best.end.x), 0.08)

    def test_empty_scene_has_no_seam(self) -> None:
        scene, background = make_scene(include_workpiece=False)
        pipeline = PointCloudPipeline(make_config(), seed=11)
        result = pipeline.run(scene, background_points=background)

        self.assertEqual(result.metrics["after_clustering"], 0)
        self.assertEqual(result.metrics["final_seam_count"], 0)


if __name__ == "__main__":
    unittest.main()
