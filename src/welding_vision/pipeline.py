from __future__ import annotations

from dataclasses import dataclass
import math
import random

from .config import PipelineConfig
from .geometry import (
    SegmentSupport,
    add,
    angle_between,
    centroid,
    distance,
    dot,
    extract_planes,
    line_from_plane_intersection,
    normalize,
    point_to_line_distance,
    point_to_plane_distance,
    project_to_line,
    scale,
    subtract,
)
from .models import PlaneModel, Point3D, SeamCandidate


@dataclass
class PipelineResult:
    workpiece_points: list[Point3D]
    planes: list[PlaneModel]
    seam_candidates: list[SeamCandidate]
    final_seams: list[SeamCandidate]
    metrics: dict[str, int]

    def to_dict(self) -> dict[str, object]:
        return {
            "metrics": self.metrics,
            "workpiece_points": [point.to_dict() for point in self.workpiece_points],
            "planes": [plane.to_dict() for plane in self.planes],
            "seam_candidates": [candidate.to_dict() for candidate in self.seam_candidates],
            "final_seams": [candidate.to_dict() for candidate in self.final_seams],
        }


class PointCloudPipeline:
    def __init__(self, config: PipelineConfig, seed: int = 7) -> None:
        self.config = config
        self.rng = random.Random(seed)

    def run(self, raw_points: list[Point3D], background_points: list[Point3D] | None = None) -> PipelineResult:
        metrics: dict[str, int] = {"raw_points": len(raw_points)}

        roi_points = self._crop_roi(raw_points)
        metrics["after_roi"] = len(roi_points)

        downsampled = self._voxel_downsample(roi_points)
        metrics["after_downsample"] = len(downsampled)

        filtered = self._remove_radius_outliers(downsampled)
        metrics["after_outlier_filter"] = len(filtered)

        background_filtered = self._subtract_background(filtered, background_points or [])
        metrics["after_background_subtraction"] = len(background_filtered)

        environment_removed = self._remove_environment_planes(background_filtered)
        metrics["after_environment_plane_removal"] = len(environment_removed)

        workpiece_points = self._select_workpiece_points(environment_removed)
        metrics["after_clustering"] = len(workpiece_points)

        workpiece_planes = extract_planes(
            workpiece_points,
            threshold=self.config.plane_distance_threshold,
            iterations=self.config.plane_ransac_iterations,
            min_inliers=self.config.plane_extraction_min_inliers,
            max_planes=self.config.max_workpiece_planes,
            prefix="workpiece-plane",
            rng=self.rng,
        )
        metrics["workpiece_plane_count"] = len(workpiece_planes)

        candidates = self._extract_seam_candidates(workpiece_points, workpiece_planes)
        metrics["seam_candidate_count"] = len(candidates)

        final_seams = [
            candidate
            for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True)
            if candidate.confidence >= self.config.final_score_threshold
        ]
        metrics["final_seam_count"] = len(final_seams)

        return PipelineResult(
            workpiece_points=workpiece_points,
            planes=workpiece_planes,
            seam_candidates=candidates,
            final_seams=final_seams,
            metrics=metrics,
        )

    def _crop_roi(self, points: list[Point3D]) -> list[Point3D]:
        return [point for point in points if self.config.roi.contains(point.to_tuple())]

    def _voxel_downsample(self, points: list[Point3D]) -> list[Point3D]:
        if self.config.voxel_size <= 0:
            return list(points)

        voxels: dict[tuple[int, int, int], Point3D] = {}
        for point in points:
            key = (
                math.floor(point.x / self.config.voxel_size),
                math.floor(point.y / self.config.voxel_size),
                math.floor(point.z / self.config.voxel_size),
            )
            voxels.setdefault(key, point)
        return list(voxels.values())

    def _remove_radius_outliers(self, points: list[Point3D]) -> list[Point3D]:
        if not points:
            return []
        index = _SpatialIndex(points, self.config.outlier_radius)
        kept = []
        for point in points:
            neighbors = index.query(point.to_tuple(), self.config.outlier_radius)
            if len(neighbors) - 1 >= self.config.outlier_min_neighbors:
                kept.append(point)
        return kept

    def _subtract_background(self, points: list[Point3D], background_points: list[Point3D]) -> list[Point3D]:
        if not background_points:
            return list(points)
        index = _SpatialIndex(background_points, self.config.background_tolerance)
        kept = []
        for point in points:
            if not index.query(point.to_tuple(), self.config.background_tolerance):
                kept.append(point)
        return kept

    def _remove_environment_planes(self, points: list[Point3D]) -> list[Point3D]:
        if len(points) < self.config.environment_plane_min_inliers:
            return list(points)

        remaining = list(points)
        removed_any = False
        for plane_index in range(self.config.max_environment_planes):
            plane = extract_planes(
                remaining,
                threshold=self.config.plane_distance_threshold,
                iterations=self.config.plane_ransac_iterations,
                min_inliers=self.config.environment_plane_min_inliers,
                max_planes=1,
                prefix=f"env-plane-{plane_index}",
                rng=self.rng,
            )
            if not plane:
                break
            current = plane[0]
            if not self._looks_like_environment_plane(current):
                break

            removed_any = True
            inlier_set = set(current.inlier_indices)
            remaining = [point for index, point in enumerate(remaining) if index not in inlier_set]

        return remaining if removed_any else points

    def _looks_like_environment_plane(self, plane: PlaneModel) -> bool:
        span_x = plane.bounds["x"][1] - plane.bounds["x"][0]
        span_y = plane.bounds["y"][1] - plane.bounds["y"][0]
        span_z = plane.bounds["z"][1] - plane.bounds["z"][0]
        max_span = max(span_x, span_y, span_z)
        if max_span < self.config.environment_plane_min_span:
            return False

        cx, cy, cz = plane.centroid
        margin = self.config.environment_boundary_margin
        roi = self.config.roi
        near_floor = abs(cz - roi.min_z) <= margin
        near_ceiling = abs(cz - roi.max_z) <= margin
        near_x_boundary = abs(cx - roi.min_x) <= margin or abs(cx - roi.max_x) <= margin
        near_y_boundary = abs(cy - roi.min_y) <= margin or abs(cy - roi.max_y) <= margin
        return near_floor or near_ceiling or near_x_boundary or near_y_boundary

    def _select_workpiece_points(self, points: list[Point3D]) -> list[Point3D]:
        clusters = self._euclidean_clusters(points)
        if not clusters:
            return []

        center = self.config.roi.center()
        scored = []
        for cluster in clusters:
            cluster_center = centroid(cluster)
            center_distance = distance(cluster_center, center)
            score = len(cluster) - (center_distance * 10.0)
            scored.append((score, cluster))

        scored.sort(key=lambda item: item[0], reverse=True)
        selected = scored[: self.config.cluster_keep_top_k]
        merged: list[Point3D] = []
        for _, cluster in selected:
            merged.extend(cluster)
        return merged

    def _euclidean_clusters(self, points: list[Point3D]) -> list[list[Point3D]]:
        if not points:
            return []

        index = _SpatialIndex(points, self.config.cluster_tolerance)
        visited = set()
        clusters: list[list[Point3D]] = []

        for start_index, point in enumerate(points):
            if start_index in visited:
                continue

            queue = [start_index]
            visited.add(start_index)
            cluster_indices = []

            while queue:
                current_index = queue.pop()
                cluster_indices.append(current_index)
                current_point = points[current_index]
                for neighbor_index in index.query_indices(current_point.to_tuple(), self.config.cluster_tolerance):
                    if neighbor_index not in visited:
                        visited.add(neighbor_index)
                        queue.append(neighbor_index)

            if len(cluster_indices) >= self.config.cluster_min_points:
                clusters.append([points[index_] for index_ in cluster_indices])

        clusters.sort(key=len, reverse=True)
        return clusters

    def _extract_seam_candidates(
        self,
        workpiece_points: list[Point3D],
        planes: list[PlaneModel],
    ) -> list[SeamCandidate]:
        if len(workpiece_points) < self.config.seam_min_support_points:
            return []

        candidates: list[SeamCandidate] = []
        for first in range(len(planes)):
            for second in range(first + 1, len(planes)):
                plane_a = planes[first]
                plane_b = planes[second]
                angle = angle_between(plane_a.normal, plane_b.normal)
                if angle < 20.0 or abs(angle - 180.0) < 5.0:
                    continue

                line = line_from_plane_intersection(plane_a, plane_b)
                if line is None:
                    continue

                line_point, line_direction = line
                support = self._collect_support_segments(workpiece_points, line_point, line_direction)
                for segment_index, segment in enumerate(support):
                    if segment.length < self.config.seam_min_length:
                        continue
                    if len(segment.support_points) < self.config.seam_min_support_points:
                        continue

                    candidate = self._build_candidate(
                        plane_a=plane_a,
                        plane_b=plane_b,
                        segment=segment,
                        line_direction=line_direction,
                        candidate_id=f"seam-{plane_a.id}-{plane_b.id}-{segment_index}",
                    )
                    candidates.append(candidate)

        candidates.sort(key=lambda item: item.confidence, reverse=True)
        return candidates

    def _collect_support_segments(
        self,
        points: list[Point3D],
        line_point: tuple[float, float, float],
        line_direction: tuple[float, float, float],
    ) -> list[SegmentSupport]:
        nearby = [
            point
            for point in points
            if point_to_line_distance(point.to_tuple(), line_point, line_direction)
            <= self.config.seam_line_distance_threshold
        ]
        if not nearby:
            return []

        projected = sorted(
            ((project_to_line(point.to_tuple(), line_point, line_direction), point) for point in nearby),
            key=lambda item: item[0],
        )

        segments: list[SegmentSupport] = []
        current_points = [projected[0][1]]
        previous_t = projected[0][0]
        current_start_t = previous_t
        max_gap = 0.0

        for t_value, point in projected[1:]:
            gap = t_value - previous_t
            if gap > self.config.seam_segment_gap:
                start = add(line_point, scale(line_direction, current_start_t))
                end = add(line_point, scale(line_direction, previous_t))
                segments.append(
                    SegmentSupport(start=start, end=end, support_points=current_points, max_gap=max_gap)
                )
                current_points = [point]
                current_start_t = t_value
                max_gap = 0.0
            else:
                current_points.append(point)
                max_gap = max(max_gap, gap)
            previous_t = t_value

        start = add(line_point, scale(line_direction, current_start_t))
        end = add(line_point, scale(line_direction, previous_t))
        segments.append(SegmentSupport(start=start, end=end, support_points=current_points, max_gap=max_gap))
        return segments

    def _build_candidate(
        self,
        plane_a: PlaneModel,
        plane_b: PlaneModel,
        segment: SegmentSupport,
        line_direction: tuple[float, float, float],
        candidate_id: str,
    ) -> SeamCandidate:
        angle = angle_between(plane_a.normal, plane_b.normal)
        geometry_score = self._geometry_score(angle, len(segment.support_points), segment.length)
        continuity_score = self._continuity_score(segment)
        position_score = self._position_score(segment)
        accessibility_score = self._accessibility_score(plane_a, plane_b)
        confidence = (
            geometry_score * 0.35
            + continuity_score * 0.25
            + position_score * 0.20
            + accessibility_score * 0.20
        )
        return SeamCandidate(
            id=candidate_id,
            start=Point3D(*segment.start),
            end=Point3D(*segment.end),
            direction=normalize(line_direction),
            length=segment.length,
            support_count=len(segment.support_points),
            source_plane_ids=(plane_a.id, plane_b.id),
            confidence=round(confidence, 4),
            score_breakdown={
                "geometry": round(geometry_score, 4),
                "continuity": round(continuity_score, 4),
                "position": round(position_score, 4),
                "accessibility": round(accessibility_score, 4),
            },
        )

    def _geometry_score(self, angle: float, support_count: int, length: float) -> float:
        fillet_score = max(0.0, 1.0 - (abs(angle - 90.0) / 90.0))
        butt_score = max(0.0, 1.0 - (abs(angle - 180.0) / 40.0))
        angle_score = max(fillet_score, butt_score)
        support_score = min(1.0, support_count / max(self.config.seam_min_support_points * 2, 1))
        length_score = min(1.0, length / max(self.config.seam_min_length * 2, 1e-6))
        return (angle_score * 0.5) + (support_score * 0.3) + (length_score * 0.2)

    def _continuity_score(self, segment: SegmentSupport) -> float:
        if segment.length <= 1e-6:
            return 0.0
        density = min(1.0, len(segment.support_points) / max(segment.length / self.config.seam_segment_gap, 1.0))
        gap_penalty = min(1.0, segment.max_gap / max(self.config.seam_segment_gap, 1e-6))
        return max(0.0, (density * 0.7) + ((1.0 - gap_penalty) * 0.3))

    def _position_score(self, segment: SegmentSupport) -> float:
        if self.config.expected_height_range is None:
            return 0.6
        z_low, z_high = self.config.expected_height_range
        mean_z = (segment.start[2] + segment.end[2]) / 2.0
        if z_low <= mean_z <= z_high:
            return 1.0
        distance_to_band = min(abs(mean_z - z_low), abs(mean_z - z_high))
        return max(0.0, 1.0 - (distance_to_band / max((z_high - z_low), 1e-6)))

    def _accessibility_score(self, plane_a: PlaneModel, plane_b: PlaneModel) -> float:
        if self.config.preferred_approach_vector is None:
            return 0.6
        preferred = normalize(self.config.preferred_approach_vector)
        seam_normal = normalize(add(plane_a.normal, plane_b.normal))
        if seam_normal == (0.0, 0.0, 0.0):
            seam_normal = plane_a.normal
        score = max(0.0, dot(seam_normal, preferred))
        return min(1.0, score)


class _SpatialIndex:
    def __init__(self, points: list[Point3D], cell_size: float) -> None:
        self.points = points
        self.cell_size = max(cell_size, 1e-6)
        self.cells: dict[tuple[int, int, int], list[int]] = {}
        for index, point in enumerate(points):
            self.cells.setdefault(self._key(point.to_tuple()), []).append(index)

    def _key(self, point: tuple[float, float, float]) -> tuple[int, int, int]:
        return (
            math.floor(point[0] / self.cell_size),
            math.floor(point[1] / self.cell_size),
            math.floor(point[2] / self.cell_size),
        )

    def _candidate_indices(self, point: tuple[float, float, float]) -> list[int]:
        base = self._key(point)
        indices = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    indices.extend(self.cells.get((base[0] + dx, base[1] + dy, base[2] + dz), []))
        return indices

    def query(self, point: tuple[float, float, float], radius: float) -> list[Point3D]:
        squared_radius = radius * radius
        matches = []
        for index in self._candidate_indices(point):
            candidate = self.points[index]
            delta = subtract(candidate.to_tuple(), point)
            if dot(delta, delta) <= squared_radius:
                matches.append(candidate)
        return matches

    def query_indices(self, point: tuple[float, float, float], radius: float) -> list[int]:
        squared_radius = radius * radius
        matches = []
        for index in self._candidate_indices(point):
            candidate = self.points[index]
            delta = subtract(candidate.to_tuple(), point)
            if dot(delta, delta) <= squared_radius:
                matches.append(index)
        return matches

