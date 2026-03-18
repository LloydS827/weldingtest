from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable

from .models import PlaneModel, Point3D


Vector = tuple[float, float, float]


def add(a: Vector, b: Vector) -> Vector:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def subtract(a: Vector, b: Vector) -> Vector:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(v: Vector, factor: float) -> Vector:
    return (v[0] * factor, v[1] * factor, v[2] * factor)


def dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector, b: Vector) -> Vector:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(v: Vector) -> float:
    return math.sqrt(dot(v, v))


def normalize(v: Vector) -> Vector:
    magnitude = norm(v)
    if magnitude == 0.0:
        return (0.0, 0.0, 0.0)
    return (v[0] / magnitude, v[1] / magnitude, v[2] / magnitude)


def distance(a: Vector, b: Vector) -> float:
    return norm(subtract(a, b))


def point_to_plane_distance(point: Vector, plane_normal: Vector, plane_offset: float) -> float:
    return abs(dot(plane_normal, point) + plane_offset)


def point_to_line_distance(point: Vector, line_point: Vector, line_direction: Vector) -> float:
    unit_direction = normalize(line_direction)
    if unit_direction == (0.0, 0.0, 0.0):
        return distance(point, line_point)
    offset = subtract(point, line_point)
    projection = scale(unit_direction, dot(offset, unit_direction))
    return norm(subtract(offset, projection))


def project_to_line(point: Vector, line_point: Vector, line_direction: Vector) -> float:
    unit_direction = normalize(line_direction)
    return dot(subtract(point, line_point), unit_direction)


def angle_between(a: Vector, b: Vector) -> float:
    unit_a = normalize(a)
    unit_b = normalize(b)
    cosine = max(-1.0, min(1.0, dot(unit_a, unit_b)))
    return math.degrees(math.acos(cosine))


def bounds_for_points(points: list[Point3D]) -> dict[str, tuple[float, float]]:
    xs = [point.x for point in points]
    ys = [point.y for point in points]
    zs = [point.z for point in points]
    return {
        "x": (min(xs), max(xs)),
        "y": (min(ys), max(ys)),
        "z": (min(zs), max(zs)),
    }


def centroid(points: Iterable[Point3D]) -> Vector:
    items = list(points)
    if not items:
        return (0.0, 0.0, 0.0)
    return (
        sum(point.x for point in items) / len(items),
        sum(point.y for point in items) / len(items),
        sum(point.z for point in items) / len(items),
    )


def plane_from_three_points(a: Point3D, b: Point3D, c: Point3D) -> tuple[Vector, float] | None:
    ab = subtract(b.to_tuple(), a.to_tuple())
    ac = subtract(c.to_tuple(), a.to_tuple())
    normal = cross(ab, ac)
    unit_normal = normalize(normal)
    if unit_normal == (0.0, 0.0, 0.0):
        return None
    offset = -dot(unit_normal, a.to_tuple())
    return unit_normal, offset


def fit_plane_ransac(
    points: list[Point3D],
    threshold: float,
    iterations: int,
    min_inliers: int,
    plane_id: str,
    rng: random.Random,
) -> PlaneModel | None:
    if len(points) < max(3, min_inliers):
        return None

    best_model: tuple[Vector, float] | None = None
    best_inliers: list[int] = []

    for _ in range(iterations):
        try:
            i1, i2, i3 = rng.sample(range(len(points)), 3)
        except ValueError:
            break

        plane = plane_from_three_points(points[i1], points[i2], points[i3])
        if plane is None:
            continue

        normal, offset = plane
        inliers = [
            index
            for index, point in enumerate(points)
            if point_to_plane_distance(point.to_tuple(), normal, offset) <= threshold
        ]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = (normal, offset)

    if best_model is None or len(best_inliers) < min_inliers:
        return None

    normal, offset = best_model
    inlier_points = [points[index] for index in best_inliers]
    return PlaneModel(
        id=plane_id,
        normal=normalize(normal),
        offset=offset,
        support_count=len(best_inliers),
        centroid=centroid(inlier_points),
        bounds=bounds_for_points(inlier_points),
        inlier_indices=best_inliers,
    )


def extract_planes(
    points: list[Point3D],
    threshold: float,
    iterations: int,
    min_inliers: int,
    max_planes: int,
    prefix: str,
    rng: random.Random,
) -> list[PlaneModel]:
    remaining = list(points)
    extracted: list[PlaneModel] = []

    for plane_index in range(max_planes):
        plane = fit_plane_ransac(
            remaining,
            threshold=threshold,
            iterations=iterations,
            min_inliers=min_inliers,
            plane_id=f"{prefix}-{plane_index}",
            rng=rng,
        )
        if plane is None:
            break

        extracted.append(plane)
        inlier_set = set(plane.inlier_indices)
        remaining = [point for index, point in enumerate(remaining) if index not in inlier_set]
        if len(remaining) < min_inliers:
            break

    return extracted


def line_from_plane_intersection(plane_a: PlaneModel, plane_b: PlaneModel) -> tuple[Vector, Vector] | None:
    direction = cross(plane_a.normal, plane_b.normal)
    denom = dot(direction, direction)
    if denom < 1e-8:
        return None

    c1 = -plane_a.offset
    c2 = -plane_b.offset
    term1 = scale(cross(direction, plane_b.normal), c1)
    term2 = scale(cross(plane_a.normal, direction), c2)
    point = scale(add(term1, term2), 1.0 / denom)
    return point, normalize(direction)


@dataclass
class SegmentSupport:
    start: Vector
    end: Vector
    support_points: list[Point3D]
    max_gap: float

    @property
    def length(self) -> float:
        return distance(self.start, self.end)

