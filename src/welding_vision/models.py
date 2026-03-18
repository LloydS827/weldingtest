from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def to_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class PlaneModel:
    id: str
    normal: tuple[float, float, float]
    offset: float
    support_count: int
    centroid: tuple[float, float, float]
    bounds: dict[str, tuple[float, float]]
    inlier_indices: list[int] = field(default_factory=list)

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "normal": list(self.normal),
            "offset": self.offset,
            "support_count": self.support_count,
            "centroid": list(self.centroid),
            "bounds": {axis: list(span) for axis, span in self.bounds.items()},
        }


@dataclass
class SeamCandidate:
    id: str
    start: Point3D
    end: Point3D
    direction: tuple[float, float, float]
    length: float
    support_count: int
    source_plane_ids: tuple[str, str]
    confidence: float
    score_breakdown: dict[str, float]

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "start": self.start.to_dict(),
            "end": self.end.to_dict(),
            "direction": list(self.direction),
            "length": self.length,
            "support_count": self.support_count,
            "source_plane_ids": list(self.source_plane_ids),
            "confidence": self.confidence,
            "score_breakdown": self.score_breakdown,
        }

