from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable, Protocol

import numpy as np

from .config import SensorDemoConfig


@dataclass(frozen=True)
class EgoPose:
    north_m: float
    east_m: float
    yaw_rad: float


@dataclass(frozen=True)
class SceneObject:
    object_id: str
    north_m: float
    east_m: float
    velocity_north_mps: float = 0.0
    velocity_east_mps: float = 0.0
    radius_m: float = 1.0
    kind: str = "vessel"
    ais_broadcasting: bool = False


@dataclass(frozen=True)
class Detection:
    sensor: str
    timestamp_s: float
    north_m: float
    east_m: float
    object_id: str | None = None
    velocity_north_mps: float | None = None
    velocity_east_mps: float | None = None
    false_positive: bool = False


class LineOfSight(Protocol):
    def __call__(self, origin_ne: tuple[float, float], target_ne: tuple[float, float], target_id: str) -> bool: ...


def _relative(ego: EgoPose, obj: SceneObject) -> tuple[float, float, float]:
    dn, de = obj.north_m - ego.north_m, obj.east_m - ego.east_m
    return math.hypot(dn, de), math.atan2(de, dn), math.atan2(de, dn) - ego.yaw_rad


class RadarSensor:
    def __init__(self, config: SensorDemoConfig, rng: np.random.Generator) -> None:
        self.config, self.rng = config, rng

    def scan(self, ego: EgoPose, objects: Iterable[SceneObject], timestamp_s: float, line_of_sight: LineOfSight | None = None) -> list[Detection]:
        detections: list[Detection] = []
        for obj in objects:
            distance, world_bearing, _ = _relative(ego, obj)
            if distance > self.config.radar_max_range_m or distance <= obj.radius_m:
                continue
            if line_of_sight is not None and not line_of_sight((ego.north_m, ego.east_m), (obj.north_m, obj.east_m), obj.object_id):
                continue
            edge = distance / self.config.radar_max_range_m
            probability = (1.0 - self.config.radar_dropout_probability) * (1.0 - 0.5 * edge**4)
            if self.rng.random() > probability:
                continue
            rs = self.config.radar_range_sigma_near_m + edge * (self.config.radar_range_sigma_far_m - self.config.radar_range_sigma_near_m)
            bs = self.config.radar_bearing_sigma_near_rad + edge * (self.config.radar_bearing_sigma_far_rad - self.config.radar_bearing_sigma_near_rad)
            measured_range = max(0.0, distance + self.rng.normal(0.0, rs))
            measured_bearing = world_bearing + self.rng.normal(0.0, bs)
            detections.append(Detection("radar", timestamp_s, ego.north_m + measured_range * math.cos(measured_bearing), ego.east_m + measured_range * math.sin(measured_bearing), obj.object_id))
        if self.rng.random() < self.config.radar_false_positive_rate_per_scan:
            distance = self.rng.uniform(0.05, 1.0) * self.config.radar_max_range_m
            bearing = self.rng.uniform(-math.pi, math.pi)
            detections.append(Detection("radar", timestamp_s, ego.north_m + distance * math.cos(bearing), ego.east_m + distance * math.sin(bearing), false_positive=True))
        return detections


class AISReceiver:
    def __init__(self, config: SensorDemoConfig, rng: np.random.Generator) -> None:
        self.config, self.rng = config, rng
        self._next_report: dict[str, float] = {}

    def scan(self, ego: EgoPose, objects: Iterable[SceneObject], timestamp_s: float) -> list[Detection]:
        detections: list[Detection] = []
        for obj in objects:
            if obj.kind != "vessel" or not obj.ais_broadcasting:
                continue
            distance, _, _ = _relative(ego, obj)
            if distance > self.config.ais_max_range_m or timestamp_s + 1e-12 < self._next_report.get(obj.object_id, 0.0):
                continue
            interval = self.rng.uniform(self.config.ais_update_min_s, self.config.ais_update_max_s)
            latency = self.rng.uniform(0.0, interval)
            self._next_report[obj.object_id] = timestamp_s + interval
            n = obj.north_m - obj.velocity_north_mps * latency + self.rng.normal(0.0, self.config.ais_position_sigma_m)
            e = obj.east_m - obj.velocity_east_mps * latency + self.rng.normal(0.0, self.config.ais_position_sigma_m)
            detections.append(Detection("ais", timestamp_s - latency, n, e, obj.object_id, obj.velocity_north_mps, obj.velocity_east_mps))
        return detections


def polygon_line_of_sight(polygons: Iterable[Iterable[tuple[float, float]]]) -> LineOfSight:
    """Static ENC-only 2-D occlusion fallback from section 3.4."""
    rings = [list(ring) for ring in polygons]

    def intersects(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
        def cross(p, q, r): return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
        return cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0

    def visible(origin, target, target_id):
        del target_id
        return not any(intersects(origin, target, ring[i], ring[(i + 1) % len(ring)]) for ring in rings for i in range(len(ring)))
    return visible
