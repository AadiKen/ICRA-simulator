from __future__ import annotations

from collections import deque
import hashlib
import json
import math

import numpy as np

from .config import SensorDemoConfig
from .sensors import EgoPose
from .tracker import Track


PER_FRAME_FIELDS = (
    "surge_velocity", "sway_velocity", "yaw_rate", "orientation_yaw_sin", "orientation_yaw_cos", "speed_over_ground",
    "goal_range", "goal_bearing_sin", "goal_bearing_cos", "current_u_body", "current_v_body", "wave_significant_height",
    "wave_peak_period", "wave_direction_sin", "wave_direction_cos",
) + tuple(f"track_{index}_{field}" for index in range(8) for field in ("range", "bearing_sin", "bearing_cos", "rel_velocity_along", "rel_velocity_across", "track_age", "source_radar", "source_ais")) + ("depth_under_keel",)
CONTRACT = {"name": "sensor-demo-observation-v2", "frame": "vessel-relative-body-NED", "per_frame_fields": PER_FRAME_FIELDS, "per_frame_dimension": 80, "stack_size": 4, "stacked_dimension": 320, "padding": {"track_range": 1.0}}
SENSOR_DEMO_OBS_CONTRACT_HASH = hashlib.sha256(json.dumps(CONTRACT, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _clip(value: float) -> float: return float(np.clip(value, -1.0, 1.0))


def assemble_frame(*, config: SensorDemoConfig, ego: EgoPose, velocity_body_mps: tuple[float, float], yaw_rate_rad_s: float, speed_over_ground_mps: float, goal_ne_m: tuple[float, float], current_ne_mps: tuple[float, float], wave_height_m: float, wave_period_s: float, wave_direction_rad: float, tracks: list[Track], now_s: float, depth_under_keel_m: float) -> np.ndarray:
    dn, de = goal_ne_m[0]-ego.north_m, goal_ne_m[1]-ego.east_m
    goal_bearing = math.atan2(de, dn)-ego.yaw_rad
    cy, sy = math.cos(ego.yaw_rad), math.sin(ego.yaw_rad)
    current_u = current_ne_mps[0]*cy + current_ne_mps[1]*sy
    current_v = -current_ne_mps[0]*sy + current_ne_mps[1]*cy
    values = [_clip(velocity_body_mps[0]/config.velocity_scale_mps), _clip(velocity_body_mps[1]/config.velocity_scale_mps), _clip(yaw_rate_rad_s/config.yaw_rate_scale_rad_s), math.sin(ego.yaw_rad), math.cos(ego.yaw_rad), _clip(speed_over_ground_mps/config.velocity_scale_mps), _clip(math.hypot(dn, de)/config.route_length_m), math.sin(goal_bearing), math.cos(goal_bearing), _clip(current_u/config.current_scale_mps), _clip(current_v/config.current_scale_mps), _clip(wave_height_m/config.wave_height_scale_m), _clip(wave_period_s/config.wave_period_scale_s), math.sin(wave_direction_rad-ego.yaw_rad), math.cos(wave_direction_rad-ego.yaw_rad)]
    ranked = sorted(tracks, key=lambda track: math.hypot(track.position_at(now_s)[0]-ego.north_m, track.position_at(now_s)[1]-ego.east_m))[:config.max_tracks]
    for track in ranked:
        tn, te = track.position_at(now_s); rn, re = tn-ego.north_m, te-ego.east_m; distance = math.hypot(rn, re); bearing = math.atan2(re, rn)-ego.yaw_rad
        rvn, rve = track.velocity_north_mps, track.velocity_east_mps
        along = rvn*math.cos(bearing+ego.yaw_rad)+rve*math.sin(bearing+ego.yaw_rad)
        across = -rvn*math.sin(bearing+ego.yaw_rad)+rve*math.cos(bearing+ego.yaw_rad)
        values.extend([_clip(distance/config.track_range_scale_m), math.sin(bearing), math.cos(bearing), _clip(along/config.velocity_scale_mps), _clip(across/config.velocity_scale_mps), _clip((now_s-track.last_update_s)/config.track_age_scale_s), float(track.source_radar), float(track.source_ais)])
    for _ in range(config.max_tracks-len(ranked)): values.extend([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    values.append(_clip(depth_under_keel_m/config.depth_scale_m))
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (80,) or not np.isfinite(result).all(): raise ValueError("invalid sensor-demo observation frame")
    return result


class ObservationStack:
    def __init__(self, config: SensorDemoConfig) -> None:
        self.config, self.frames = config, deque(maxlen=config.frame_stack)

    def reset(self, frame: np.ndarray) -> np.ndarray:
        self.frames.clear(); self.frames.extend(np.asarray(frame, np.float32).copy() for _ in range(self.config.frame_stack)); return self.value()

    def append(self, frame: np.ndarray) -> np.ndarray:
        if not self.frames: return self.reset(frame)
        self.frames.append(np.asarray(frame, np.float32).copy()); return self.value()

    def value(self) -> np.ndarray:
        if len(self.frames) != self.config.frame_stack: raise RuntimeError("observation stack has not been initialized")
        return np.concatenate(tuple(self.frames)).astype(np.float32, copy=False)
