from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SensorDemoConfig:
    """Frozen demo constants. Sensor values are plausible, not field validated."""

    waypoint_interval_s: float = 5.0
    waypoint_range_min_m: float = 2.0
    waypoint_range_max_m: float = 8.0
    waypoint_bearing_max_rad: float = math.pi / 3
    radar_max_range_m: float = 300.0
    radar_range_sigma_near_m: float = 0.5
    radar_range_sigma_far_m: float = 4.0
    radar_bearing_sigma_near_rad: float = math.radians(0.5)
    radar_bearing_sigma_far_rad: float = math.radians(3.0)
    radar_dropout_probability: float = 0.05
    radar_false_positive_rate_per_scan: float = 0.02
    ais_max_range_m: float = 10_000.0
    ais_position_sigma_m: float = 3.0
    ais_update_min_s: float = 5.0
    ais_update_max_s: float = 15.0
    track_gate_m: float = 35.0
    track_coast_time_s: float = 30.0
    tracker_alpha: float = 0.65
    tracker_beta: float = 0.20
    max_tracks: int = 8
    frame_stack: int = 4
    route_length_m: float = 2_000.0
    velocity_scale_mps: float = 5.0
    yaw_rate_scale_rad_s: float = 1.0
    current_scale_mps: float = 2.0
    wave_height_scale_m: float = 5.0
    wave_period_scale_s: float = 20.0
    track_range_scale_m: float = 500.0
    track_age_scale_s: float = 30.0
    depth_scale_m: float = 50.0
    safe_radius_m: float = 30.0
    goal_radius_m: float = 15.0
    max_episode_time_s: float = 1_800.0
    route_margin_m: float = 500.0
    w_progress: float = 1.0
    w_collision: float = 100.0
    w_proximity: float = 2.0
    w_jitter: float = 0.05
    w_effort: float = 0.01
    w_goal: float = 100.0

    def __post_init__(self) -> None:
        if self.frame_stack != 4 or self.max_tracks != 8:
            raise ValueError("The v2 observation contract requires K=4 and N=8")
        if not 0 < self.waypoint_range_min_m <= self.waypoint_range_max_m:
            raise ValueError("invalid waypoint range")
