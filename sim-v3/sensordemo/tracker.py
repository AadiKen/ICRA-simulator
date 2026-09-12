from __future__ import annotations

from dataclasses import dataclass
import math

from .config import SensorDemoConfig
from .sensors import Detection


@dataclass
class Track:
    track_id: int
    north_m: float
    east_m: float
    velocity_north_mps: float
    velocity_east_mps: float
    last_update_s: float
    state_time_s: float
    source_radar: bool = False
    source_ais: bool = False
    object_id: str | None = None

    def position_at(self, now_s: float) -> tuple[float, float]:
        dt = max(0.0, now_s - self.state_time_s)
        return self.north_m + self.velocity_north_mps * dt, self.east_m + self.velocity_east_mps * dt


class TrackManager:
    def __init__(self, config: SensorDemoConfig) -> None:
        self.config, self.tracks, self._next_id = config, [], 1

    def update(self, detections: list[Detection], now_s: float) -> list[Track]:
        for track in self.tracks:
            track.north_m, track.east_m = track.position_at(now_s)
            track.state_time_s = now_s
        for detection in sorted(detections, key=lambda item: item.timestamp_s):
            candidates = [(math.hypot(track.north_m-detection.north_m, track.east_m-detection.east_m), track) for track in self.tracks if detection.object_id is None or track.object_id in (None, detection.object_id)]
            distance, track = min(candidates, default=(math.inf, None), key=lambda item: item[0])
            if track is None or distance > self.config.track_gate_m:
                track = Track(self._next_id, detection.north_m, detection.east_m, detection.velocity_north_mps or 0.0, detection.velocity_east_mps or 0.0, detection.timestamp_s, now_s, object_id=detection.object_id)
                self._next_id += 1
                self.tracks.append(track)
            else:
                dt = max(1e-6, now_s - track.last_update_s)
                rn, re = detection.north_m-track.north_m, detection.east_m-track.east_m
                track.north_m += self.config.tracker_alpha * rn
                track.east_m += self.config.tracker_alpha * re
                track.velocity_north_mps += self.config.tracker_beta * rn / dt
                track.velocity_east_mps += self.config.tracker_beta * re / dt
                if detection.velocity_north_mps is not None:
                    track.velocity_north_mps = (track.velocity_north_mps + detection.velocity_north_mps) / 2
                    track.velocity_east_mps = (track.velocity_east_mps + (detection.velocity_east_mps or 0.0)) / 2
                track.last_update_s = detection.timestamp_s
                track.object_id = track.object_id or detection.object_id
            track.source_radar |= detection.sensor == "radar"
            track.source_ais |= detection.sensor == "ais"
        self.tracks = [track for track in self.tracks if now_s - track.last_update_s <= self.config.track_coast_time_s]
        return list(self.tracks)
