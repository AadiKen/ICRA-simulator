from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class VesselState:
    north_m: float
    east_m: float
    yaw_rad: float


class WaypointSource(ABC):
    @abstractmethod
    def next_waypoint(self, obs: np.ndarray, state: VesselState) -> tuple[float, float]: ...


class PolicySource(WaypointSource):
    def __init__(self, model, range_min_m: float, range_max_m: float, bearing_max_rad: float) -> None:
        self.model, self.range_min_m, self.range_max_m, self.bearing_max_rad = model, range_min_m, range_max_m, bearing_max_rad

    def next_waypoint(self, obs, state):
        del state
        action, _ = self.model.predict(obs, deterministic=True)
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        return self.range_min_m+(a[0]+1.0)*0.5*(self.range_max_m-self.range_min_m), a[1]*self.bearing_max_rad


class NaiveSource(WaypointSource):
    def __init__(self, goal_ne_m: tuple[float, float], range_m: float) -> None: self.goal_ne_m, self.range_m = goal_ne_m, range_m
    def next_waypoint(self, obs, state):
        del obs
        bearing = math.atan2(self.goal_ne_m[1]-state.east_m, self.goal_ne_m[0]-state.north_m)-state.yaw_rad
        return self.range_m, math.atan2(math.sin(bearing), math.cos(bearing))


def relative_waypoint_to_ned(state: VesselState, delta_range_m: float, delta_bearing_rad: float) -> tuple[float, float]:
    heading = state.yaw_rad + delta_bearing_rad
    return state.north_m + delta_range_m*math.cos(heading), state.east_m + delta_range_m*math.sin(heading)
