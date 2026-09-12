from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

from bcod_sim import PersistentNodeBridge

from .config import SensorDemoConfig
from .sensors import SceneObject, polygon_line_of_sight
from .waypoint_env import SensorDemoWaypointEnv


ROOT = Path(__file__).resolve().parents[1]
DEMO_START_NE = (10_000.0, 10_000.0)
DEMO_GOAL_NE = (10_360.0, 10_180.0)
DEMO_BOUNDS = (9_950.0, 10_410.0, 9_950.0, 10_230.0)
# Two headlands create a visible occlusion-rich passage. Coordinates are local
# NED metres and intentionally synthetic until a geographic route is selected.
DEMO_ENC_POLYGONS = (
    ((10_120., 10_030.), (10_205., 10_030.), (10_205., 10_080.), (10_120., 10_080.)),
    ((10_225., 10_120.), (10_305., 10_120.), (10_305., 10_175.), (10_225., 10_175.)),
)


class LocalNodeSensorDemoBackend:
    """Runnable adapter using the production Node plant and frozen LOS-PID-v2 gains.

    This is a simulator-validated controller path, not a claim of field-trial
    validation. Replace only this adapter when that external autopilot is exposed.
    """

    physics_timestep_s = 0.05

    def __init__(self, config: SensorDemoConfig | None = None) -> None:
        self.config = config or SensorDemoConfig()
        self.bridge = PersistentNodeBridge(ROOT)
        self.rng = np.random.default_rng(0)
        self.waypoint = DEMO_GOAL_NE
        self.leg_start = DEMO_START_NE
        self.current = (0., 0.)
        self.wave = (0., 8., 0.)
        self.objects: list[SceneObject] = []
        self._visible = polygon_line_of_sight(DEMO_ENC_POLYGONS)
        self._last_info: dict[str, Any] = {}

    def reset(self, seed: int) -> dict[str, Any]:
        self.rng = np.random.default_rng(seed)
        angle = self.rng.uniform(-math.pi, math.pi); speed = self.rng.uniform(0.05, 0.45)
        self.current = (speed*math.cos(angle), speed*math.sin(angle))
        self.wave = (self.rng.uniform(.2, 1.2), self.rng.uniform(6., 12.), self.rng.uniform(-math.pi, math.pi))
        start = (DEMO_START_NE[0]+self.rng.uniform(-10, 10), DEMO_START_NE[1]+self.rng.uniform(-10, 10))
        self.objects = [
            SceneObject("headland-a", 10_162., 10_055., radius_m=45., kind="static"),
            SceneObject("headland-b", 10_265., 10_147., radius_m=42., kind="static"),
            SceneObject("traffic-ais", 10_250., 9_990., 0., .8, radius_m=5., ais_broadcasting=True),
            SceneObject("traffic-dark", 10_330., 10_205., -.55, -.25, radius_m=4., ais_broadcasting=False),
        ]
        cfg = {"schema_version": 1, "experiment": {"name": f"sensor-demo-{seed}", "seed": seed, "timestep_s": self.physics_timestep_s, "duration_s": self.config.max_episode_time_s+10}, "backend": {"type": "node"}, "vehicle": {"preset": "vehicle-a-otter", "plant": "planar3"}, "environment": {"current_mps": [*self.current, 0.], "wind_mps": [0., 0., 0.]}, "initial_state": {"position_ned_m": [*start, 0.], "attitude_rad": [0., 0., math.atan2(DEMO_GOAL_NE[1]-start[1], DEMO_GOAL_NE[0]-start[0])]}, "mission": {"type": "rl-common-waypoint-v1", "waypoints": [{"north_m": DEMO_GOAL_NE[0], "east_m": DEMO_GOAL_NE[1]}]}, "sensors": []}
        self.leg_start, self.waypoint = start, DEMO_GOAL_NE
        return self.bridge.reset([cfg])

    def ground_truth(self): return self.bridge.ground_truth()

    def scene_objects(self):
        time_s = float(self.ground_truth().get("time_s", 0.))
        result = []
        for item in self.objects:
            if item.kind == "vessel":
                result.append(SceneObject(item.object_id, item.north_m+item.velocity_north_mps*time_s, item.east_m+item.velocity_east_mps*time_s, item.velocity_north_mps, item.velocity_east_mps, item.radius_m, item.kind, item.ais_broadcasting))
            else: result.append(item)
        return result

    def line_of_sight(self, origin_ne, target_ne, target_id): return self._visible(origin_ne, target_ne, target_id)

    def environment_at(self, north_m, east_m):
        # Mild spatial variation makes current-aware routing observable while
        # retaining the run's configured background-current provenance.
        phase = (north_m+east_m)*.005
        return {"current_ned_mps": [self.current[0]+.05*math.sin(phase), self.current[1]+.05*math.cos(phase)], "wave_significant_height_m": self.wave[0], "wave_peak_period_s": self.wave[1], "wave_direction_ned_rad": self.wave[2], "depth_under_keel_m": max(-2., 18.-.01*max(0., north_m-10_300.))}

    def set_autopilot_waypoint(self, north_m, east_m):
        truth = self.ground_truth(); self.leg_start = tuple(truth["position_ned_m"][:2]); self.waypoint = (north_m, east_m)

    def step_autopilot(self):
        truth = self.ground_truth(); n, e = truth["position_ned_m"][:2]; yaw = truth["attitude_rad"][2]; u = truth["velocity_body_mps"][0]; r = truth["angular_rate_body_rad_s"][2]
        dn, de = self.waypoint[0]-self.leg_start[0], self.waypoint[1]-self.leg_start[1]; length = max(math.hypot(dn, de), 1e-9); cn, ce = dn/length, de/length; cross = -ce*(n-self.leg_start[0])+cn*(e-self.leg_start[1]); desired = math.atan2(ce, cn)-math.atan2(cross, 8.); error = math.atan2(math.sin(desired-yaw), math.cos(desired-yaw))
        surge = float(np.clip(100.*(1.-u), -150., 150.)); yaw_moment = float(np.clip(100.*error-35.*r, -100., 100.))
        result = self.bridge.step([{"actuators": {"desiredWrench": [surge, 0., 0., 0., 0., yaw_moment]}}]); truth = self.ground_truth(); n, e = truth["position_ned_m"][:2]
        distances = [math.hypot(obj.north_m-n, obj.east_m-e)-obj.radius_m for obj in self.scene_objects()]
        environment = self.environment_at(n, e); collision = min(distances, default=math.inf) <= 2.; grounding = environment["depth_under_keel_m"] <= 0
        info = result["infos"][0]; diagnostics = info.get("vehicle_diagnostics", {}); full = diagnostics.get("full_wrench", [0.]*6)
        return {"collision": collision, "grounding": grounding, "true_cpa_m": min(distances, default=math.inf), "thrust_magnitude_n": abs(float(full[0] if full else 0.)), "simulator_terminated": result["terminated"][0], "controller": "frozen LOS-PID-v2", "controller_validation_scope": "bcod-sim reference; not field-trial evidence"}

    def close(self): self.bridge.close()


def create_training_env(*, blind: bool = False) -> SensorDemoWaypointEnv:
    config = SensorDemoConfig()
    return SensorDemoWaypointEnv(LocalNodeSensorDemoBackend(config), DEMO_GOAL_NE, DEMO_BOUNDS, config=config, blind=blind, base_seed=420_000)
