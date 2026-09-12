from __future__ import annotations

from typing import Any, Protocol
import math
import gymnasium as gym
import numpy as np

from .ablations import blind_observation
from .config import SensorDemoConfig
from .obs_contract import ObservationStack, assemble_frame
from .reward import compute_reward
from .sensors import AISReceiver, EgoPose, RadarSensor, SceneObject
from .tracker import TrackManager
from .waypoint_sources import VesselState, relative_waypoint_to_ned


class SensorDemoBackend(Protocol):
    """Boundary to the validated autopilot and simulator services."""
    physics_timestep_s: float
    def reset(self, seed: int) -> dict[str, Any]: ...
    def ground_truth(self) -> dict[str, Any]: ...
    def scene_objects(self) -> list[SceneObject]: ...
    def line_of_sight(self, origin_ne, target_ne, target_id) -> bool: ...
    def environment_at(self, north_m: float, east_m: float) -> dict[str, Any]: ...
    def set_autopilot_waypoint(self, north_m: float, east_m: float) -> None: ...
    def step_autopilot(self) -> dict[str, Any]: ...
    def close(self) -> None: ...


class SensorDemoWaypointEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, backend: SensorDemoBackend, goal_ne_m: tuple[float, float], route_bounds_ne: tuple[float, float, float, float], config: SensorDemoConfig | None = None, blind: bool = False, base_seed: int = 0) -> None:
        self.backend, self.goal, self.bounds = backend, goal_ne_m, route_bounds_ne
        self.config, self.blind, self.base_seed, self.episode = config or SensorDemoConfig(), blind, base_seed, 0
        ratio = self.config.waypoint_interval_s/backend.physics_timestep_s
        if not math.isclose(ratio, round(ratio), abs_tol=1e-9): raise ValueError("waypoint interval must be an integer number of physics steps")
        self.steps_per_action = round(ratio)
        self.action_space = gym.spaces.Box(np.array([-1., -1.], np.float32), np.array([1., 1.], np.float32))
        self.observation_space = gym.spaces.Box(-1., 1., (320,), np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed); actual = self.base_seed+self.episode if seed is None else seed; self.episode += 1
        self.backend.reset(actual); self.actual_seed = actual; self.rng = np.random.default_rng(actual); self.radar = RadarSensor(self.config, self.rng); self.ais = AISReceiver(self.config, self.rng); self.tracker = TrackManager(self.config); self.stack = ObservationStack(self.config)
        self.elapsed_s = 0.; self.previous_bearing = 0.; self.total_effort = 0.; self.episode_return = 0.; self.episode_steps = 0; self.minimum_true_cpa_m = math.inf; self.reward_totals = {name: 0. for name in ("progress", "collision", "proximity", "jitter", "effort", "goal")}; truth = self.backend.ground_truth(); self.previous_goal_distance = self._goal_distance(truth)
        obs = self.stack.reset(self._frame(truth)); return self._maybe_blind(obs), {"seed": actual}

    def _goal_distance(self, truth): return math.hypot(self.goal[0]-truth["position_ned_m"][0], self.goal[1]-truth["position_ned_m"][1])

    def _frame(self, truth):
        n, e = truth["position_ned_m"][:2]; yaw = truth["attitude_rad"][2]; ego = EgoPose(n, e, yaw); objects = self.backend.scene_objects()
        detections = self.radar.scan(ego, objects, self.elapsed_s, self.backend.line_of_sight)+self.ais.scan(ego, objects, self.elapsed_s)
        tracks = self.tracker.update(detections, self.elapsed_s); env = self.backend.environment_at(n, e); velocity = tuple(truth["velocity_body_mps"][:2]); sog = math.hypot(*truth.get("velocity_ned_mps", velocity)[:2])
        return assemble_frame(config=self.config, ego=ego, velocity_body_mps=velocity, yaw_rate_rad_s=truth["angular_rate_body_rad_s"][2], speed_over_ground_mps=sog, goal_ne_m=self.goal, current_ne_mps=tuple(env.get("current_ned_mps", (0., 0.))[:2]), wave_height_m=env.get("wave_significant_height_m", 0.), wave_period_s=env.get("wave_peak_period_s", 0.), wave_direction_rad=env.get("wave_direction_ned_rad", 0.), tracks=tracks, now_s=self.elapsed_s, depth_under_keel_m=env.get("depth_under_keel_m", self.config.depth_scale_m))

    def _maybe_blind(self, obs): return blind_observation(obs) if self.blind else obs

    def step(self, action):
        a = np.clip(np.asarray(action, float), -1., 1.); distance = self.config.waypoint_range_min_m+(a[0]+1.)*.5*(self.config.waypoint_range_max_m-self.config.waypoint_range_min_m); bearing = a[1]*self.config.waypoint_bearing_max_rad
        truth = self.backend.ground_truth(); state = VesselState(truth["position_ned_m"][0], truth["position_ned_m"][1], truth["attitude_rad"][2]); waypoint = relative_waypoint_to_ned(state, distance, bearing); self.backend.set_autopilot_waypoint(*waypoint)
        collision = False; effort = 0.; true_cpa = math.inf; backend_info = {}
        for _ in range(self.steps_per_action):
            backend_info = self.backend.step_autopilot() or {}; self.elapsed_s += self.backend.physics_timestep_s; effort += float(backend_info.get("thrust_magnitude_n", 0.))*self.backend.physics_timestep_s; collision |= bool(backend_info.get("collision") or backend_info.get("grounding")); true_cpa = min(true_cpa, float(backend_info.get("true_cpa_m", math.inf)))
            if collision: break
        truth = self.backend.ground_truth(); goal_distance = self._goal_distance(truth); reached = goal_distance <= self.config.goal_radius_m; n, e = truth["position_ned_m"][:2]; outside = not (self.bounds[0]-self.config.route_margin_m <= n <= self.bounds[1]+self.config.route_margin_m and self.bounds[2]-self.config.route_margin_m <= e <= self.bounds[3]+self.config.route_margin_m); timeout = self.elapsed_s >= self.config.max_episode_time_s
        terms = compute_reward(self.config, previous_goal_distance_m=self.previous_goal_distance, goal_distance_m=goal_distance, collision=collision, true_cpa_m=true_cpa, bearing_action=float(a[1]), previous_bearing_action=self.previous_bearing, integrated_thrust_ns=effort, thrust_normalizer_ns=1_000.*self.config.waypoint_interval_s, reached_goal=reached)
        self.previous_goal_distance, self.previous_bearing = goal_distance, float(a[1]); self.episode_return += terms.total; self.episode_steps += 1; self.minimum_true_cpa_m = min(self.minimum_true_cpa_m, true_cpa)
        for name, value in terms.__dict__.items(): self.reward_totals[name] += value
        obs = self._maybe_blind(self.stack.append(self._frame(truth))); terminated = reached or collision or outside
        reason = "success" if reached else "collision" if collision else "outside_route_bounds" if outside else "timeout" if timeout else "running"
        return obs, terms.total, terminated, timeout, {"success": reached, "collision": collision, "outside_route_bounds": outside, "termination_reason": reason, "terminal_seed": self.actual_seed, "elapsed_s": self.elapsed_s, "episode_return": self.episode_return, "episode_steps": self.episode_steps, "minimum_true_cpa_m": self.minimum_true_cpa_m, "episode_reward_components": self.reward_totals.copy(), "commanded_waypoint_ned_m": waypoint, "goal_distance_m": goal_distance, "reward_components": terms.__dict__, "track_count": len(self.tracker.tracks), "tracks": [track.__dict__.copy() for track in self.tracker.tracks], **backend_info}

    def close(self): self.backend.close()
