"""Frozen common-waypoint task adapter for the Stonefish Vehicle A bridge."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
from typing import Any, Sequence

import numpy as np

from contract_mapping import CONTRACT_SHA256, StonefishContractMapper
from stonefish_bridge import StonefishBridge
from termination import StonefishTerminationMonitor


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT / "packages/python-client"))
sys.path.insert(0, str(REPOSITORY_ROOT / "packages/python-client/bcod_sim"))
from common_task import (  # noqa: E402
    CompletionTracker,
    classify_termination,
    compute_reward,
    cross_track_distance,
    passed_waypoint_plane,
)
from bcod_sim.common_task_env import Mulberry32  # noqa: E402
from bcod_sim.native_task_contract import load_native_task_contract  # noqa: E402


def draw_reset_randomization(contract: dict[str, Any], seed: int) -> dict[str, Any]:
    """Use the frozen bcod-sim PRNG and draw order for a portable scenario."""
    ranges = contract["reset_randomization"]
    rng = Mulberry32(seed)

    def uniform(bounds):
        low, high = bounds
        return low + (high - low) * rng.next()

    angle = math.radians(uniform(ranges["route_rotation_deg"]))
    north = float(uniform(ranges["start_position_offset_m"]))
    east = float(uniform(ranges["start_position_offset_m"]))
    current_speed = uniform(ranges["current_speed_m_s"])
    current_direction = 2.0 * math.pi * rng.next()
    wind_speed = uniform(ranges["wind_speed_m_s"])
    wind_direction = 2.0 * math.pi * rng.next()
    yaw = math.radians(uniform(ranges["start_heading_deg"]))
    return {
        "angle_rad": angle,
        "start_ned_m": [north, east],
        "heading_ned_rad": yaw,
        "current_ned_mps": [
            current_speed * math.cos(current_direction),
            current_speed * math.sin(current_direction),
            0.0,
        ],
        "wind_ned_mps": [
            wind_speed * math.cos(wind_direction),
            wind_speed * math.sin(wind_direction),
            0.0,
        ],
    }


class StonefishCommonTask:
    """No-Gym task wrapper used by validation and future training adapters."""

    def __init__(
        self,
        bridge: StonefishBridge,
        repository: Path,
        condition_contract_path: str | Path | None = None,
        disturbance_mode: str = "zero",
    ) -> None:
        repository = Path(repository)
        # Preserve direct Gate-D callers that historically passed the legacy
        # contract filename instead of the repository root.
        if repository.is_file():
            repository = repository.parents[3]
        self.contract, self.contract_binding = load_native_task_contract(
            repository, condition_contract_path
        )
        shaping = self.contract["reward"]["potential_shaping"]
        self.shaping_k = float(shaping["k"])
        self.shaping_gamma = float(shaping["gamma"])
        if self.shaping_gamma != 1.0:
            raise ValueError("Stonefish reward port requires shaping gamma exactly 1.0")
        self.bridge = bridge
        if disturbance_mode not in {"zero", "seeded"}:
            raise ValueError("disturbance_mode must be 'zero' or 'seeded'")
        self.disturbance_mode = disturbance_mode
        self.timeout_steps = int(self.contract["timing"]["episode_length_steps"])
        self.mapper = StonefishContractMapper(
            bridge, goal_north_m=0, goal_east_m=0,
            episode_physics_steps=self.timeout_steps,
        )
        self.control_interval_s = float(self.contract["timing"]["control_interval_s"])
        terminal = self.contract["learnability"]["absolute_success_rate_threshold"]["terminal_definition"]
        self.final_radius_m = float(terminal["radius_m"])
        self.intermediate_radius_m = 6.0
        self.route: list[list[float]] = []
        self.start = np.zeros(2)
        self.position = np.zeros(2)
        self.velocity = np.zeros(2)
        self.waypoint = 0
        self.previous_distance = 0.0
        self.previous_final_distance = 0.0
        self.previous_action = np.zeros(4)
        self.cross_track_sum = 0.0
        self.control_steps = 0
        self.termination = StonefishTerminationMonitor()

    def reset(self, seed: int) -> tuple[np.ndarray, dict[str, Any]]:
        ranges = self.contract["reset_randomization"]
        randomization = draw_reset_randomization(self.contract, seed)
        angle = randomization["angle_rad"]
        north, east = randomization["start_ned_m"]
        yaw = randomization["heading_ned_rad"]
        applied_current = (
            tuple(randomization["current_ned_mps"])
            if self.disturbance_mode == "seeded" else (0.0, 0.0, 0.0)
        )
        ca, sa = math.cos(angle), math.sin(angle)
        self.start = np.asarray([north, east], dtype=np.float64)
        self.route = [
            [north + n * ca - e * sa, east + n * sa + e * ca]
            for n, e in ranges["route_relative_m"]
        ]
        self.waypoint = 0
        self.mapper.goal_north_m, self.mapper.goal_east_m = self.route[0]
        sample = self.mapper.reset(
            seed,
            initial_north_m=north,
            initial_east_m=east,
            initial_yaw_rad=yaw,
            current_ned_mps=applied_current,
        )
        self.position = np.asarray(sample.raw_response["observation"]["gps"][2:4])
        self.velocity[:] = 0.0
        self.previous_action[:] = 0.0
        self.previous_distance = self._distance(self.waypoint)
        self.previous_final_distance = self._distance(len(self.route) - 1)
        self.completion_tracker = CompletionTracker.for_route(self.start, self.route)
        self.cross_track_sum = 0.0
        self.control_steps = 0
        self.termination.reset()
        return np.asarray(sample.observation), {
            "seed": seed,
            "start_ned_m": self.start.tolist(),
            "initial_yaw_ned_rad": yaw,
            "route_ned_m": self.route,
            "sampled_current_ned_mps": randomization["current_ned_mps"],
            "sampled_wind_ned_mps": randomization["wind_ned_mps"],
            "disturbance_mode": self.disturbance_mode,
            "current_ned_mps": list(applied_current[:2]),
            "wind_ned_mps": [0.0, 0.0],
            "applied_current_ned_mps": list(applied_current),
            "applied_wind_ned_mps": [0.0, 0.0, 0.0],
            "contract_binding": self.contract_binding,
        }

    def _distance(self, index: int) -> float:
        return float(np.linalg.norm(np.asarray(self.route[index]) - self.position))

    def step(self, action: Sequence[float]) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        applied = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if applied.shape != (4,) or not np.all(applied[2:] == 0.0):
            raise ValueError("Vehicle A action must be [port, starboard, 0, 0]")
        old_position = self.position.copy()
        sample = self.mapper.step(applied)
        raw = sample.raw_response
        self.position = np.asarray(raw["observation"]["gps"][2:4], dtype=np.float64)
        self.velocity = (self.position - old_position) / self.control_interval_s
        distance = self._distance(self.waypoint)
        leg_start = self.start if self.waypoint == 0 else np.asarray(self.route[self.waypoint - 1])
        cross_track = cross_track_distance(self.position.tolist(), leg_start.tolist(), self.route[self.waypoint])
        self.cross_track_sum += cross_track

        native_reason = self.termination.update(
            raw, dt_s=self.control_interval_s, requested_action=applied
        )
        if self.waypoint < len(self.route) - 1 and (
            distance <= self.intermediate_radius_m
            or passed_waypoint_plane(self.position.tolist(), leg_start.tolist(), self.route[self.waypoint])
        ):
            self.waypoint += 1
            next_previous_distance = self._distance(self.waypoint)
            self.mapper.goal_north_m, self.mapper.goal_east_m = self.route[self.waypoint]
            sample = self.mapper.remap(raw)
        else:
            next_previous_distance = distance

        final_distance = self._distance(len(self.route) - 1)
        success = self.waypoint == len(self.route) - 1 and final_distance <= self.final_radius_m
        timed_out = self.mapper.physics_steps >= self.timeout_steps
        reason = classify_termination(
            success=success,
            collision_type=native_reason if native_reason in ("grounding", "object_collision") else None,
            allocation_failed=native_reason == "allocation_failure",
            unstable=native_reason == "instability",
            timed_out=timed_out,
        )
        scored = compute_reward(
            self.previous_distance, distance, cross_track, applied,
            self.previous_action, reason,
            previous_final_distance_m=self.previous_final_distance,
            final_distance_m=final_distance,
            shaping_k=self.shaping_k,
            shaping_gamma=self.shaping_gamma,
            shaping_enabled=True,
        )
        completion_fraction = self.completion_tracker.update(scored.progress_reward)
        self.previous_distance = next_previous_distance
        self.previous_final_distance = final_distance
        self.previous_action = applied.copy()
        self.control_steps += 1
        terminated = reason not in ("running", "timeout")
        truncated = reason == "timeout"
        yaw = float(sample.observation[6])
        forward = np.asarray([math.cos(yaw), math.sin(yaw)])
        diagnostics = raw.get("diagnostics", {})
        return np.asarray(sample.observation), scored.reward, terminated, truncated, {
            "success": success,
            "completion_fraction": completion_fraction,
            "termination_reason": reason,
            "reward_components": scored.components(),
            "position_ned_m": self.position.tolist(),
            "ground_velocity_ned_mps": self.velocity.tolist(),
            "surge_speed_mps": float(np.dot(self.velocity, forward)),
            "distance_to_current_waypoint_m": self._distance(self.waypoint),
            "distance_to_final_waypoint_m": final_distance,
            "waypoints_reached": self.waypoint + (1 if success else 0),
            "current_waypoint_index": self.waypoint,
            "mean_cross_track_m": self.cross_track_sum / self.control_steps,
            "control_steps": self.control_steps,
            "physics_steps": self.mapper.physics_steps,
            "policy_propulsion_thrust_n": [
                float(diagnostics.get("port_thrust", math.nan)),
                float(diagnostics.get("starboard_thrust", math.nan)),
            ],
        }
