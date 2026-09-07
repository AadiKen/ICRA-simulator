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
sys.path.insert(0, str(REPOSITORY_ROOT / "packages/python-client/bcod_sim"))
from common_task import (  # noqa: E402
    classify_termination,
    compute_reward,
    cross_track_distance,
    passed_waypoint_plane,
)


class StonefishCommonTask:
    """No-Gym task wrapper used by validation and future training adapters."""

    def __init__(self, bridge: StonefishBridge, contract_path: Path) -> None:
        document = json.loads(contract_path.read_text())
        if document.get("content_sha256") != CONTRACT_SHA256:
            raise ValueError("Stonefish task requires the frozen contract")
        self.contract = next(
            task for task in document["tasks"]
            if task["task_id"] == "common-waypoint-transit-v1"
        )
        shaping = self.contract["reward"]["potential_shaping"]
        self.shaping_k = float(shaping["k"])
        self.shaping_gamma = float(shaping["gamma"])
        if self.shaping_gamma != 1.0:
            raise ValueError("Stonefish reward port requires shaping gamma exactly 1.0")
        self.bridge = bridge
        self.mapper = StonefishContractMapper(bridge, goal_north_m=0, goal_east_m=0)
        self.timeout_steps = int(self.contract["timing"]["episode_length_steps"])
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
        rng = np.random.default_rng(seed)
        angle = math.radians(rng.uniform(*ranges["route_rotation_deg"]))
        north = float(rng.uniform(*ranges["start_position_offset_m"]))
        east = float(rng.uniform(*ranges["start_position_offset_m"]))
        yaw = math.radians(rng.uniform(*ranges["start_heading_deg"]))
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
        )
        self.position = np.asarray(sample.raw_response["observation"]["gps"][2:4])
        self.velocity[:] = 0.0
        self.previous_action[:] = 0.0
        self.previous_distance = self._distance(self.waypoint)
        self.previous_final_distance = self._distance(len(self.route) - 1)
        self.cross_track_sum = 0.0
        self.control_steps = 0
        self.termination.reset()
        return np.asarray(sample.observation), {
            "seed": seed,
            "start_ned_m": self.start.tolist(),
            "initial_yaw_ned_rad": yaw,
            "route_ned_m": self.route,
            "current_ned_mps": [0.0, 0.0],
            "wind_ned_mps": [0.0, 0.0],
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
        self.previous_distance = next_previous_distance
        self.previous_final_distance = final_distance
        self.previous_action = applied.copy()
        self.control_steps += 1
        terminated = reason not in ("running", "timeout")
        truncated = reason == "timeout"
        yaw = float(sample.observation[6])
        forward = np.asarray([math.cos(yaw), math.sin(yaw)])
        return np.asarray(sample.observation), scored.reward, terminated, truncated, {
            "success": success,
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
        }
