"""Simulator-neutral common-waypoint task calculations.

This module deliberately knows nothing about Node, ROS 2, Gazebo Transport, or
any simulator result shape.  Backends normalize telemetry and termination into
these small value objects before evaluating the frozen task contract.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np


TerminationReason = Literal[
    "running", "success", "grounding", "object_collision",
    "allocation_failure", "instability", "simulator_termination", "timeout",
]

PROGRESS_REWARD_GAIN = 2.0
COMPLETION_FRACTION_DEFINITION = (
    "clamp(signed cumulative route progress / full seeded route length, 0, 1); "
    "each step's route progress is the exact distance delta used by the shared "
    "progress reward, and full route length is start-to-first-waypoint plus all "
    "subsequent waypoint-leg lengths"
)


def full_route_length_m(
    start: tuple[float, float] | list[float] | np.ndarray,
    route: list[tuple[float, float] | list[float] | np.ndarray],
) -> float:
    """Return the length of every seeded route leg, including start -> waypoint 0."""
    points = [np.asarray(start, dtype=float), *(np.asarray(p, dtype=float) for p in route)]
    if len(points) < 2:
        raise ValueError("completion fraction requires at least one waypoint")
    total = sum(float(np.linalg.norm(b - a)) for a, b in zip(points, points[1:]))
    if total <= 0.0:
        raise ValueError("completion fraction requires a non-zero full route length")
    return total


@dataclass
class CompletionTracker:
    """Shared completion metric driven only by the reward's progress component."""

    total_route_length_m: float
    cumulative_route_progress_m: float = 0.0

    @classmethod
    def for_route(cls, start, route) -> "CompletionTracker":
        return cls(full_route_length_m(start, route))

    def update(self, progress_reward: float) -> float:
        # This is deliberately derived from the already-computed reward component:
        # no harness is allowed to recompute progress from simulator positions.
        self.cumulative_route_progress_m += float(progress_reward) / PROGRESS_REWARD_GAIN
        return float(np.clip(
            self.cumulative_route_progress_m / self.total_route_length_m, 0.0, 1.0
        ))


@dataclass(frozen=True)
class RewardResult:
    reward: float
    base_reward: float
    progress_reward: float
    cross_track_penalty: float
    action_delta_penalty: float
    terminal_reward: float
    potential_shaping: float

    def components(self) -> dict[str, float]:
        return {
            "progress": self.progress_reward,
            "cross_track": self.cross_track_penalty,
            "action_delta": self.action_delta_penalty,
            "terminal": self.terminal_reward,
            "base_reward": self.base_reward,
            "potential_shaping": self.potential_shaping,
            "shaped_reward": self.reward,
        }


def terminal_reward(reason: TerminationReason) -> float:
    if reason == "success":
        return 100.0
    if reason in ("grounding", "object_collision", "instability"):
        return -100.0
    if reason == "allocation_failure":
        return -25.0
    if reason == "timeout":
        return -10.0
    return 0.0


def compute_reward(
    previous_distance_m: float,
    distance_m: float,
    cross_track_m: float,
    action: np.ndarray,
    previous_action: np.ndarray,
    termination_reason: TerminationReason = "running",
    *,
    previous_final_distance_m: float | None = None,
    final_distance_m: float | None = None,
    shaping_k: float = 0.0,
    shaping_gamma: float = 1.0,
    shaping_enabled: bool = False,
) -> RewardResult:
    """Evaluate the frozen reward after backend telemetry is normalized."""
    progress = PROGRESS_REWARD_GAIN * (float(previous_distance_m) - float(distance_m))
    cross_track = -0.02 * float(cross_track_m)
    delta = np.asarray(action, dtype=float) - np.asarray(previous_action, dtype=float)
    action_delta = -0.05 * float(np.sum(delta * delta))
    terminal = terminal_reward(termination_reason)
    base = progress + cross_track + action_delta + terminal
    shaping = 0.0
    if shaping_enabled:
        if previous_final_distance_m is None or final_distance_m is None:
            raise ValueError("potential shaping requires both final-waypoint distances")
        phi_previous = -float(shaping_k) * float(previous_final_distance_m)
        # A time-limit truncation is not an absorbing MDP terminal state.  Keep
        # its real potential so the timeout cannot manufacture a shaping bonus.
        absorbing_terminal = termination_reason not in ("running", "timeout")
        phi_next = 0.0 if absorbing_terminal else -float(shaping_k) * float(final_distance_m)
        shaping = float(shaping_gamma) * phi_next - phi_previous
    return RewardResult(base + shaping, base, progress, cross_track, action_delta, terminal, shaping)


def classify_termination(
    *,
    success: bool,
    collision_type: str | None = None,
    allocation_failed: bool = False,
    unstable: bool = False,
    simulator_terminated: bool = False,
    timed_out: bool = False,
) -> TerminationReason:
    """Apply the contract's termination precedence to normalized backend flags."""
    if success:
        return "success"
    if collision_type == "grounding":
        return "grounding"
    if collision_type:
        return "object_collision"
    if allocation_failed:
        return "allocation_failure"
    if unstable:
        return "instability"
    if simulator_terminated:
        return "simulator_termination"
    if timed_out:
        return "timeout"
    return "running"


def cross_track_distance(position: tuple[float, float] | list[float], start: tuple[float, float] | list[float], end: tuple[float, float] | list[float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 0:
        raise ValueError("route leg must have non-zero length")
    q = max(0.0, min(1.0, ((position[0] - start[0]) * dx + (position[1] - start[1]) * dy) / denominator))
    return math.hypot(position[0] - (start[0] + q * dx), position[1] - (start[1] + q * dy))


def passed_waypoint_plane(position: tuple[float, float] | list[float], start: tuple[float, float] | list[float], end: tuple[float, float] | list[float], corridor_m: float = 15.361124064575238) -> bool:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 0:
        raise ValueError("route leg must have non-zero length")
    cross = abs(-dy * (position[0] - start[0]) + dx * (position[1] - start[1])) / math.sqrt(denominator)
    return (position[0] - start[0]) * dx + (position[1] - start[1]) * dy >= denominator and cross <= corridor_m
