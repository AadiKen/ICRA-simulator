"""Stonefish Vehicle A mapping for the frozen 15-field common contract."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

from stonefish_bridge import StonefishBridge


CONTRACT_SHA256 = "2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36"
FIELD_NAMES = (
    "imu.linear_accel_x",
    "imu.linear_accel_y",
    "imu.linear_accel_z",
    "imu.angular_rate_x",
    "imu.angular_rate_y",
    "imu.angular_rate_z",
    "imu.orientation_yaw_rad",
    "gps.relative_goal_north_m",
    "gps.relative_goal_east_m",
    "gps.fix_valid",
    "previous_action.effector_0",
    "previous_action.effector_1",
    "previous_action.steer_0",
    "previous_action.steer_1",
    "normalized_time_remaining",
)


@dataclass(frozen=True)
class ContractSample:
    observation: tuple[float, ...]
    raw_response: dict[str, Any]
    physics_steps: int


class StonefishContractMapper:
    """Expose the Gate A process bridge at the frozen task timing and schema.

    Stonefish already reports world position in NED and IMU vectors in body FRD,
    so the coordinate transform for every mapped component is the identity.  Yaw
    is deliberately taken from Compass rather than the IMU orientation tuple.
    """

    INTERNAL_TIMESTEP_S = 0.002
    CONTRACT_PHYSICS_TIMESTEP_S = 0.05
    CONTROL_INTERVAL_S = 0.1
    EPISODE_PHYSICS_STEPS = 2400
    INTERNAL_STEPS_PER_CONTRACT_STEP = 25
    CONTRACT_STEPS_PER_CONTROL = 2

    def __init__(
        self,
        bridge: StonefishBridge,
        *,
        goal_north_m: float,
        goal_east_m: float,
    ) -> None:
        self.bridge = bridge
        self.goal_north_m = float(goal_north_m)
        self.goal_east_m = float(goal_east_m)
        self.physics_steps = 0
        self.previous_action = (0.0, 0.0, 0.0, 0.0)

    def reset(self, seed: int, *, gps_z_ned: float = -0.5) -> ContractSample:
        self.physics_steps = 0
        self.previous_action = (0.0, 0.0, 0.0, 0.0)
        raw = self.bridge.reset(seed, gps_z_ned=gps_z_ned)
        return self._sample(raw)

    def step(self, action: Sequence[float]) -> ContractSample:
        if len(action) != 4:
            raise ValueError("contract action must contain four fields")
        applied = tuple(float(max(-1.0, min(1.0, value))) for value in action)
        if applied[2:] != (0.0, 0.0):
            raise ValueError("Vehicle A steer_0 and steer_1 must be zero")
        raw = self.bridge.step(
            applied[0],
            applied[1],
            physics_steps=(
                self.INTERNAL_STEPS_PER_CONTRACT_STEP
                * self.CONTRACT_STEPS_PER_CONTROL
            ),
        )
        self.physics_steps = min(
            self.EPISODE_PHYSICS_STEPS,
            self.physics_steps + self.CONTRACT_STEPS_PER_CONTROL,
        )
        self.previous_action = applied
        return self._sample(raw)

    def _sample(self, raw: dict[str, Any]) -> ContractSample:
        sensors = raw["observation"]
        imu = sensors["imu"]
        gps = sensors["gps"]
        compass = sensors["compass"]
        if len(imu) != 9 or len(gps) != 4 or len(compass) != 1:
            raise ValueError("unexpected Stonefish sensor response shape")

        fix_valid = bool(sensors["fix_valid"])
        if fix_valid:
            relative_north = self.goal_north_m - float(gps[2])
            relative_east = self.goal_east_m - float(gps[3])
        else:
            # Preserve the Gate A sentinel policy without turning sentinel
            # coordinates into a spurious goal vector.
            relative_north = 0.0
            relative_east = 0.0

        remaining = (
            self.EPISODE_PHYSICS_STEPS - self.physics_steps
        ) / self.EPISODE_PHYSICS_STEPS
        observation = (
            float(imu[6]),
            float(imu[7]),
            float(imu[8]),
            float(imu[3]),
            float(imu[4]),
            float(imu[5]),
            float(compass[0]),
            relative_north,
            relative_east,
            float(fix_valid),
            *self.previous_action,
            remaining,
        )
        if len(observation) != len(FIELD_NAMES):
            raise AssertionError("contract observation length changed")
        if not all(math.isfinite(value) for value in observation):
            raise ValueError("contract observation contains a non-finite value")
        return ContractSample(observation, raw, self.physics_steps)
