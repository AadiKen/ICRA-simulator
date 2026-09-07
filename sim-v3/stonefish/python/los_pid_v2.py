"""Fixed LOS-PID-v2 port with the validated surge and turn corrections."""

from __future__ import annotations

import math

import numpy as np


LOOKAHEAD_M = 8.0
YAW_KP = 100.0
YAW_KD = 35.0
TARGET_SPEED_MPS = 1.0
MIN_TURN_SPEED_MPS = 0.5
YAW_SCHEDULE_FULL_SCALE = 100.0
SURGE_KP = 100.0
SURGE_KI = 100.0
THRUST_CEILING_N = 95.0


def clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class LosPidV2:
    def __init__(self, *, use_integral: bool = True,
                 use_turn_speed_schedule: bool = True) -> None:
        self.use_integral = use_integral
        self.use_turn_speed_schedule = use_turn_speed_schedule
        self.surge_integral_error = 0.0
        self.last_scheduled_speed = TARGET_SPEED_MPS

    def _surge_force(self, speed: float, target: float, dt_s: float) -> float:
        error = target - speed
        candidate_integral = self.surge_integral_error + error * dt_s
        integral_term = SURGE_KI * candidate_integral if self.use_integral else 0.0
        candidate = SURGE_KP * error + integral_term
        saturated = clamp(candidate, -THRUST_CEILING_N, THRUST_CEILING_N)
        drives_further = candidate != saturated and (
            (candidate > 0 and error > 0) or (candidate < 0 and error < 0)
        )
        if self.use_integral and not drives_further:
            self.surge_integral_error = candidate_integral
        force = SURGE_KP * error
        if self.use_integral:
            force += SURGE_KI * self.surge_integral_error
        return clamp(force, -THRUST_CEILING_N, THRUST_CEILING_N)

    def action(self, env, observation: np.ndarray) -> np.ndarray:
        leg_start = env.start if env.waypoint == 0 else np.asarray(env.route[env.waypoint - 1])
        goal = np.asarray(env.route[env.waypoint])
        dn, de = goal - leg_start
        length = math.hypot(float(dn), float(de))
        cn, ce = float(dn) / length, float(de) / length
        along = float(np.dot(env.position - leg_start, np.asarray([cn, ce])))
        cross = -ce * float(env.position[0] - leg_start[0]) + cn * float(env.position[1] - leg_start[1])
        if along >= length:
            desired = math.atan2(float(goal[1] - env.position[1]),
                                 float(goal[0] - env.position[0]))
        else:
            desired = math.atan2(ce, cn) - math.atan2(cross, LOOKAHEAD_M)
        heading_error = wrap(desired - float(observation[6]))
        yaw_demand = clamp(
            YAW_KP * heading_error - YAW_KD * float(observation[5]), -100.0, 100.0
        )
        target_speed = TARGET_SPEED_MPS
        if self.use_turn_speed_schedule:
            fraction = min(1.0, abs(yaw_demand) / YAW_SCHEDULE_FULL_SCALE)
            blend = 0.5 * (1.0 + math.cos(math.pi * fraction))
            target_speed = MIN_TURN_SPEED_MPS + (TARGET_SPEED_MPS - MIN_TURN_SPEED_MPS) * blend
        self.last_scheduled_speed = target_speed
        surge = self._surge_force(
            float(env.velocity @ np.asarray([math.cos(float(observation[6])),
                                             math.sin(float(observation[6]))])),
            target_speed, env.control_interval_s,
        )
        # Stonefish positive yaw requires port force greater than starboard.
        return np.asarray([
            clamp((surge + yaw_demand) / THRUST_CEILING_N, -1.0, 1.0),
            clamp((surge - yaw_demand) / THRUST_CEILING_N, -1.0, 1.0),
            0.0, 0.0,
        ], dtype=np.float64)
