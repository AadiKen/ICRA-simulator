#!/usr/bin/env python3
"""Pose-derived planar velocity for live Gazebo/VRX adapters.

This is the live-runtime counterpart of ``poseDerivedPlanarTwist`` in
episode-driver.ts.  It deliberately never consumes OdometryPublisher twist.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrapped_angle_delta(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


@dataclass(frozen=True)
class PlanarPose:
    time_s: float
    north_m: float
    east_m: float
    yaw_ned_rad: float


class PoseDerivedPlanarVelocity:
    def __init__(self) -> None:
        self.previous: PlanarPose | None = None
        self.last_velocity = (0.0, 0.0, 0.0)

    def reset(self) -> None:
        self.previous = None
        self.last_velocity = (0.0, 0.0, 0.0)

    def update(self, pose: PlanarPose) -> tuple[float, float, float]:
        previous = self.previous
        if previous is None:
            self.previous = pose
            return self.last_velocity
        dt = pose.time_s - previous.time_s
        if abs(dt) <= 1e-12:
            return self.last_velocity
        if not math.isfinite(dt) or dt < 0:
            raise ValueError(f"pose timestamps must increase; received dt={dt}")
        north_rate = (pose.north_m - previous.north_m) / dt
        east_rate = (pose.east_m - previous.east_m) / dt
        cosine, sine = math.cos(pose.yaw_ned_rad), math.sin(pose.yaw_ned_rad)
        self.previous = pose
        self.last_velocity = (
            north_rate * cosine + east_rate * sine,
            -north_rate * sine + east_rate * cosine,
            wrapped_angle_delta(pose.yaw_ned_rad, previous.yaw_ned_rad) / dt,
        )
        return self.last_velocity
