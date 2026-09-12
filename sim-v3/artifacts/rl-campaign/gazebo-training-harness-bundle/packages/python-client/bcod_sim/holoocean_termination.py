"""HoloOcean-native termination monitoring for Vehicle A."""
from __future__ import annotations

import math
from typing import Iterable

import numpy as np


TILT_LIMIT_RAD = math.radians(60.0)
INSTABILITY_HOLD_S = 1.0
ALLOCATION_RELATIVE_ERROR = 0.20
ALLOCATION_HOLD_S = 1.0


class HoloOceanTerminationMonitor:
    """Map available HoloOcean signals to the portable termination categories.

    HoloOcean's CollisionSensor exposes only a boolean. It deliberately maps to
    generic ``collision`` because the sensor discards the other actor and cannot
    distinguish grounding from an object collision.
    """

    def __init__(self) -> None:
        self.unstable_s = 0.0
        self.allocation_bad_s = 0.0

    def update(
        self,
        *,
        dt_s: float,
        acceleration_body_frd_mps2: Iterable[float],
        colliding: bool,
        commanded_thrust_n: Iterable[float],
        applied_thrust_n: Iterable[float],
    ) -> str | None:
        acceleration = np.asarray(tuple(acceleration_body_frd_mps2), dtype=np.float64)
        commanded = np.asarray(tuple(commanded_thrust_n), dtype=np.float64)
        applied = np.asarray(tuple(applied_thrust_n), dtype=np.float64)

        finite_state = acceleration.shape == (3,) and bool(np.all(np.isfinite(acceleration)))
        magnitude = float(np.linalg.norm(acceleration)) if finite_state else 0.0
        # IMUSensor has no attitude output. Gravity direction supplies a
        # sensor-derived combined tilt estimate, with a one-second hold to reject
        # short translational-acceleration transients.
        tilt_rad = (
            math.atan2(math.hypot(float(acceleration[0]), float(acceleration[1])), abs(float(acceleration[2])))
            if finite_state and magnitude > 1e-6
            else math.inf
        )
        unstable = not finite_state or not math.isfinite(tilt_rad) or tilt_rad > TILT_LIMIT_RAD
        self.unstable_s = self.unstable_s + dt_s if unstable else 0.0

        finite_allocation = (
            commanded.shape == (2,)
            and applied.shape == (2,)
            and bool(np.all(np.isfinite(commanded)))
            and bool(np.all(np.isfinite(applied)))
        )
        requested = float(np.linalg.norm(commanded)) if finite_allocation else math.inf
        residual = float(np.linalg.norm(commanded - applied)) if finite_allocation else math.inf
        allocation_bad = (not finite_allocation) or (
            requested > 1e-12 and residual / requested > ALLOCATION_RELATIVE_ERROR
        )
        self.allocation_bad_s = self.allocation_bad_s + dt_s if allocation_bad else 0.0

        if not finite_state or self.unstable_s >= INSTABILITY_HOLD_S - 1e-12:
            return "instability"
        if colliding:
            return "collision"
        if not finite_allocation or self.allocation_bad_s >= ALLOCATION_HOLD_S - 1e-12:
            return "allocation_failure"
        return None
