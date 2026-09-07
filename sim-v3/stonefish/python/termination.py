"""Portable termination taxonomy using Stonefish-native observations."""

from __future__ import annotations

import math
from typing import Any, Sequence


class StonefishTerminationMonitor:
    TILT_LIMIT_RAD = math.radians(60.0)
    INSTABILITY_HOLD_S = 1.0

    def __init__(self) -> None:
        self.unstable_s = 0.0

    def reset(self) -> None:
        self.unstable_s = 0.0

    def update(
        self,
        response: dict[str, Any],
        *,
        dt_s: float,
        requested_action: Sequence[float],
    ) -> str | None:
        imu = response.get("observation", {}).get("imu", ())
        finite_imu = len(imu) == 9 and all(math.isfinite(float(x)) for x in imu)
        tilted = finite_imu and (
            abs(float(imu[0])) > self.TILT_LIMIT_RAD
            or abs(float(imu[1])) > self.TILT_LIMIT_RAD
        )
        self.unstable_s = self.unstable_s + dt_s if (not finite_imu or tilted) else 0.0

        diagnostics = response.get("diagnostics", {})
        contacts = diagnostics.get("contacts", {})
        requested_finite = len(requested_action) == 4 and all(
            math.isfinite(float(x)) for x in requested_action
        )
        thrust_finite = all(
            math.isfinite(float(diagnostics.get(name, math.nan)))
            for name in ("port_thrust", "starboard_thrust")
        )

        # Frozen precedence among the categories implemented at Gate C.
        if not finite_imu or self.unstable_s >= self.INSTABILITY_HOLD_S - 1e-12:
            return "instability"
        if contacts.get("grounding", False):
            return "grounding"
        if contacts.get("object_collision", False):
            return "object_collision"
        # Vehicle A has no allocator: its normalized commands map directly to
        # bounded thruster setpoints. Only a non-finite command/output is a
        # clean native mapping failure; actuator lag is not misclassified.
        if not requested_finite or not thrust_finite:
            return "allocation_failure"
        return None
