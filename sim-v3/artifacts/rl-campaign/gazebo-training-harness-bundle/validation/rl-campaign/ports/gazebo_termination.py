"""Gate C termination contract for the Gazebo runtime."""
from __future__ import annotations

import math

ROLL_PITCH_LIMIT_RAD=math.radians(60)
INSTABILITY_HOLD_S=1.0
ALLOCATION_RELATIVE_ERROR=0.20
ALLOCATION_HOLD_S=1.0  # ten 0.1 s control intervals

def _names(value):
    if isinstance(value,str):
        return [value]
    if isinstance(value,list):
        return [name for child in value for name in _names(child)]
    if isinstance(value,dict):
        found=[]
        for key,child in value.items():
            if key.lower() in ("collision1","collision2","collision","name"):
                found.extend(_names(child))
            elif isinstance(child,(dict,list)):
                found.extend(_names(child))
        return found
    return []

def classify_contacts(messages):
    """Classify hull contacts; bathymetry/ground always beats object contact."""
    names=[name.lower() for name in _names(messages)]
    external=[name for name in names if "surveyor" not in name and "hull_collision" not in name]
    if any(any(token in name for token in ("bathymetry","seabed","ground","bottom")) for name in external):
        return "grounding"
    return "object_collision" if external else None

class TerminationMonitor:
    def __init__(self):
        self.unstable_s=0.0
        self.allocation_bad_s=0.0

    def update(self,*,dt_s,roll_rad,pitch_rad,contact_messages,commanded_thrust_n,achieved_thrust_n):
        finite_state=all(math.isfinite(x) for x in (roll_rad,pitch_rad))
        unstable=(not finite_state) or abs(roll_rad)>ROLL_PITCH_LIMIT_RAD or abs(pitch_rad)>ROLL_PITCH_LIMIT_RAD
        self.unstable_s=self.unstable_s+dt_s if unstable else 0.0

        contact=classify_contacts(contact_messages)
        finite_allocation=all(math.isfinite(x) for x in (*commanded_thrust_n,*achieved_thrust_n))
        requested=math.hypot(*commanded_thrust_n) if finite_allocation else math.inf
        residual=math.hypot(*(a-b for a,b in zip(commanded_thrust_n,achieved_thrust_n))) if finite_allocation else math.inf
        allocation_bad=(not finite_allocation) or (requested>1e-12 and residual/requested>ALLOCATION_RELATIVE_ERROR)
        self.allocation_bad_s=self.allocation_bad_s+dt_s if allocation_bad else 0.0

        # Frozen contract precedence.
        if not finite_state or self.unstable_s>=INSTABILITY_HOLD_S-1e-12:return "instability"
        if contact=="grounding":return "grounding"
        if contact=="object_collision":return "object_collision"
        if not finite_allocation or self.allocation_bad_s>=ALLOCATION_HOLD_S-1e-12:return "allocation_failure"
        return None
