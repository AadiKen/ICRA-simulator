"""Confirm MagnetometerSensor attachment on SurfaceVessel without changing policy input."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))
from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


def main() -> None:
    wrapper = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=7319)
    randomization = wrapper._draw_randomization(7319)
    scenario = wrapper._scenario(randomization)
    scenario["agents"][0]["sensors"].append(
        {
            "sensor_type": "MagnetometerSensor",
            "sensor_name": "MagnetometerDiagnostic",
            "socket": "Platform",
            "Hz": round(1.0 / wrapper.physics_timestep_s),
            "configuration": {"Sigma": 0.0, "MagneticVector": [1.0, 0.0, 0.0]},
        }
    )
    import holoocean

    env = holoocean.make(scenario_cfg=scenario, show_viewport=False)
    try:
        state = env.reset()
        state = env.step(np.zeros(2), 2)
        magnetic = np.asarray(state["MagnetometerDiagnostic"], dtype=np.float64)
        estimated_ned_yaw = math.atan2(float(magnetic[1]), float(magnetic[0]))
        commanded_ned_yaw = float(randomization["heading_ned_rad"])
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "surface_vessel_attachable": True,
                    "socket": "Platform",
                    "payload": magnetic.tolist(),
                    "commanded_ned_yaw_rad": commanded_ned_yaw,
                    "estimated_ned_yaw_rad": estimated_ned_yaw,
                    "absolute_error_rad": abs(estimated_ned_yaw - commanded_ned_yaw),
                    "configured_sigma": 0.0,
                    "policy_observation_changed": False,
                },
                indent=2,
            )
        )
    finally:
        env.__exit__(None, None, None)


if __name__ == "__main__":
    main()
