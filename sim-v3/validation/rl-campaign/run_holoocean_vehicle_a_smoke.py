"""Diagnostic-only HoloOcean Vehicle A wrapper smoke test; never trains."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))

from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


def main() -> None:
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=7319)
    rows = []
    try:
        observation, reset_info = env.reset()
        if observation.shape != (15,) or observation.dtype != np.float32:
            raise AssertionError("reset observation violates the frozen 15-field shape/dtype")
        if set(env._last_state) - {
            "GPSSensor",
            "IMUSensor",
            "MagnetometerSensor",
            "CollisionSensor",
            "t",
        }:
            raise AssertionError("unexpected sensor reached the policy wrapper")
        for _ in range(5):
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            if observation.shape != (15,) or not np.all(np.isfinite(observation)):
                raise AssertionError("step observation is malformed or non-finite")
            rows.append(
                {
                    "action": action.tolist(),
                    "gps_fix_valid": bool(observation[9]),
                    "reward": reward,
                    "terminated": terminated,
                    "truncated": truncated,
                    "physics_steps": info["physics_steps"],
                    "termination_reason": info["termination_reason"],
                    "collision_type": info["collision_type"],
                }
            )
        print(
            json.dumps(
                {
                    "status": "PASS",
                    "diagnostic_only": True,
                    "training_run": False,
                    "contract_sha256": env.EXPECTED_CONTRACT_SHA256,
                    "field_names": env.FIELD_NAMES,
                    "reset_observation_shape": list(observation.shape),
                    "reset_info": reset_info,
                    "configured_policy_sensors": [
                        "GPSSensor",
                        "IMUSensor",
                        "MagnetometerSensor",
                    ],
                    "configured_termination_sensors": ["CollisionSensor", "IMUSensor"],
                    "observed_state_keys": sorted(env._last_state),
                    "steps": rows,
                },
                indent=2,
            )
        )
    finally:
        env.close()


if __name__ == "__main__":
    main()
