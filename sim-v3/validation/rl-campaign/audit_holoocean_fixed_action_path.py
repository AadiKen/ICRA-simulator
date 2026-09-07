#!/usr/bin/env python3
"""Verify scenario geometry and realized actuators on the reward-audit path."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim.holoocean_vehicle_a_env import HoloOceanVehicleAEnv  # noqa: E402


def action_at(step: int) -> np.ndarray:
    return np.asarray(
        [0.55 * math.sin(0.071 * step), 0.55 * math.sin(0.071 * step + 0.83)],
        dtype=np.float32,
    )


def main() -> int:
    seed = 10000
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=seed, wind_mode="off")
    rows = []
    expected_state = np.zeros(2, dtype=np.float64)
    try:
        observation, reset_info = env.reset()
        requested_spawn = env._start_ned.copy()
        observed_spawn = env._last_gps_position_ned_m.copy()
        route = np.asarray(env._route_ned, dtype=np.float64)
        first_bearing = math.atan2(
            route[0, 1] - requested_spawn[1], route[0, 0] - requested_spawn[0]
        )
        scenario = {
            "seed": seed,
            "requested_spawn_ned_m": requested_spawn.tolist(),
            "observed_initial_gps_ned_m": observed_spawn.tolist(),
            "spawn_position_error_m": float(np.linalg.norm(observed_spawn - requested_spawn)),
            "requested_initial_distance_m": float(np.linalg.norm(route[0] - requested_spawn)),
            "observed_initial_distance_m": float(np.linalg.norm(route[0] - observed_spawn)),
            "first_leg_bearing_deg": math.degrees(first_bearing),
            "requested_heading_deg": math.degrees(env._last_randomization["heading_ned_rad"]),
            "requested_heading_error_deg": math.degrees(
                math.atan2(
                    math.sin(env._last_randomization["heading_ned_rad"] - first_bearing),
                    math.cos(env._last_randomization["heading_ned_rad"] - first_bearing),
                )
            ),
            "observed_sensor_yaw_deg": math.degrees(float(observation[6])),
            "current_nwu_mps": env._last_randomization["current_nwu_mps"],
            "current_applied": reset_info["current_applied"],
            "wind_mode": reset_info["wind_mode"],
            "wind_applied": reset_info["wind_applied"],
        }
        alpha = 1.0 - math.exp(-env.physics_timestep_s / env.MOTOR_TIME_CONSTANT_S)
        for step in range(25):
            action = action_at(step)
            expected_target = np.asarray(action, dtype=np.float64) * env.MAX_THRUST_N
            for _ in range(env.physics_steps_per_action):
                expected_state += (expected_target - expected_state) * alpha
            _, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                raise AssertionError(f"unexpected termination at step {step + 1}")
            target = np.asarray(info["actuator_target_force_n"], dtype=np.float64)
            lagged = np.asarray(info["actuator_lagged_force_n"], dtype=np.float64)
            applied = np.asarray(info["applied_thruster_force_n"], dtype=np.float64)
            rows.append({
                "control_step": step + 1,
                "normalized_action": action.tolist(),
                "expected_target_force_n": expected_target.tolist(),
                "reported_target_force_n": target.tolist(),
                "expected_lagged_force_n": expected_state.tolist(),
                "reported_lagged_force_n": lagged.tolist(),
                "applied_native_force_n": applied.tolist(),
                "target_error_n": float(np.max(np.abs(target - expected_target))),
                "lag_error_n": float(np.max(np.abs(lagged - expected_state))),
                "applied_vs_lagged_error_n": float(np.max(np.abs(applied - lagged))),
            })
    finally:
        env.close()
    maxima = {
        "target_error_n": max(row["target_error_n"] for row in rows),
        "lag_error_n": max(row["lag_error_n"] for row in rows),
        "applied_vs_lagged_error_n": max(row["applied_vs_lagged_error_n"] for row in rows),
    }
    passed = all(value <= 1e-9 for value in maxima.values())
    report = {
        "schema_version": 1,
        "artifact_kind": "holoocean-fixed-action-scenario-and-actuator-path-audit",
        "status": "PASS" if passed else "FAIL",
        "training_run": False,
        "scenario": scenario,
        "actuator": {
            "configured_force_ceiling_n": env.MAX_THRUST_N,
            "configured_time_constant_s": env.MOTOR_TIME_CONSTANT_S,
            "physics_timestep_s": env.physics_timestep_s,
            "physics_steps_per_action": env.physics_steps_per_action,
            "wind_mode": "off",
            "expected_formula": "target=clip(action,-1,1)*501.1328125; x_next=x+(target-x)*(1-exp(-dt/0.25)) on every physics tick",
            "max_errors": maxima,
            "samples": rows,
        },
    }
    output = ROOT / "artifacts/rl-campaign/holoocean-fixed-action-path-audit.json"
    temporary = output.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n")
    temporary.replace(output)
    print(json.dumps({"output": str(output), "status": report["status"], "scenario": scenario, "max_errors": maxima}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
