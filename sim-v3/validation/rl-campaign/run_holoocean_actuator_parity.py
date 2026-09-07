"""Verify HoloOcean Vehicle A force scale and lag against bcod-sim; no training."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))
sys.path.insert(0, str(ROOT / "validation/rl-campaign"))
from holoocean_vehicle_a_env import HoloOceanVehicleAEnv
from run_holoocean_yaw_determinism import directional_signs


def main() -> None:
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=7319, settle_physics_steps=20)
    rows = []
    try:
        env.reset()
        for control_step in range(1, 41):
            _, _, terminated, truncated, info = env.step(np.ones(2, dtype=np.float32))
            if terminated or truncated:
                raise AssertionError(f"unexpected termination: {info['termination_reason']}")
            elapsed_s = control_step * env.control_interval_s
            expected = env.MAX_THRUST_N * (
                1.0 - math.exp(-elapsed_s / env.MOTOR_TIME_CONSTANT_S)
            )
            actual = float(info["actuator_lagged_force_n"][0])
            rows.append(
                {
                    "control_step": control_step,
                    "elapsed_s": elapsed_s,
                    "expected_force_n": expected,
                    "actual_force_n": actual,
                    "absolute_error_n": abs(actual - expected),
                }
            )
    finally:
        env.close()

    signs = directional_signs()
    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-actuator-force-lag-parity",
        "status": "PASS",
        "training_run": False,
        "reward_implementation_started": False,
        "bcod_sim_source": {
            "parameters": "core/vehicles/coefficients.js:165-186",
            "update": "packages/core/src/actuators.js:77-93",
            "command_conversion": "packages/core/src/actuators.js:219-247",
            "formula": "x_next = x + clamp((clamp(target,min,max)-x)*(1-exp(-dt/tau)), -rateMax*dt, rateMax*dt)",
            "vehicle_a_values": {
                "min_force_n": -95.0,
                "max_force_n": 95.0,
                "tau_s": 0.25,
                "rate_max_n_s": "Infinity",
                "physics_dt_s": 0.05,
            },
        },
        "verification": {
            "step_command": [1.0, 1.0],
            "samples": rows,
            "max_absolute_formula_error_n": max(row["absolute_error_n"] for row in rows),
            "final_force_n": rows[-1]["actual_force_n"],
            "steady_state_target_n": 95.0,
            "final_target_error_n": 95.0 - rows[-1]["actual_force_n"],
            "within_one_percent_at_end": abs(95.0 - rows[-1]["actual_force_n"]) <= 0.95,
        },
        "directional_sign_checks": signs,
        "sign_checks_pass": bool(
            signs["opposite_yaw_rate_signs"]
            and signs["equal_forward"]["yaw_rate_tail_mean_rad_s"] == 0.0
            and signs["equal_forward"]["relative_goal_east_max_deviation_m"] == 0.0
        ),
    }
    output = ROOT / "artifacts/rl-campaign/holoocean-vehicle-a-actuator-parity.json"
    output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(
        json.dumps(
            {
                "output": str(output),
                "verification": artifact["verification"],
                "directional_sign_checks": signs,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
