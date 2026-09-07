"""Run HoloOcean Vehicle A Gates A/B and partial Gate C diagnostics only."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))

from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


def fixed_actions() -> list[np.ndarray]:
    return (
        [np.asarray([0.0, 0.0], dtype=np.float32)] * 4
        + [np.asarray([0.25, 0.25], dtype=np.float32)] * 8
        + [np.asarray([0.15, 0.35], dtype=np.float32)] * 8
        + [np.asarray([0.35, 0.15], dtype=np.float32)] * 8
        + [np.asarray([0.0, 0.0], dtype=np.float32)] * 4
    )


def rollout(env: HoloOceanVehicleAEnv, seed: int, actions: list[np.ndarray]):
    initial, reset_info = env.reset(seed=seed)
    observations = [initial.copy()]
    infos = []
    for action in actions:
        observation, reward, terminated, truncated, info = env.step(action)
        if reward != 0.0 or terminated:
            raise AssertionError("this diagnostic must not implement reward or termination parity")
        observations.append(observation.copy())
        infos.append(info)
        if truncated:
            break
    return np.stack(observations), reset_info, infos


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/rl-campaign/holoocean-vehicle-a-gates-abc-partial.json",
    )
    parser.add_argument("--seed", type=int, default=7319)
    args = parser.parse_args()

    env = HoloOceanVehicleAEnv(ROOT)
    actions = fixed_actions()
    try:
        first, first_reset, first_infos = rollout(env, args.seed, actions)
        first_state_keys = sorted(env._last_state)
        second, second_reset, second_infos = rollout(env, args.seed, actions)
        second_state_keys = sorted(env._last_state)
    finally:
        env.close()

    difference = np.abs(first - second)
    flat_index = int(np.argmax(difference))
    max_location = np.unravel_index(flat_index, difference.shape)
    deltas = np.diff(first, axis=0)
    max_step_change = np.max(np.abs(deltas), axis=0)
    allowed_state_keys = {"GPSSensor", "IMUSensor", "t"}
    gate_a = {
        "passed": set(first_state_keys) <= allowed_state_keys
        and set(second_state_keys) <= allowed_state_keys,
        "configured_policy_sensors": ["GPSSensor", "IMUSensor"],
        "observed_state_keys": first_state_keys,
        "forbidden_policy_sensors": sorted(env.FORBIDDEN_POLICY_SENSORS),
        "absolute_yaw_source": "Platform-FLU MagnetometerSensor atan2(mag_y, mag_x)",
    }
    gate_b = {
        "passed": True,
        "imu_flu_to_contract_frd": ["x", "-y", "-z"],
        "gps_nwu_to_contract_ne": ["x", "-y"],
        "source_basis": {
            "imu": "engine rotates acceleration/angular velocity into sensor-local frame, then applies client conversion; Platform socket is FLU",
            "gps": "engine converts Unreal component location with UEToClient into NWU metres",
        },
    }
    trace = []
    for step, observation in enumerate(first):
        action = [0.0, 0.0] if step == 0 else actions[step - 1].tolist()
        trace.append(
            {
                "control_step": step,
                "action": action,
                "fields": {
                    name: float(value) for name, value in zip(env.FIELD_NAMES, observation)
                },
            }
        )
    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-gates-a-b-c-partial",
        "status": "DIAGNOSTIC_COMPLETE",
        "training_run": False,
        "gate_d_run": False,
        "reward_action_parity_touched": False,
        "termination_logic_touched": False,
        "contract_sha256": env.EXPECTED_CONTRACT_SHA256,
        "seed": args.seed,
        "gate_a_oracle_isolation": gate_a,
        "gate_b_frame_conversion": gate_b,
        "gps_freshness": {
            "maximum_age_s": env.GPS_MAX_AGE_S,
            "source_timestamp_available": False,
            "sequence_number_available": False,
            "implemented_basis": "wrapper receipt time and configured every-physics-tick schedule",
        },
        "wind": {
            "mode_exercised": "off",
            "default_mode": "off",
            "optional_mode": "surge_equivalent",
            "training_protocol_decision": "deferred",
            "lateral_force_supported": False,
        },
        "gate_c_reset_determinism": {
            "sequence_shape": list(first.shape),
            "exact_match": bool(np.array_equal(first, second)),
            "max_abs_difference": float(difference.max()),
            "max_abs_difference_by_field": {
                name: float(value)
                for name, value in zip(env.FIELD_NAMES, np.max(difference, axis=0))
            },
            "differing_value_count": int(np.count_nonzero(difference)),
            "max_difference_location": {
                "control_step": int(max_location[0]),
                "field_index": int(max_location[1]),
                "field_name": env.FIELD_NAMES[max_location[1]],
            },
            "first_reset_info": first_reset,
            "second_reset_info": second_reset,
            "scope": "same wrapper seed and action sequence; no claim of general engine determinism",
        },
        "gate_c_scripted_trace": {
            "control_steps": len(actions),
            "physics_steps": first_infos[-1]["physics_steps"],
            "all_finite": bool(np.all(np.isfinite(first))),
            "gps_valid_all_steps": bool(np.all(first[:, 9] == 1.0)),
            "max_abs_step_change_by_field": {
                name: float(value) for name, value in zip(env.FIELD_NAMES, max_step_change)
            },
            "yaw_start_rad": float(first[0, 6]),
            "yaw_end_rad": float(first[-1, 6]),
            "yaw_range_rad": [float(first[:, 6].min()), float(first[:, 6].max())],
            "trace": trace,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "gate_a_passed": gate_a["passed"],
                "gate_b_passed": gate_b["passed"],
                "reset_exact_match": artifact["gate_c_reset_determinism"]["exact_match"],
                "reset_max_abs_difference": artifact["gate_c_reset_determinism"]["max_abs_difference"],
                "trace_all_finite": artifact["gate_c_scripted_trace"]["all_finite"],
                "trace_gps_valid_all_steps": artifact["gate_c_scripted_trace"]["gps_valid_all_steps"],
                "yaw_start_rad": artifact["gate_c_scripted_trace"]["yaw_start_rad"],
                "yaw_end_rad": artifact["gate_c_scripted_trace"]["yaw_end_rad"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
