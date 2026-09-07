"""Characterize HoloOcean Vehicle A yaw signs and reset repeatability; no training."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import subprocess
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))
from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


SEED = 7319
REPEATS = 8
COMPARE_ACTIONS = [
    *([np.asarray([0.0, 0.0], dtype=np.float32)] * 4),
    *([np.asarray([0.25, 0.25], dtype=np.float32)] * 8),
    *([np.asarray([0.15, 0.35], dtype=np.float32)] * 8),
]


@contextmanager
def unreal_single_thread_mode():
    """Append Unreal's packaged-runtime single-thread flag to Holodeck launches."""
    original_popen = subprocess.Popen

    def single_thread_popen(arguments, *args, **kwargs):
        if (
            isinstance(arguments, list)
            and arguments
            and "Holodeck" in str(arguments[0])
            and "-onethread" not in arguments
        ):
            arguments = [*arguments, "-onethread"]
        return original_popen(arguments, *args, **kwargs)

    subprocess.Popen = single_thread_popen
    try:
        yield
    finally:
        subprocess.Popen = original_popen


def rollout(env: HoloOceanVehicleAEnv, actions: list[np.ndarray]) -> np.ndarray:
    initial, _ = env.reset(seed=SEED)
    values = [initial.copy()]
    for action in actions:
        observation, reward, terminated, truncated, _ = env.step(action)
        if reward != 0.0 or terminated or truncated:
            raise AssertionError("diagnostic unexpectedly entered deferred task logic")
        values.append(observation.copy())
    return np.stack(values)


def characterize(repeats: list[np.ndarray], field_names: tuple[str, ...]) -> dict:
    reference = repeats[0]
    comparisons = []
    maxima = []
    locations = []
    per_field = []
    for index, candidate in enumerate(repeats[1:], start=1):
        difference = np.abs(reference - candidate)
        location = np.unravel_index(int(np.argmax(difference)), difference.shape)
        maximum = float(difference[location])
        maxima.append(maximum)
        locations.append((int(location[0]), int(location[1])))
        field_maxima = np.max(difference, axis=0)
        per_field.append(field_maxima)
        comparisons.append(
            {
                "repeat": index + 1,
                "exact_match": bool(np.array_equal(reference, candidate)),
                "max_abs_difference": maximum,
                "worst_control_step": int(location[0]),
                "worst_field_index": int(location[1]),
                "worst_field_name": field_names[location[1]],
            }
        )
    combined = np.stack(per_field)
    return {
        "repeat_count": len(repeats),
        "sequence_shape": list(reference.shape),
        "all_exact": all(item["exact_match"] for item in comparisons),
        "max_abs_difference_range": [min(maxima), max(maxima)],
        "worst_location_consistent": len(set(locations)) == 1,
        "worst_locations": [
            {"control_step": step, "field_index": field, "field_name": field_names[field]}
            for step, field in sorted(set(locations))
        ],
        "localized_to_first_three_control_steps": all(step <= 3 for step, _ in locations),
        "per_field_max_abs_range": {
            name: [float(combined[:, i].min()), float(combined[:, i].max())]
            for i, name in enumerate(field_names)
        },
        "comparisons_to_repeat_1": comparisons,
    }


def sign_probe(action: np.ndarray, label: str) -> dict:
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=SEED, settle_physics_steps=20)
    seeded_randomization = env._draw_randomization

    def calm_axis_aligned_randomization(seed: int):
        result = seeded_randomization(seed)
        result["heading_ned_rad"] = 0.0
        result["current_nwu_mps"] = [0.0, 0.0, 0.0]
        result["wind_ned_mps"] = [0.0, 0.0, 0.0]
        return result

    env._draw_randomization = calm_axis_aligned_randomization
    try:
        initial, _ = env.reset()
        samples = []
        for _ in range(20):
            observation, _, _, _, _ = env.step(action)
            samples.append(observation)
        values = np.stack(samples)
        tail = values[-5:]
        return {
            "label": label,
            "action": action.tolist(),
            "yaw_rate_tail_mean_rad_s": float(tail[:, 5].mean()),
            "yaw_rate_tail_min_rad_s": float(tail[:, 5].min()),
            "yaw_rate_tail_max_rad_s": float(tail[:, 5].max()),
            "yaw_change_rad": float(values[-1, 6] - initial[6]),
            "relative_goal_east_change_m": float(values[-1, 8] - initial[8]),
            "relative_goal_east_max_deviation_m": float(
                np.max(np.abs(values[:, 8] - initial[8]))
            ),
            "initial_heading_ned_rad": 0.0,
            "current_nwu_mps": [0.0, 0.0, 0.0],
            "wind_ned_mps": [0.0, 0.0, 0.0],
        }
    finally:
        env.close()


def directional_signs() -> dict:
    signs = {
        "port_forward_starboard_reverse": sign_probe(
            np.asarray([0.25, -0.25], dtype=np.float32), "port forward, starboard reverse"
        ),
        "equal_forward": sign_probe(
            np.asarray([0.25, 0.25], dtype=np.float32), "equal forward"
        ),
        "port_reverse_starboard_forward": sign_probe(
            np.asarray([-0.25, 0.25], dtype=np.float32), "port reverse, starboard forward"
        ),
    }
    first_rate = signs["port_forward_starboard_reverse"]["yaw_rate_tail_mean_rad_s"]
    reverse_rate = signs["port_reverse_starboard_forward"]["yaw_rate_tail_mean_rad_s"]
    signs["opposite_yaw_rate_signs"] = bool(first_rate * reverse_rate < 0.0)
    return signs


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/rl-campaign/holoocean-yaw-determinism.json",
    )
    parser.add_argument("--signs-only", action="store_true")
    parser.add_argument("--single-thread-check", action="store_true")
    args = parser.parse_args()

    if args.single_thread_check:
        artifact = json.loads(args.output.read_text())
        baseline_env = HoloOceanVehicleAEnv(ROOT)
        settled_env = HoloOceanVehicleAEnv(ROOT, settle_physics_steps=20)
        try:
            with unreal_single_thread_mode():
                baseline = [rollout(baseline_env, COMPARE_ACTIONS) for _ in range(REPEATS)]
                settled = [rollout(settled_env, COMPARE_ACTIONS) for _ in range(REPEATS)]
        finally:
            baseline_env.close()
            settled_env.close()
        result = {
            "runtime_flag": "-onethread",
            "pre_registered_prediction": "If multi-threaded physics-solver iteration order causes the reset divergence, forcing single-thread execution should collapse it to near-zero, comparable to the float64 machine-epsilon divergence in bcod-sim.",
            "without_settle": characterize(baseline, HoloOceanVehicleAEnv.FIELD_NAMES),
            "with_20_physics_step_settle": characterize(
                settled, HoloOceanVehicleAEnv.FIELD_NAMES
            ),
        }
        previous = artifact["determinism"]
        result["comparison_baseline"] = {
            "without_settle_max_abs_difference_range": previous["without_settle"]["max_abs_difference_range"],
            "with_settle_max_abs_difference_range": previous["with_20_physics_step_settle"]["max_abs_difference_range"],
        }
        result["prediction_outcome"] = (
            "collapsed"
            if result["without_settle"]["max_abs_difference_range"][1] < 1e-10
            and result["with_20_physics_step_settle"]["max_abs_difference_range"][1] < 1e-10
            else "did_not_collapse"
        )
        artifact["determinism"]["single_thread_check"] = result
        args.output.write_text(json.dumps(artifact, indent=2) + "\n")
        print(json.dumps({"output": str(args.output), "single_thread_check": result}, indent=2))
        return

    if args.signs_only:
        artifact = json.loads(args.output.read_text())
        artifact["directional_sign_checks"] = directional_signs()
        magnetometer = artifact.get("magnetometer", {})
        magnetometer.pop("candidate_heading_formula_flu", None)
        magnetometer["candidate_contract_ned_heading_formula_from_flu"] = (
            "atan2(mag_y, mag_x), subject to configured field and mounting calibration"
        )
        magnetometer["policy_observation_changed"] = True
        magnetometer["decision"] = "adopted for policy yaw as atan2(mag_y, mag_x)"
        magnetometer["limitations"] = (
            "simplified sensor: no bias, drift, calibration error, field variation, "
            "or magnetic-interference simulation"
        )
        args.output.write_text(json.dumps(artifact, indent=2) + "\n")
        print(json.dumps({"output": str(args.output), "directional_sign_checks": artifact["directional_sign_checks"]}, indent=2))
        return

    baseline_env = HoloOceanVehicleAEnv(ROOT)
    settled_env = HoloOceanVehicleAEnv(ROOT, settle_physics_steps=20)
    try:
        baseline = [rollout(baseline_env, COMPARE_ACTIONS) for _ in range(REPEATS)]
        settled = [rollout(settled_env, COMPARE_ACTIONS) for _ in range(REPEATS)]
    finally:
        baseline_env.close()
        settled_env.close()

    fields = HoloOceanVehicleAEnv.FIELD_NAMES
    signs = directional_signs()

    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-yaw-and-determinism",
        "status": "DIAGNOSTIC_COMPLETE",
        "training_run": False,
        "gate_d_run": False,
        "termination_logic_touched": False,
        "reward_action_parity_touched": False,
        "contract_sha256": HoloOceanVehicleAEnv.EXPECTED_CONTRACT_SHA256,
        "magnetometer": {
            "available": True,
            "surface_vessel_attachable": True,
            "policy_observation_changed": True,
            "model": "configured global magnetic vector rotated into local sensor frame plus Gaussian Sigma/Cov noise",
            "bias_model": False,
            "candidate_contract_ned_heading_formula_from_flu": "atan2(mag_y, mag_x), subject to configured field and mounting calibration",
            "decision": "adopted for policy yaw as atan2(mag_y, mag_x)",
            "limitations": "simplified sensor: no bias, drift, calibration error, field variation, or magnetic-interference simulation",
        },
        "determinism": {
            "seed": SEED,
            "actions": [action.tolist() for action in COMPARE_ACTIONS],
            "without_settle": characterize(baseline, fields),
            "with_20_physics_step_settle": characterize(settled, fields),
            "settle_duration_s": 20 * HoloOceanVehicleAEnv(ROOT).physics_timestep_s,
            "claim_limit": "tests repeatability for this seed/action trace only; wrapper seeding does not seed Unreal physics",
        },
        "directional_sign_checks": signs,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "without_settle": artifact["determinism"]["without_settle"],
                "with_settle": artifact["determinism"]["with_20_physics_step_settle"],
                "directional_sign_checks": signs,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
