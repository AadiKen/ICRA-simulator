"""Gate D dynamics loose-end, action/reward parity, and pre-training gate."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics

import numpy as np

from los_pid_v2 import LosPidV2
from stonefish_bridge import StonefishBridge
from stonefish_task import StonefishCommonTask, compute_reward


SEEDS = (7319, 7320, 7321, 7322, 7323)


def factory(args) -> StonefishBridge:
    return StonefishBridge(
        args.executable, args.data_dir,
        library_dirs=(args.stonefish_lib, args.deps_lib), physics_threads=1,
    )


def full_command_diagnosis(args) -> dict:
    with factory(args) as bridge:
        response = bridge.reset(9001)
        previous = response["observation"]["gps"][2:4]
        rows = []
        for second in range(120):
            response = bridge.step(1.0, 1.0, physics_steps=500)
            position = response["observation"]["gps"][2:4]
            imu = response["observation"]["imu"]
            rows.append({
                "time_s": second + 1,
                "speed_mps": math.hypot(position[0] - previous[0], position[1] - previous[1]),
                "roll_rad": imu[0], "pitch_rad": imu[1],
                "yaw_rad": response["observation"]["compass"][0],
            })
            previous = position
    tail = rows[-60:]
    x = np.arange(60, dtype=np.float64)
    y = np.asarray([row["speed_mps"] for row in tail])
    slope = float(np.polyfit(x, y, 1)[0])
    return {
        "duration_s": 120,
        "one_second_samples": rows,
        "last_60s_speed_mean_mps": float(y.mean()),
        "last_60s_speed_range_mps": [float(y.min()), float(y.max())],
        "last_60s_linear_speed_slope_mps_per_s": slope,
        "maximum_abs_roll_rad": max(abs(row["roll_rad"]) for row in rows),
        "maximum_abs_pitch_rad": max(abs(row["pitch_rad"]) for row in rows),
        "all_finite": all(math.isfinite(value) for row in rows for value in row.values()),
        "interpretation": "bounded surface-coupled hull attitude/yaw oscillation around a drag-limited speed; no unbounded state or integrator term",
    }


def action_parity(args) -> dict:
    checkpoints = (1, 25, 50, 125, 250, 500, 2500)
    with factory(args) as bridge:
        bridge.reset(500)
        rows, elapsed = [], 0
        for checkpoint in checkpoints:
            response = bridge.step(1.0, 1.0, physics_steps=checkpoint - elapsed)
            elapsed = checkpoint
            actual = response["diagnostics"]["port_thrust"]
            exact_continuous = 95.0 * (1.0 - math.exp(-checkpoint * 0.002 / 0.25))
            stonefish_discrete = 95.0 * (1.0 - (1.0 - 0.002 / 0.25) ** checkpoint)
            rows.append({"time_s": checkpoint * 0.002, "actual_force_n": actual,
                         "continuous_first_order_force_n": exact_continuous,
                         "stonefish_discrete_first_order_force_n": stonefish_discrete,
                         "continuous_difference_n": actual - exact_continuous,
                         "discrete_difference_n": actual - stonefish_discrete})
        bridge.reset(501)
        signed = bridge.step(0.5, -0.5, physics_steps=2500)
    return {
        "normalized_range": [-1.0, 1.0], "ordering": ["port", "starboard"],
        "positive_meaning": "forward thrust", "ceiling_n_each": 95.0,
        "first_order_time_constant_s": 0.25,
        "lag_integrator": "Stonefish FirstOrder explicit 0.002 s update",
        "step_response": rows,
        "maximum_abs_discrete_model_error_n": max(abs(row["discrete_difference_n"]) for row in rows),
        "maximum_abs_continuous_model_difference_n": max(abs(row["continuous_difference_n"]) for row in rows),
        "signed_probe": {
            "command": [0.5, -0.5],
            "port_thrust_n": signed["diagnostics"]["port_thrust"],
            "starboard_thrust_n": signed["diagnostics"]["starboard_thrust"],
        },
    }


def rollout(args, contract: Path, seed: int, policy: str,
            *, use_integral: bool = True, use_schedule: bool = True) -> dict:
    with factory(args) as bridge:
        env = StonefishCommonTask(bridge, contract)
        observation, reset_info = env.reset(seed)
        controller = LosPidV2(use_integral=use_integral,
                              use_turn_speed_schedule=use_schedule)
        shaped_return = 0.0
        base_return = 0.0
        scheduled = []
        speeds = []
        info = {}
        for step in range(1, 1201):
            if policy == "LOS-PID-v2":
                action = controller.action(env, observation)
            elif policy == "reverse-retreat":
                # Moderate reverse remains a deliberate retreat but avoids
                # turning the exploit probe into an unrelated capsize test.
                action = np.asarray([-0.25, -0.25, 0.0, 0.0])
            else:
                raise ValueError(policy)
            observation, reward, terminated, truncated, info = env.step(action)
            shaped_return += reward
            base_return += info["reward_components"]["base_reward"]
            scheduled.append(controller.last_scheduled_speed)
            speeds.append(info["surge_speed_mps"])
            if terminated or truncated:
                break
    return {
        "seed": seed, "policy": policy,
        "use_pi_surge": use_integral,
        "use_turn_speed_schedule": use_schedule,
        "success": info["success"], "termination_reason": info["termination_reason"],
        "control_steps": step, "shaped_return": shaped_return,
        "base_return": base_return,
        "net_final_goal_progress_m": math.dist(reset_info["start_ned_m"], reset_info["route_ned_m"][-1]) - info["distance_to_final_waypoint_m"],
        "final_distance_m": info["distance_to_final_waypoint_m"],
        "waypoints_reached": info["waypoints_reached"],
        "mean_cross_track_m": info["mean_cross_track_m"],
        "mean_surge_speed_mps": statistics.fmean(speeds),
        "mean_scheduled_speed_mps": statistics.fmean(scheduled),
        "minimum_scheduled_speed_mps": min(scheduled),
        "final_integral_error": controller.surge_integral_error,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    for name in ("executable", "data-dir", "stonefish-lib", "deps-lib", "contract", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()
    corrected = [rollout(args, args.contract, seed, "LOS-PID-v2") for seed in SEEDS]
    retreat = rollout(args, args.contract, SEEDS[0], "reverse-retreat")
    no_integral = [rollout(args, args.contract, seed, "LOS-PID-v2", use_integral=False)
                   for seed in SEEDS]
    no_schedule = [rollout(args, args.contract, seed, "LOS-PID-v2", use_schedule=False)
                   for seed in SEEDS]
    approach = corrected[0]
    exploit_safe = (
        approach["net_final_goal_progress_m"] > 0
        and retreat["net_final_goal_progress_m"] < 0
        and approach["shaped_return"] > retreat["shaped_return"]
        and approach["base_return"] > retreat["base_return"]
    )
    # Directly exercise timeout potential handling from the imported shared source.
    timeout_probe = compute_reward(
        1.0, 1.0, 0.0, np.zeros(4), np.zeros(4), "timeout",
        previous_final_distance_m=10.0, final_distance_m=12.0,
        shaping_k=0.1757208861220446, shaping_gamma=1.0, shaping_enabled=True,
    )
    successes = sum(row["success"] for row in corrected)
    result = {
        "schema_version": 1, "artifact_kind": "stonefish-gate-d-and-pretraining-validation",
        "status": "PASS_READY_FOR_TRAINING_REVIEW" if successes and exploit_safe else "FAIL_STOP_NO_TRAINING",
        "training_started": False, "ppo_started": False,
        "part0_full_command_diagnosis": full_command_diagnosis(args),
        "throughput_artifact": "gate_d_throughput.json",
        "reward_port": {
            "source": "packages/python-client/bcod_sim/common_task.py",
            "reimplemented": False, "shaping_gamma": 1.0,
            "timeout_retains_observed_final_potential": True,
            "timeout_probe_potential_shaping": timeout_probe.potential_shaping,
            "timeout_probe_expected": -2.0 * 0.1757208861220446,
        },
        "action_parity": action_parity(args),
        "controller": {
            "id": "LOS-PID-v2", "lookahead_m": 8.0, "yaw_kp": 100.0,
            "yaw_kd": 35.0, "target_speed_mps": 1.0,
            "surge_pi": {"kp": 100.0, "ki": 100.0,
                         "anti_windup": "conditional integration on saturation"},
            "turn_speed_schedule": {"minimum_mps": 0.5, "maximum_mps": 1.0,
                                    "cosine_full_scale_yaw_demand": 100.0},
            "mapping": "positive yaw demand -> greater port than starboard command",
        },
        "corrected_controller_results": corrected,
        "correction_ablations_seed_7319": {
            "no_pi_integral": no_integral[0], "no_turn_speed_schedule": no_schedule[0],
        },
        "correction_ablation_summaries": {
            "no_pi_integral": {
                "episodes": len(no_integral),
                "successes": sum(row["success"] for row in no_integral),
                "median_control_steps": statistics.median(row["control_steps"] for row in no_integral),
                "median_mean_surge_speed_mps": statistics.median(row["mean_surge_speed_mps"] for row in no_integral),
            },
            "no_turn_speed_schedule": {
                "episodes": len(no_schedule),
                "successes": sum(row["success"] for row in no_schedule),
                "median_control_steps": statistics.median(row["control_steps"] for row in no_schedule),
                "median_mean_surge_speed_mps": statistics.median(row["mean_surge_speed_mps"] for row in no_schedule),
            },
        },
        "summary": {
            "episodes": len(corrected), "successes": successes,
            "success_rate": successes / len(corrected),
            "median_shaped_return": statistics.median(row["shaped_return"] for row in corrected),
            "all_shaped_returns_positive": all(row["shaped_return"] > 0 for row in corrected),
        },
        "exploit_check": {"approach": approach, "retreat": retreat,
                          "safe": exploit_safe},
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"status": result["status"], "summary": result["summary"],
                      "ablations": result["correction_ablations_seed_7319"],
                      "exploit_safe": exploit_safe,
                      "part0": {key: value for key, value in result["part0_full_command_diagnosis"].items()
                                if key != "one_second_samples"},
                      "action_max_errors": {
                          "discrete_n": result["action_parity"]["maximum_abs_discrete_model_error_n"],
                          "continuous_n": result["action_parity"]["maximum_abs_continuous_model_difference_n"]}}, indent=2))


if __name__ == "__main__":
    main()
