"""Pre-training HoloOcean reward validation with LOS-PID-v2 and retreat control."""
from __future__ import annotations

import json
import math
from pathlib import Path
import statistics
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))
from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


SEEDS = [7319, 7320, 7321, 7322, 7323]
GAINS = {"lookahead": 8.0, "kp": 100.0, "kd": 35.0, "speed": 1.0}
THRUST_CEILING_N = HoloOceanVehicleAEnv.MAX_THRUST_N
SURGE_KI = 100.0
MIN_TURN_SPEED_MPS = 0.5
YAW_SCHEDULE_FULL_SCALE = 100.0
FAILED_RUN_BASELINE = {
    "episodes": 5,
    "successes": 0,
    "success_rate": 0.0,
    "median_shaped_return": -114.10937924152442,
}


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def scheduled_speed_mps(yaw_demand: float) -> float:
    fraction = min(1.0, abs(yaw_demand) / YAW_SCHEDULE_FULL_SCALE)
    blend = 0.5 * (1.0 + math.cos(math.pi * fraction))
    return MIN_TURN_SPEED_MPS + (GAINS["speed"] - MIN_TURN_SPEED_MPS) * blend


def pi_surge_force(
    speed_mps: float,
    state: dict[str, float],
    dt_s: float,
    target_speed_mps: float = GAINS["speed"],
) -> float:
    error = target_speed_mps - speed_mps
    candidate_integral = state["surge_integral_error"] + error * dt_s
    candidate_force = GAINS["kp"] * error + SURGE_KI * candidate_integral
    saturated_force = clamp(candidate_force, -THRUST_CEILING_N, THRUST_CEILING_N)
    drives_further_into_saturation = candidate_force != saturated_force and (
        (candidate_force > 0.0 and error > 0.0)
        or (candidate_force < 0.0 and error < 0.0)
    )
    if not drives_further_into_saturation:
        state["surge_integral_error"] = candidate_integral
    force = (
        GAINS["kp"] * error
        + SURGE_KI * state["surge_integral_error"]
    )
    return clamp(force, -THRUST_CEILING_N, THRUST_CEILING_N)


def los_pid_v2(
    env: HoloOceanVehicleAEnv,
    observation: np.ndarray,
    controller_state: dict[str, float],
) -> np.ndarray:
    position = env._last_gps_position_ned_m
    if position is None:
        return np.zeros(2, dtype=np.float32)
    leg_start = env._start_ned if env._waypoint == 0 else np.asarray(env._route_ned[env._waypoint - 1])
    goal = np.asarray(env._route_ned[env._waypoint])
    dn, de = goal - leg_start
    length = math.hypot(float(dn), float(de))
    cn, ce = float(dn) / length, float(de) / length
    cross = -ce * (float(position[0]) - float(leg_start[0])) + cn * (
        float(position[1]) - float(leg_start[1])
    )
    desired = math.atan2(ce, cn) - math.atan2(cross, GAINS["lookahead"])
    heading_error = wrap(desired - float(observation[6]))
    forward = np.asarray(
        [math.cos(float(observation[6])), math.sin(float(observation[6]))]
    )
    surge_speed = float(np.dot(env._ground_velocity_ned_mps, forward))
    yaw = clamp(
        GAINS["kp"] * heading_error - GAINS["kd"] * float(observation[5]),
        -100.0,
        100.0,
    )
    target_speed = scheduled_speed_mps(yaw)
    controller_state["scheduled_speed_mps"] = target_speed
    surge = pi_surge_force(
        surge_speed,
        controller_state,
        env.control_interval_s,
        target_speed,
    )
    return np.asarray(
        [clamp((surge + yaw) / THRUST_CEILING_N, -1.0, 1.0), clamp((surge - yaw) / THRUST_CEILING_N, -1.0, 1.0)],
        dtype=np.float32,
    )


def calm_environment(seed: int) -> HoloOceanVehicleAEnv:
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=seed, settle_physics_steps=20)
    original = env._draw_randomization

    def calm_randomization(value: int):
        result = original(value)
        result["current_nwu_mps"] = [0.0, 0.0, 0.0]
        result["wind_ned_mps"] = [0.0, 0.0, 0.0]
        return result

    env._draw_randomization = calm_randomization
    return env


def rollout(seed: int, policy: str) -> dict:
    env = calm_environment(seed)
    shaped_return = 0.0
    base_return = 0.0
    speeds_mps: list[float] = []
    controller_state = {"surge_integral_error": 0.0, "scheduled_speed_mps": GAINS["speed"]}
    scheduled_speeds_mps: list[float] = []
    try:
        observation, _ = env.reset()
        initial_first_distance = env._distance_to_waypoint(0)
        initial_final_distance = env._distance_to_waypoint(len(env._route_ned) - 1)
        info = {}
        for control_step in range(1, math.ceil(env.timeout_steps / env.physics_steps_per_action) + 1):
            if policy == "LOS-PID-v2":
                action = los_pid_v2(env, observation, controller_state)
            elif policy == "full-forward":
                action = np.asarray([1.0, 1.0], dtype=np.float32)
            else:
                action = np.asarray([-1.0, -1.0], dtype=np.float32)
            observation, reward, terminated, truncated, info = env.step(action)
            speeds_mps.append(float(np.linalg.norm(env._ground_velocity_ned_mps)))
            scheduled_speeds_mps.append(controller_state["scheduled_speed_mps"])
            shaped_return += float(reward)
            base_return += float(info["reward_components"]["base_reward"])
            if terminated or truncated:
                break
        return {
            "seed": seed,
            "policy": policy,
            "success": bool(info["success"]),
            "termination_reason": info["termination_reason"],
            "control_steps": control_step,
            "initial_first_waypoint_distance_m": initial_first_distance,
            "final_first_waypoint_distance_m": env._distance_to_waypoint(0),
            "initial_final_distance_m": initial_final_distance,
            "final_distance_m": float(info["distance_to_final_waypoint_m"]),
            "net_final_goal_progress_m": initial_final_distance
            - float(info["distance_to_final_waypoint_m"]),
            "shaped_return": shaped_return,
            "base_return": base_return,
            "waypoints_reached": int(info["waypoints_reached"]),
            "mean_cross_track_m": float(info["mean_cross_track_m"]),
            "speed_profile_mps": {
                "mean": statistics.fmean(speeds_mps),
                "maximum": max(speeds_mps),
                "final": speeds_mps[-1],
                "mean_last_10_s": statistics.fmean(speeds_mps[-100:]),
            },
            "final_surge_integral_error_m": controller_state["surge_integral_error"],
            "scheduled_speed_profile_mps": {
                "mean": statistics.fmean(scheduled_speeds_mps),
                "minimum": min(scheduled_speeds_mps),
                "final": scheduled_speeds_mps[-1],
            },
        }
    finally:
        env.close()


def speed_step_probe(seed: int = SEEDS[0]) -> dict:
    env = calm_environment(seed)
    controller_state = {"surge_integral_error": 0.0}
    speeds: list[float] = []
    forces: list[float] = []
    try:
        env.reset()
        for _ in range(round(40.0 / env.control_interval_s)):
            speed = float(np.linalg.norm(env._ground_velocity_ned_mps))
            force = pi_surge_force(speed, controller_state, env.control_interval_s)
            env.step(np.asarray([force, force], dtype=np.float32) / THRUST_CEILING_N)
            speeds.append(float(np.linalg.norm(env._ground_velocity_ned_mps)))
            forces.append(force)
    finally:
        env.close()
    settling_index = next(
        (
            index
            for index in range(len(speeds))
            if all(abs(value - 1.0) <= 0.02 for value in speeds[index:])
        ),
        None,
    )
    return {
        "target_speed_mps": 1.0,
        "duration_s": 40.0,
        "final_speed_mps": speeds[-1],
        "mean_last_5_s_mps": statistics.fmean(speeds[-50:]),
        "maximum_speed_mps": max(speeds),
        "overshoot_percent": max(0.0, max(speeds) - 1.0) * 100.0,
        "settling_time_2_percent_s": None if settling_index is None else settling_index * env.control_interval_s,
        "final_force_n_per_thruster": forces[-1],
        "maximum_force_n_per_thruster": max(forces),
        "final_integral_error_m": controller_state["surge_integral_error"],
    }


def main() -> None:
    step_response = speed_step_probe()
    controller = [rollout(seed, "LOS-PID-v2") for seed in SEEDS]
    retreat = rollout(SEEDS[0], "reverse-retreat")
    full_forward = rollout(SEEDS[0], "full-forward")
    paired_approach = controller[0]
    successes = [row for row in controller if row["success"]]
    exploit_safe = bool(
        paired_approach["net_final_goal_progress_m"] > 0.0
        and retreat["net_final_goal_progress_m"] < 0.0
        and paired_approach["shaped_return"] > retreat["shaped_return"]
        and paired_approach["base_return"] > retreat["base_return"]
    )
    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-reward-pretraining-validation",
        "status": "PASS" if successes and exploit_safe else "FAIL_STOP_NO_TRAINING",
        "training_run": False,
        "ppo_started": False,
        "reward_source": {
            "implementation": "packages/python-client/bcod_sim/common_task.py:44-87",
            "shaping_gamma": 1.0,
            "shaping_k": controller and HoloOceanVehicleAEnv(ROOT).contract["reward"]["potential_shaping"]["k"],
            "timeout_keeps_nonabsorbing_potential": True,
        },
        "controller": {
            "id": "LOS-PID-v2",
            "source": "validation/rl-campaign/ports/portable-controllers.ts:9-37",
            "gains": GAINS,
            "state_adapter": "GPS position/difference-derived ground velocity, magnetometer yaw, IMU yaw rate; no privileged pose/velocity sensor",
            "environment": "calm water; randomized route rotation, spawn offset, and initial heading retained",
            "adaptation_finding": f"HoloOcean-only actuator mapping uses port-minus-starboard for positive NED yaw and the wrapper's performance-calibrated +/-{THRUST_CEILING_N:g} N per-thruster scale; the shared portable controller remains unchanged",
            "thrust_calibration": {
                "ceiling_n_per_thruster": THRUST_CEILING_N,
                "target_cruise_speed_mps": 1.0,
                "artifact": "artifacts/rl-campaign/holoocean-vehicle-a-thrust-calibration.json",
                "methodological_decision": "Literal +/-95 N force matching was tested first and abandoned because packaged HoloOcean mass and damping cannot be matched to bcod-sim; task-performance-envelope calibration replaces raw-force parity.",
            },
            "surge_force_law": f"clamp(100 * speed_error + {SURGE_KI:g} * integral(speed_error), +/-{THRUST_CEILING_N:g} N)",
            "surge_pi": {
                "kp_n_per_mps": GAINS["kp"],
                "ki_n_per_m": SURGE_KI,
                "anti_windup": "conditional integration: freeze when saturated and speed error would drive farther into saturation",
            },
            "turn_speed_schedule": {
                "formula": "v_min + (v_max-v_min) * 0.5 * (1 + cos(pi * min(1, abs(yaw_demand)/100)))",
                "maximum_speed_mps": GAINS["speed"],
                "minimum_speed_mps": MIN_TURN_SPEED_MPS,
                "full_scale_yaw_demand": YAW_SCHEDULE_FULL_SCALE,
                "scope": "HoloOcean-local only",
            },
        },
        "speed_step_response": step_response,
        "controller_results": controller,
        "failed_run_baseline": FAILED_RUN_BASELINE,
        "summary": {
            "episodes": len(controller),
            "successes": len(successes),
            "success_rate": len(successes) / len(controller),
            "median_shaped_return": statistics.median(row["shaped_return"] for row in controller),
            "successful_shaped_returns": [row["shaped_return"] for row in successes],
            "all_successful_shaped_returns_positive": bool(successes) and all(row["shaped_return"] > 0 for row in successes),
        },
        "exploit_check": {
            "seed": SEEDS[0],
            "approach": paired_approach,
            "reverse_retreat": retreat,
            "safe": exploit_safe,
            "criterion": "approach makes positive net final-goal progress, retreat makes negative progress, and approach exceeds retreat in both shaped and base return",
        },
        "feasibility_probe": {
            "policy": "sustained equal full-forward command; diagnostic only",
            "result": full_forward,
        },
    }
    output = ROOT / "artifacts/rl-campaign/holoocean-vehicle-a-reward-validation.json"
    output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(json.dumps(artifact, indent=2))


if __name__ == "__main__":
    main()
