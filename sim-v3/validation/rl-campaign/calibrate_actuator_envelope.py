#!/usr/bin/env python3
"""Simulator-neutral actuator-envelope probes for the four native evaluation arms."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import subprocess
import sys
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "packages/python-client"), str(ROOT / "validation/rl-campaign")]
from train_portable_ppo_feedforward import make_env  # noqa: E402

DT = 0.1
LEVELS = (0.25, 0.5, 0.75, 1.0)
THRESHOLDS = (0.5, 1.0, 1.5)
ROUTE = np.asarray([[0.0, 0.0], [45.0, 8.0], [90.0, -6.0], [140.0, 12.0]])


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def sample(env, info: dict, observation: np.ndarray) -> dict:
    state = info.get("terminal_state") or {}
    velocity = state.get("velocity_body_mps")
    position = state.get("position_ned_m")
    attitude = state.get("attitude_rad")
    speed = info.get("speed_mps")
    if position is None:
        position = info.get("position_ned_m")
    if velocity is None:
        velocity = info.get("ground_velocity_ned_mps")
    if speed is None and info.get("surge_speed_mps") is not None:
        speed = abs(float(info["surge_speed_mps"]))
    if speed is None and velocity is not None:
        speed = math.hypot(float(velocity[0]), float(velocity[1]))
    if speed is None and hasattr(env, "_ground_velocity_ned_mps"):
        velocity = np.asarray(env._ground_velocity_ned_mps, dtype=float)
        speed = float(np.linalg.norm(velocity))
    if position is None and getattr(env, "_last_gps_position_ned_m", None) is not None:
        position = np.asarray(env._last_gps_position_ned_m, dtype=float)
    if attitude is None:
        yaw = float(observation[6])
    else:
        yaw = float(attitude[2])
    delivered = info.get("applied_thruster_force_n")
    if delivered is None:
        delivered = info.get("policy_propulsion_thrust_n")
    return {
        "speed_mps": None if speed is None else float(speed),
        "yaw_rad": yaw,
        "position_ned_m": None if position is None else [float(position[0]), float(position[1])],
        "delivered_force_n": None if delivered is None else [float(x) for x in delivered],
    }


def environment(args, seed: int):
    args.fixed_reset_seed = seed
    return make_env(args)


def straight_probe(args, level: float, seed: int) -> dict:
    env = environment(args, seed)
    speeds, forces = [], []
    crossings = {str(x): None for x in THRESHOLDS}
    try:
        obs, _ = env.reset()
        for step in range(round(args.max_straight_seconds / DT)):
            obs, _, terminated, truncated, info = env.step(np.asarray([level, level], np.float32))
            row = sample(env, info, obs)
            speeds.append(row["speed_mps"])
            if row["delivered_force_n"] is not None:
                forces.append(row["delivered_force_n"])
            for threshold in THRESHOLDS:
                if crossings[str(threshold)] is None and row["speed_mps"] >= threshold:
                    crossings[str(threshold)] = (step + 1) * DT
            if terminated or truncated:
                break
        window = min(len(speeds), round(10.0 / DT))
        tail = speeds[-window:]
        slope = np.polyfit(np.arange(window) * DT, tail, 1)[0] if window > 1 else math.nan
        force_tail = np.asarray(forces[-window:], dtype=float) if forces else None
        return {
            "command_fraction": level,
            "duration_s": len(speeds) * DT,
            "time_to_speed_s": crossings,
            "steady_state_speed_mps": statistics.fmean(tail),
            "tail_speed_range_mps": max(tail) - min(tail),
            "tail_speed_slope_mps2": float(slope),
            "steady_confirmed": bool(abs(slope) <= args.steady_slope_tolerance),
            "delivered_force_mean_n": None if force_tail is None else force_tail.mean(axis=0).tolist(),
            "delivered_force_range_n": None if force_tail is None else [force_tail.min(axis=0).tolist(), force_tail.max(axis=0).tolist()],
        }
    finally:
        env.close()


def turn_probe(args, target_degrees: float, seed: int) -> dict:
    env = environment(args, seed)
    base = args.turn_base_command
    history = []
    command_history = []
    force_history = []
    try:
        obs, _ = env.reset()
        info = {}
        for _ in range(round(args.turn_steady_seconds / DT)):
            obs, _, terminated, truncated, info = env.step(np.asarray([base, base], np.float32))
            if terminated or truncated:
                break
        start = sample(env, info, obs)
        target_change = math.radians(target_degrees)
        settled_at = None
        reached_at = None
        max_progress = -math.inf
        settle_hold = 0
        previous_yaw = start["yaw_rad"]
        cumulative_yaw = 0.0
        for step in range(round(args.max_turn_seconds / DT)):
            signed_authority = math.copysign(args.turn_authority, target_degrees)
            action = (np.asarray([base - signed_authority, base + signed_authority], np.float32)
                      if reached_at is None else np.asarray([base, base], np.float32))
            # HoloOcean parity checks establish [port forward, starboard reverse]
            # as positive yaw. Reverse the differential sign for a positive target.
            action = (np.asarray([base + signed_authority, base - signed_authority], np.float32)
                      if reached_at is None else np.asarray([base, base], np.float32))
            obs, _, terminated, truncated, info = env.step(action)
            row = sample(env, info, obs)
            command_history.append(action.tolist())
            force_history.append(row["delivered_force_n"])
            yaw_increment = wrap(row["yaw_rad"] - previous_yaw)
            yaw_rate = yaw_increment / DT
            cumulative_yaw += yaw_increment
            previous_yaw = row["yaw_rad"]
            # Project the signed accumulated yaw onto the requested turn
            # direction.  copysign(cumulative_yaw, target_degrees) is wrong
            # for negative targets because it always returns a negative value.
            progress = cumulative_yaw * math.copysign(1.0, target_degrees)
            max_progress = max(max_progress, progress)
            if reached_at is None and progress >= abs(target_change):
                reached_at = (step + 1) * DT
            if reached_at is not None and abs(yaw_rate) <= math.radians(args.settle_rate_degrees_s):
                settle_hold += 1
                if settle_hold >= round(args.settle_hold_seconds / DT) and settled_at is None:
                    settled_at = (step + 1) * DT
            else:
                settle_hold = 0
            history.append(row)
            if settled_at is not None or terminated or truncated:
                break
        end = history[-1]
        distance = None
        if start["position_ned_m"] is not None and end["position_ned_m"] is not None:
            distance = float(np.linalg.norm(np.asarray(end["position_ned_m"]) - np.asarray(start["position_ned_m"])))
        target_abs = abs(math.radians(target_degrees))
        return {
            "target_heading_change_deg": target_degrees,
            "base_command_fraction": base,
            "reach_time_s": reached_at,
            "settle_time_s": settled_at,
            "settled": settled_at is not None,
            "overshoot_deg": math.degrees(max(0.0, max_progress - target_abs)),
            "distance_cost_m": distance,
            "start_speed_mps": start["speed_mps"],
            "end_speed_mps": end["speed_mps"],
            "duration_s": len(history) * DT,
            "commanded_force_first_n": (np.asarray(command_history[0]) * env.max_thrust_n).tolist() if command_history else None,
            "delivered_force_first_n": force_history[0] if force_history else None,
            "delivered_force_last_n": force_history[-1] if force_history else None,
            "differential_command_fraction": float(np.mean([abs(x[0] - x[1]) > 1e-6 for x in command_history])) if command_history else 0.0,
            "differential_delivery_fraction": float(np.mean([f is not None and abs(f[0] - f[1]) > 1e-3 for f in force_history])) if force_history else 0.0,
        }
    finally:
        env.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("bcod-sim", "gazebo-harmonic", "holoocean", "stonefish"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--condition-contract", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=7319)
    parser.add_argument("--max-straight-seconds", type=float, default=100.0)
    parser.add_argument("--steady-slope-tolerance", type=float, default=0.002)
    parser.add_argument("--turn-steady-seconds", type=float, default=15.0)
    parser.add_argument("--max-turn-seconds", type=float, default=30.0)
    parser.add_argument("--turn-base-command", type=float, default=0.35)
    parser.add_argument("--turn-authority", type=float, default=0.3)
    parser.add_argument("--settle-rate-degrees-s", type=float, default=2.0)
    parser.add_argument("--settle-hold-seconds", type=float, default=1.0)
    parser.add_argument("--runtime-command")
    parser.add_argument("--stonefish-executable", type=Path)
    parser.add_argument("--stonefish-data-dir", type=Path)
    parser.add_argument("--stonefish-lib", type=Path)
    parser.add_argument("--stonefish-deps-lib", type=Path)
    parser.add_argument("--stonefish-physics-threads", type=int, default=1)
    args = parser.parse_args()
    if os.environ.get("HOLOOCEAN_NULL_RHI") == "1":
        original_popen = subprocess.Popen
        def null_rhi_popen(command, *positional, **keywords):
            if isinstance(command, list) and command and "Holodeck" in str(command[0]):
                command = [*command, "-nullrhi"]
            return original_popen(command, *positional, **keywords)
        subprocess.Popen = null_rhi_popen
    args.base_seed = args.seed
    args.final_leg_curriculum = False
    args.diagnostic_only = True
    args.allow_unconformant_diagnostic = True
    args.condition_contract_path = args.condition_contract
    args.disturbance_mode = "zero"
    args.holoocean_wind_mode = "off"
    args.stonefish_sensor_noise = False
    route_angles = np.unwrap(np.arctan2(np.diff(ROUTE[:, 1]), np.diff(ROUTE[:, 0])))
    real_turns = np.degrees(np.diff(route_angles)).tolist()
    speed_curve = [straight_probe(args, level, args.seed + i) for i, level in enumerate(LEVELS)]
    targets = [30.0, 90.0, *real_turns]
    turns = [turn_probe(args, target, args.seed + 100 + i) for i, target in enumerate(targets)]
    contract = json.loads(args.condition_contract.read_text())
    report = {
        "schema_version": 1,
        "artifact_kind": "native-actuator-envelope-calibration",
        "backend": args.backend,
        "protocol": {
            "calm_water": True,
            "command_levels": list(LEVELS),
            "straight_duration_s": args.max_straight_seconds,
            "steady_window_s": 10.0,
            "steady_slope_tolerance_mps2": args.steady_slope_tolerance,
            "route_turn_angles_deg": real_turns,
            "turn_controller": {"kind": "fixed differential pulse then equal-thrust coast", "base_command": args.turn_base_command, "differential_authority": args.turn_authority, "settle_rate_degrees_s": args.settle_rate_degrees_s},
        },
        "contract": {"path": str(args.condition_contract.resolve()), "content_sha256": contract.get("content_sha256"), "file_sha256": hashlib.sha256(args.condition_contract.read_bytes()).hexdigest()},
        "speed_curve": speed_curve,
        "turning_probes": turns,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
