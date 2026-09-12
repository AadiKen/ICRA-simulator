#!/usr/bin/env python3
"""Trace HoloOcean observations, policy actions, commands, and trajectory."""
from __future__ import annotations
import argparse, json, subprocess, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "packages/python-client")]
from stable_baselines3 import PPO  # noqa: E402
from bcod_sim.holoocean_vehicle_a_env import HoloOceanVehicleAEnv  # noqa: E402
from eval.common_suite.adapters.holoocean_adapter import HoloOceanAdapter  # noqa: E402

def enable_null_rhi():
    original = subprocess.Popen
    def patched(args, *positional, **keywords):
        if isinstance(args, list) and args and "Holodeck" in str(args[0]):
            args = [*args, "-nullrhi"]
        return original(args, *positional, **keywords)
    subprocess.Popen = patched

def run_episode(model, adapter, label, seed, contract):
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=seed, disturbance_mode="zero",
        wind_mode="off", condition_contract_path=contract)
    try:
        observation, reset_info = env.reset()
        observations, actions, targets, applied, trace = [observation.copy()], [], [], [], []
        initial_position = env._last_gps_position_ned_m.copy()
        step = 0
        while True:
            shared = adapter.native_obs_to_shared(observation)
            action, _ = model.predict(shared, deterministic=True)
            action = np.asarray(action, dtype=np.float64)
            observation, _, terminated, truncated, info = env.step(action)
            step += 1
            observations.append(observation.copy())
            actions.append(action.copy())
            targets.append(info["actuator_target_force_n"])
            applied.append(info["applied_thruster_force_n"])
            if step == 1 or step % 25 == 0 or terminated or truncated:
                trace.append({"step": step, "position_ned_m": env._last_gps_position_ned_m.tolist(),
                    "yaw_ned_rad": float(env._yaw_ned_rad), "relative_goal_m": observation[7:9].tolist(),
                    "action": action.tolist(), "target_force_n": info["actuator_target_force_n"],
                    "applied_force_n": info["applied_thruster_force_n"],
                    "waypoint": info["current_waypoint_index"],
                    "final_distance_m": info["distance_to_final_waypoint_m"],
                    "completion_fraction": info["completion_fraction"]})
            if terminated or truncated:
                break
        obs, act = np.asarray(observations), np.asarray(actions)
        target, native = np.asarray(targets), np.asarray(applied)
        final_position = env._last_gps_position_ned_m.copy()
        return {"label": label, "seed": seed, "contract_binding": reset_info["contract_binding"],
            "steps": step, "termination": info["termination_reason"], "success": bool(info["success"]),
            "initial_position_ned_m": initial_position.tolist(), "final_position_ned_m": final_position.tolist(),
            "displacement_m": float(np.linalg.norm(final_position - initial_position)),
            "waypoints_reached": int(info["waypoints_reached"]),
            "completion_fraction": float(info["completion_fraction"]),
            "observation_shape": list(obs.shape), "observation_finite": bool(np.isfinite(obs).all()),
            "observation_min": obs.min(axis=0).tolist(), "observation_max": obs.max(axis=0).tolist(),
            "gps_valid_fraction": float(obs[:, 9].mean()), "action_min": act.min(axis=0).tolist(),
            "action_max": act.max(axis=0).tolist(), "action_mean": act.mean(axis=0).tolist(),
            "action_abs_mean": np.abs(act).mean(axis=0).tolist(),
            "action_zero_fraction": float(np.mean(np.all(np.abs(act) < 1e-6, axis=1))),
            "action_saturated_fraction": float(np.mean(np.any(np.abs(act) >= .999, axis=1))),
            "target_force_min_n": target.min(axis=0).tolist(), "target_force_max_n": target.max(axis=0).tolist(),
            "applied_force_min_n": native.min(axis=0).tolist(), "applied_force_max_n": native.max(axis=0).tolist(),
            "trace": trace}
    finally:
        env.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--candidate-contract", type=Path, required=True)
    parser.add_argument("--legacy-seed", type=int, default=10010)
    parser.add_argument("--candidate-seed", type=int, default=42000)
    parser.add_argument("--null-rhi", action="store_true")
    args = parser.parse_args()
    if args.null_rhi:
        enable_null_rhi()
    model, adapter = PPO.load(args.checkpoint, device="cpu"), HoloOceanAdapter()
    episodes = [run_episode(model, adapter, "legacy-working-control", args.legacy_seed, None),
        run_episode(model, adapter, "zero-current-nominal-failure", args.candidate_seed,
            args.candidate_contract)]
    print(json.dumps({"schema_version": 1, "episodes": episodes}, indent=2))

if __name__ == "__main__":
    main()
