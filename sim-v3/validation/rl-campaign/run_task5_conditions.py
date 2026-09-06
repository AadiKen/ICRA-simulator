#!/usr/bin/env python3
"""Frozen 250k recurrent-PPO comparison for the three Task 5 conditions."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim import CommonWaypointEnv  # noqa: E402


OUT = ROOT / "artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k"
CONDITIONS = ("control", "noise-zero", "default-noise")


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


class Task5ConditionEnv(CommonWaypointEnv):
    """The frozen 15-field task with exactly one Task 5 condition applied."""
    def __init__(self, *args, condition: str, **kwargs):
        if condition not in CONDITIONS:
            raise ValueError(f"unknown Task 5 condition: {condition}")
        self.task5_condition = condition
        super().__init__(*args, **kwargs)

    def _config(self, seed):
        config = super()._config(seed)
        if self.task5_condition == "noise-zero":
            config["sensors"] = [
                {"plugin": "imu", "enabled": True, "config": {
                    "accel_noise_std_mps2": 0.0,
                    "gyro_noise_std_rad_s": 0.0,
                    "accel_bias_std_mps2": 0.0,
                    "gyro_bias_std_rad_s": 0.0,
                    "accel_drift_std_per_sqrt_s": 0.0,
                    "gyro_drift_std_per_sqrt_s": 0.0,
                    "accel_vibration_sensitivity": 0.0,
                    "gyro_vibration_sensitivity": 0.0,
                }},
                {"plugin": "gps", "enabled": True, "config": {
                    "base_horizontal_std_m": 0.0,
                    "base_vertical_std_m": 0.0,
                    "velocity_std_mps": 0.0,
                }},
            ]
        return config

    def _obs(self):
        observation = super()._obs()
        if self.task5_condition == "control":
            observation[6] = 0.0
        return observation


def factory(rank: int, condition: str):
    return lambda: Task5ConditionEnv(
        ROOT, base_seed=200_000 + rank * 1_000_000,
        final_leg_curriculum=True, condition=condition)


def evaluate(model, condition: str):
    rows = []
    for seed in range(30000, 30050):
        env = Task5ConditionEnv(
            ROOT, fixed_reset_seed=seed, final_leg_curriculum=True,
            condition=condition)
        try:
            observation, _ = env.reset()
            state = None
            episode_start = np.ones((1,), dtype=bool)
            total_return = 0.0
            component_totals = {name: 0.0 for name in (
                "progress", "cross_track", "action_delta", "terminal",
                "base_reward", "potential_shaping")}
            while True:
                action, state = model.predict(
                    observation, state=state, episode_start=episode_start,
                    deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                total_return += float(reward)
                for name in component_totals:
                    component_totals[name] += float(info["reward_components"][name])
                episode_start = np.asarray([terminated or truncated], dtype=bool)
                if terminated or truncated:
                    break
            rows.append({"seed": seed, "success": bool(info["success"]),
                         "return": total_return,
                         "termination_reason": info["termination_reason"],
                         "component_totals": component_totals})
        finally:
            env.close()
    return {
        "first_seed": 30000,
        "last_seed": 30049,
        "episodes": 50,
        "success_rate": sum(row["success"] for row in rows) / 50,
        "median_return": float(statistics.median(row["return"] for row in rows)),
        "median_component_contributions": {
            name: float(statistics.median(row["component_totals"][name] for row in rows))
            for name in rows[0]["component_totals"]},
        "rows": rows,
    }


def output_directory(condition: str, training_seed: int, ent_coef: float = 0.0):
    if ent_coef != 0.0:
        label = f"{ent_coef:g}".replace(".", "p")
        return OUT / f"{condition}-ent-{label}" / f"seed-{training_seed}"
    return OUT / condition if training_seed == 7319 else OUT / condition / f"seed-{training_seed}"


def run(condition: str, training_seed: int, ent_coef: float = 0.0):
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.vec_env import SubprocVecEnv

    output = output_directory(condition, training_seed, ent_coef)
    output.mkdir(parents=True, exist_ok=True)
    env = SubprocVecEnv(
        [factory(rank, condition) for rank in range(16)], start_method="fork")
    model = RecurrentPPO(
        "MlpLstmPolicy", env, seed=training_seed, n_steps=512, batch_size=512,
        n_epochs=10, learning_rate=3e-4, gamma=.99, gae_lambda=.95,
        clip_range=.2, ent_coef=ent_coef,
        policy_kwargs={"net_arch": [128, 128], "lstm_hidden_size": 128,
                       "enable_critic_lstm": True},
        verbose=1, device="cpu")
    atomic_json(output / "training-state.json", {
        "status": "running", "condition": condition, "timesteps": 0,
        "training_seed": training_seed, "ent_coef": ent_coef,
        "algorithm": "RecurrentPPO", "policy": "MlpLstmPolicy",
        "contract_sha256": CommonWaypointEnv.EXPECTED_CONTRACT_SHA256})
    try:
        model.learn(total_timesteps=250_000, progress_bar=False)
        model.save(output / "recurrent-ppo-250k")
    finally:
        env.close()
    result = evaluate(model, condition)
    report = {
        "schema_version": 1,
        "artifact_kind": "task-5-sensor-yaw-250k-condition",
        "condition": condition,
        "contract_sha256": CommonWaypointEnv.EXPECTED_CONTRACT_SHA256,
        "training": {
            "timesteps": 250_000, "seed": training_seed,
            "algorithm": "RecurrentPPO", "policy": "MlpLstmPolicy",
            "n_steps": 512, "batch_size": 512, "n_epochs": 10,
            "learning_rate": 3e-4, "gamma": .99, "gae_lambda": .95,
            "clip_range": .2, "ent_coef": ent_coef,
            "net_arch": [128, 128], "lstm_hidden_size": 128,
            "enable_critic_lstm": True, "final_leg_curriculum": True},
        "evaluation": result,
        "output_directory": str(output.relative_to(ROOT)),
        "phase_2_started": False}
    atomic_json(output / "report.json", report)
    atomic_json(output / "training-state.json", {
        "status": "completed", "condition": condition, "timesteps": 250_000,
        "training_seed": training_seed, "ent_coef": ent_coef,
        "success_rate": result["success_rate"], "phase_2_started": False,
        "report": str((output / "report.json").relative_to(ROOT))})
    print(json.dumps({"condition": condition, "evaluation": result}, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", choices=CONDITIONS, required=True)
    parser.add_argument("--training-seed", type=int, default=7319)
    parser.add_argument("--ent-coef", type=float, default=0.0)
    args = parser.parse_args()
    if args.ent_coef < 0:
        parser.error("--ent-coef must be non-negative")
    run(args.condition, args.training_seed, args.ent_coef)


if __name__ == "__main__":
    main()
