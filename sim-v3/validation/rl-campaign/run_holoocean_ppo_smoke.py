#!/usr/bin/env python3
"""Mechanical RecurrentPPO smoke test for the validated HoloOcean Vehicle A path.

This intentionally runs only one vector rollout (2,048 timesteps with four
environments).  It verifies training, periodic checkpointing, checkpoint reload,
and machine-readable metric output; it is not a learning result.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import sys
import time
from pathlib import Path

import numpy as np

SOURCE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE_ROOT / "packages/python-client"))

from bcod_sim.holoocean_vehicle_a_env import HoloOceanVehicleAEnv  # noqa: E402


class CalmVehicleAEnv(HoloOceanVehicleAEnv):
    """Frozen task randomization with both environmental disturbances disabled."""

    def _draw_randomization(self, seed: int):
        value = super()._draw_randomization(seed)
        value["current_nwu_mps"] = [0.0, 0.0, 0.0]
        value["wind_ned_mps"] = [0.0, 0.0, 0.0]
        return value


def make_env(repository: Path, rank: int, training_seed: int):
    def factory():
        from stable_baselines3.common.monitor import Monitor

        env = CalmVehicleAEnv(
            repository,
            base_seed=training_seed + 1_000_000 * rank,
            wind_mode="off",
        )
        return Monitor(env)

    return factory


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repository", type=Path, default=SOURCE_ROOT)
    parser.add_argument("--seed", type=int, default=7319)
    parser.add_argument("--timesteps", type=int, default=2048)
    parser.add_argument("--n-envs", type=int, default=4)
    args = parser.parse_args()
    if args.n_envs != 4:
        raise ValueError("the frozen smoke-test configuration requires four environments")

    import torch
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import SubprocVecEnv

    if not torch.cuda.is_available():
        raise RuntimeError("smoke test must execute on an allocated CUDA GPU node")

    args.output.mkdir(parents=True, exist_ok=True)
    checkpoint_dir = args.output / "checkpoints"
    log_dir = args.output / "metrics"
    checkpoint_dir.mkdir(exist_ok=True)
    log_dir.mkdir(exist_ok=True)
    env = SubprocVecEnv(
        [make_env(args.repository.resolve(), rank, args.seed) for rank in range(args.n_envs)],
        start_method="fork",
    )
    requested = int(args.timesteps)
    started = time.time()
    try:
        model = RecurrentPPO(
            "MlpLstmPolicy",
            env,
            seed=args.seed,
            n_steps=512,
            batch_size=512,
            n_epochs=10,
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.005,
            policy_kwargs={
                "net_arch": [128, 128],
                "lstm_hidden_size": 128,
                "enable_critic_lstm": True,
            },
            verbose=1,
            device="cuda",
        )
        model.set_logger(configure(str(log_dir), ["stdout", "csv", "json"]))
        callback = CheckpointCallback(
            save_freq=max(1, 2048 // args.n_envs),
            save_path=str(checkpoint_dir),
            name_prefix="holoocean-smoke",
        )
        model.learn(total_timesteps=requested, callback=callback, progress_bar=False)
        # On-policy algorithms record optimization metrics after the rollout's
        # standard dump. Flush that final pending record in a one-rollout smoke.
        model.logger.dump(model.num_timesteps)
        final_path = args.output / "recurrent-ppo-smoke-final"
        model.save(final_path)
        actual = int(model.num_timesteps)
    finally:
        env.close()

    checkpoints = sorted(checkpoint_dir.glob("*.zip"))
    if not checkpoints:
        raise RuntimeError("checkpoint callback did not write a checkpoint")
    load_target = checkpoints[-1]
    loaded = RecurrentPPO.load(load_target, device="cuda")
    probe_observation = np.zeros((1, 15), dtype=np.float32)
    probe_action, _ = loaded.predict(probe_observation, deterministic=True)
    if probe_action.shape != (1, 2) or not np.isfinite(probe_action).all():
        raise RuntimeError("reloaded checkpoint failed deterministic inference probe")

    progress_csv = log_dir / "progress.csv"
    progress_json = log_dir / "progress.json"
    with progress_csv.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    json_rows = [line for line in progress_json.read_text().splitlines() if line.strip()]
    required_columns = {"time/total_timesteps", "time/fps", "train/loss"}
    observed_columns = set().union(*(row.keys() for row in rows)) if rows else set()
    missing = sorted(required_columns - observed_columns)
    if not rows or not json_rows or missing:
        raise RuntimeError(f"training metrics incomplete: missing={missing}")

    result = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-recurrent-ppo-smoke",
        "status": "passed",
        "interpretation": "mechanical smoke test only; not evidence of learning",
        "host": platform.node(),
        "pid": os.getpid(),
        "cuda": {
            "available": True,
            "device_name": torch.cuda.get_device_name(0),
            "torch_version": torch.__version__,
        },
        "environment": {
            "parallel_environments": args.n_envs,
            "wind": "off",
            "ocean_current": "off",
            "observation_fields": 15,
        },
        "training": {
            "algorithm": "RecurrentPPO",
            "policy": "MlpLstmPolicy",
            "seed": args.seed,
            "requested_timesteps": requested,
            "actual_timesteps": actual,
            "wall_clock_s": time.time() - started,
            "n_steps": 512,
            "batch_size": 512,
            "n_epochs": 10,
            "learning_rate": 3e-4,
            "gamma": 0.99,
            "gae_lambda": 0.95,
            "clip_range": 0.2,
            "ent_coef": 0.005,
            "policy_kwargs": {
                "net_arch": [128, 128],
                "lstm_hidden_size": 128,
                "enable_critic_lstm": True,
            },
        },
        "checkpoint": {
            "count": len(checkpoints),
            "files": [str(path) for path in checkpoints],
            "reload_target": str(load_target),
            "reload_num_timesteps": int(loaded.num_timesteps),
            "inference_probe_action": probe_action.tolist(),
            "loadable": True,
        },
        "logging": {
            "csv": str(progress_csv),
            "json": str(progress_json),
            "csv_rows": len(rows),
            "json_rows": len(json_rows),
            "columns": sorted(observed_columns),
            "required_columns_present": True,
        },
    }
    atomic_json(args.output / "smoke-report.json", result)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
