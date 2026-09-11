#!/usr/bin/env python3
"""Evaluate a saved portable feed-forward PPO checkpoint without training it."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
sys.path.insert(0, str(ROOT / "validation/rl-campaign"))

from stable_baselines3 import PPO  # noqa: E402
from train_portable_ppo_feedforward import (  # noqa: E402
    atomic_json,
    evaluate_feedforward,
    make_env,
    write_episode_rows,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("bcod-sim", "gazebo-harmonic", "holoocean", "stonefish"), required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--first-seed", type=int, default=10000)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--runtime-command")
    parser.add_argument("--stonefish-executable", type=Path)
    parser.add_argument("--stonefish-data-dir", type=Path)
    parser.add_argument("--stonefish-lib", type=Path)
    parser.add_argument("--stonefish-deps-lib", type=Path)
    parser.add_argument("--stonefish-physics-threads", type=int, default=1)
    parser.add_argument("--stonefish-sensor-noise", action="store_true")
    parser.add_argument("--holoocean-wind-mode", choices=("off", "surge_equivalent"), default="off")
    parser.add_argument("--disturbance-mode", choices=("zero", "seeded"), default="zero")
    parser.add_argument("--condition-contract", type=Path, help="Content-hashed common-suite nominal contract used for native calibration")
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error("--episodes must be positive")
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        parser.error(f"checkpoint does not exist: {checkpoint}")
    args.base_seed = args.first_seed
    args.fixed_reset_seed = None
    args.final_leg_curriculum = False
    args.diagnostic_only = False
    args.condition_contract_path = args.condition_contract
    model = PPO.load(checkpoint, device=args.device)

    def environment(seed: int):
        args.fixed_reset_seed = seed
        return make_env(args)

    result = evaluate_feedforward(model, environment, args.first_seed, args.episodes)
    args.output.mkdir(parents=True, exist_ok=True)
    write_episode_rows(args.output, result["rows"])
    summary = {key: value for key, value in result.items() if key != "rows"}
    summary.update({
        "schema_version": 1,
        "artifact_kind": "portable-checkpoint-held-out-evaluation",
        "backend": args.backend,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "first_seed": args.first_seed,
        "last_seed": args.first_seed + args.episodes - 1,
        "model_timesteps": int(model.num_timesteps),
        "training_performed": False,
    })
    atomic_json(args.output / "evaluation-summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
