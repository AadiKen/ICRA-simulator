#!/usr/bin/env python3
"""One PPO entry point for Node, VRX, and Gazebo Harmonic.

External backends deliberately refuse training until their conformance artifact
passes and Gate 5 action fairness is resolved.  ``--diagnostic-only`` permits
wrapper smoke tests, never a policy-training run.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import statistics
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim import CommonWaypointEnv, GazeboGymEnv, VrxGymEnv  # noqa: E402

V7_OUT = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-recurrent-local"
EPISODE_COLUMNS = ("run_id", "seed", "simulator", "vehicle", "task_id",
                   "task_portable", "policy_id", "algorithm", "return", "success",
                   "episode_length", "wall_clock_s", "termination_reason",
                   "collision_type", "host_class")


def detect_host_class():
    """Require an explicit valid class when supplied; recognize SLURM as cluster."""
    configured = os.environ.get("BCOD_HOST_CLASS")
    if configured is not None:
        if configured not in ("local", "cluster", "synthetic"):
            raise RuntimeError("BCOD_HOST_CLASS must be local, cluster, or synthetic")
        return configured
    return "cluster" if os.environ.get("SLURM_JOB_ID") else "local"


def episode_metric_row(*, seed, env, info, total_return, wall_clock_s, policy_id, algorithm):
    reason = str(info["termination_reason"])
    collision = "grounding" if reason == "grounding" else "object" if reason == "collision" else "none"
    backend_type=getattr(env,"backend_type","test");vehicle_preset=getattr(env,"vehicle_preset","test")
    row = {
        "run_id": f"{backend_type}__{vehicle_preset}__{policy_id}__seed-{seed}",
        "seed": int(seed), "simulator": str(backend_type),
        "vehicle": str(vehicle_preset), "task_id": "common-waypoint-transit-v1",
        "task_portable": True, "policy_id": str(policy_id), "algorithm": str(algorithm),
        "return": float(total_return), "success": bool(info["success"]),
        "episode_length": int(info.get("physics_steps",0)), "wall_clock_s": float(wall_clock_s),
        "termination_reason": reason, "collision_type": collision,
        "host_class": detect_host_class(),
    }
    if tuple(row) != EPISODE_COLUMNS:
        raise RuntimeError("episode writer does not match the declared 15-column schema")
    return row


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def bcod_factory(rank: int, curriculum: bool):
    return lambda: CommonWaypointEnv(
        ROOT, base_seed=200_000 + rank * 1_000_000,
        final_leg_curriculum=curriculum,
    )


def make_env(args):
    common = dict(base_seed=args.base_seed, fixed_reset_seed=args.fixed_reset_seed,
                  final_leg_curriculum=args.final_leg_curriculum)
    if args.backend == "bcod-sim":
        return CommonWaypointEnv(ROOT, **common)
    if not args.runtime_command:
        raise SystemExit("--runtime-command is required for an external backend")
    runtime = shlex.split(args.runtime_command)
    cls = VrxGymEnv if args.backend == "vrx" else GazeboGymEnv
    return cls(ROOT, runtime, allow_unconformant_diagnostic=args.diagnostic_only, **common)


def evaluate_recurrent(model, env_factory, first_seed: int, episodes: int):
    """Evaluate without dropping recurrent state between control ticks."""
    rows = []
    for seed in range(first_seed, first_seed + episodes):
        env = env_factory(seed)
        try:
            started = time.perf_counter()
            observation, _ = env.reset()
            lstm_state = None
            episode_start = np.ones((1,), dtype=bool)
            total_return = 0.0
            while True:
                action, lstm_state = model.predict(
                    observation,
                    state=lstm_state,
                    episode_start=episode_start,
                    deterministic=True,
                )
                observation, reward, terminated, truncated, info = env.step(action)
                total_return += float(reward)
                episode_start = np.asarray([terminated or truncated], dtype=bool)
                if terminated or truncated:
                    break
            algorithm = model.__class__.__name__
            rows.append(episode_metric_row(
                seed=seed, env=env, info=info, total_return=total_return,
                wall_clock_s=time.perf_counter() - started,
                policy_id=f"{algorithm}-deterministic", algorithm=algorithm,
            ))
        finally:
            env.close()
    return {
        "episodes": episodes,
        "success_rate": sum(row["success"] for row in rows) / episodes,
        "median_return": float(statistics.median(row["return"] for row in rows)),
        "rows": rows,
    }


def run_v7_protocol():
    """Frozen recurrent Phase 1 gate followed by Phase 2 only on convergence."""
    from sb3_contrib import RecurrentPPO
    from stable_baselines3.common.vec_env import SubprocVecEnv

    V7_OUT.mkdir(parents=True, exist_ok=True)
    phase1_env = SubprocVecEnv([bcod_factory(i, True) for i in range(16)], start_method="fork")
    model = RecurrentPPO(
        "MlpLstmPolicy", phase1_env, seed=7319, n_steps=512, batch_size=512,
        n_epochs=10, learning_rate=3e-4, gamma=.99, gae_lambda=.95,
        clip_range=.2, ent_coef=0,
        policy_kwargs={"net_arch": [128, 128], "lstm_hidden_size": 128,
                       "enable_critic_lstm": True},
        verbose=1, device="cpu",
    )
    curve, consecutive = [], 0
    atomic_json(V7_OUT / "training-state.json", {
        "status": "phase-1-running", "timesteps": 0,
        "algorithm": "RecurrentPPO", "policy": "MlpLstmPolicy",
        "convergence_rule": "success_rate >= 0.9 at two consecutive 250k checkpoints",
    })
    for steps in range(250_000, 1_500_001, 250_000):
        model.learn(total_timesteps=250_000, reset_num_timesteps=False, progress_bar=False)
        model.save(V7_OUT / f"phase1-{steps}")
        result = evaluate_recurrent(
            model,
            lambda seed: CommonWaypointEnv(ROOT, fixed_reset_seed=seed,
                                            final_leg_curriculum=True),
            30000, 50,
        )
        curve.append({"checkpoint_steps": steps, "evaluation": result})
        consecutive = consecutive + 1 if result["success_rate"] >= .9 else 0
        atomic_json(V7_OUT / "phase-1-report.json", {
            "schema_version": 1, "converged": consecutive >= 2, "curve": curve})
        atomic_json(V7_OUT / "training-state.json", {
            "status": "phase-1-running", "timesteps": steps,
            "success_rate": result["success_rate"],
            "consecutive_passing_checkpoints": consecutive,
        })
        print(json.dumps({"checkpoint_steps": steps,
                          "success_rate": result["success_rate"],
                          "consecutive_passing_checkpoints": consecutive}), flush=True)
        if consecutive >= 2:
            break
    phase1_env.close()
    if consecutive < 2:
        atomic_json(V7_OUT / "training-state.json", {
            "status": "phase-1-failed", "timesteps": curve[-1]["checkpoint_steps"],
            "success_rate": curve[-1]["evaluation"]["success_rate"],
            "full_episode_training_started": False,
            "required_next_step": "Run recurrent success/failure heading-error diagnostic against the v6 baseline before any architecture or budget change.",
        })
        return 2

    phase2_env = SubprocVecEnv([bcod_factory(i, False) for i in range(16)], start_method="fork")
    model.set_env(phase2_env)
    model.learn(total_timesteps=3_000_000, reset_num_timesteps=True, progress_bar=False)
    model.save(V7_OUT / "recurrent-ppo-final")
    phase2_env.close()
    evaluation = evaluate_recurrent(
        model, lambda seed: CommonWaypointEnv(ROOT, fixed_reset_seed=seed), 10000, 50)
    atomic_json(V7_OUT / "report.json", {
        "schema_version": 1, "artifact_kind": "surveyor-p3-v7-recurrent-ppo",
        "phase_1_steps": curve[-1]["checkpoint_steps"],
        "phase_1_curve": curve, "phase_2_steps": 3_000_000,
        "evaluation": evaluation,
    })
    atomic_json(V7_OUT / "training-state.json", {
        "status": "completed", "phase_2_started": True,
        "report": str((V7_OUT / "report.json").relative_to(ROOT)),
    })
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("bcod-sim", "vrx", "gazebo-harmonic"), required=True)
    parser.add_argument("--runtime-command", help="Persistent normalized JSONL runtime command")
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--fixed-reset-seed", type=int)
    parser.add_argument("--final-leg-curriculum", action="store_true")
    parser.add_argument("--diagnostic-only", action="store_true",
                        help="Smoke-test an unpassed external port; training remains prohibited")
    parser.add_argument("--smoke-steps", type=int, default=0)
    parser.add_argument("--timesteps", type=int, default=0)
    parser.add_argument("--eval-first-seed", type=int, default=10000)
    parser.add_argument("--eval-episodes", type=int, default=0)
    parser.add_argument("--run-v7-protocol", action="store_true")
    args = parser.parse_args()
    if args.run_v7_protocol:
        if args.backend != "bcod-sim":
            raise SystemExit("The approved v7 protocol currently applies only to bcod-sim")
        if args.timesteps or args.diagnostic_only or args.smoke_steps:
            raise SystemExit("--run-v7-protocol cannot be combined with ad-hoc run options")
        raise SystemExit(run_v7_protocol())
    if args.diagnostic_only and args.timesteps:
        raise SystemExit("diagnostic-only runtimes cannot train a policy")
    env = make_env(args)
    try:
        observation, info = env.reset()
        for _ in range(args.smoke_steps):
            observation, _, terminated, truncated, info = env.step(env.action_space.sample())
            if terminated or truncated:
                observation, info = env.reset()
        if args.timesteps:
            from sb3_contrib import RecurrentPPO
            model = RecurrentPPO(
                "MlpLstmPolicy", env, seed=args.base_seed, n_steps=512, batch_size=512,
                n_epochs=10, learning_rate=3e-4, gamma=.99, gae_lambda=.95,
                clip_range=.2, ent_coef=0,
                policy_kwargs={"net_arch": [128, 128], "lstm_hidden_size": 128,
                               "enable_critic_lstm": True},
                verbose=1, device="cpu",
            )
            model.learn(total_timesteps=args.timesteps, progress_bar=False)
            model.save(ROOT / f"artifacts/rl-campaign/{args.backend}-recurrent-ppo")
            if args.eval_episodes:
                def evaluation_env(seed):
                    eval_args = argparse.Namespace(**vars(args))
                    eval_args.fixed_reset_seed = seed
                    return make_env(eval_args)
                print({"evaluation": evaluate_recurrent(
                    model, evaluation_env, args.eval_first_seed, args.eval_episodes)})
        print({"backend": args.backend, "reset": info, "observation_size": len(observation),
               "smoke_steps": args.smoke_steps, "trained_steps": args.timesteps})
    finally:
        env.close()


if __name__ == "__main__":
    main()
