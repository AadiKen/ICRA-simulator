#!/usr/bin/env python3
"""One instrumented feedforward PPO entry point for bcod-sim, Gazebo, and HoloOcean.

External backends deliberately refuse training until their conformance artifact
passes and Gate 5 action fairness is resolved.  ``--diagnostic-only`` permits
wrapper smoke tests, never a policy-training run.
"""
from __future__ import annotations

import argparse
import csv
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import statistics
import subprocess
import sys
import time

import gymnasium as gym
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
sys.path.insert(0, str(ROOT / "stonefish/python"))

from bcod_sim import (CommonWaypointEnv, GazeboGymEnv, HoloOceanVehicleAEnv,
                      VrxGymEnv)  # noqa: E402
from stonefish_gym_env import StonefishGymEnv  # noqa: E402

FEEDFORWARD_PROTOCOL_OUT = ROOT / "artifacts/rl-campaign/surveyor/portable-feedforward-local"
EPISODE_COLUMNS = ("run_id", "seed", "simulator", "vehicle", "task_id",
                   "task_portable", "policy_id", "algorithm", "return", "success",
                   "episode_length", "wall_clock_s", "termination_reason",
                   "collision_type", "host_class")

BCOD_PROTOCOL_ARTIFACT = (
    ROOT / "artifacts/rl-campaign/surveyor/15-field-rerun-preregistration.json"
)
FEEDFORWARD_PROTOCOL_ARTIFACT = (
    ROOT / "artifacts/rl-campaign/surveyor/portable-feedforward-training-protocol.json"
)
FROZEN_CONTRACT_SHA256 = "2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36"
REWARD_COMPONENT_COLUMNS = (
    "episode_index", "episode_seed", "control_step", "physics_steps",
    "progress", "cross_track", "action_delta", "terminal", "base_reward",
    "potential_shaping", "shaped_reward", "completion_fraction", "success",
    "waypoints_reached", "termination_reason", "terminated", "truncated",
)
ALIGNED_EVALUATION_COLUMNS = (
    "checkpoint_timesteps", "actual_model_timesteps", "episodes",
    "median_return", "mean_return", "success_rate",
)
_SHARED_ALGORITHM_CONFIG = {
    "algorithm": "PPO",
    "policy": "MlpPolicy",
    "seed": 7319,
    "n_steps": 512,
    "batch_size": 512,
    "n_epochs": 10,
    "learning_rate": 3e-4,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_range": 0.2,
    "policy_kwargs": {
        "net_arch": [128, 128],
    },
}


def algorithm_config(backend: str) -> dict:
    """Return the one immutable algorithm configuration for every simulator.

    ``backend`` is accepted so callers cannot accidentally select a separate
    Gazebo configuration.  It intentionally does not affect the result.
    The entropy coefficient is read from the preregistered bcod-sim protocol,
    rather than being copied into this training entry point.
    """
    if backend not in ("bcod-sim", "vrx", "gazebo-harmonic", "holoocean", "stonefish"):
        raise ValueError(f"unsupported backend: {backend}")
    protocol = json.loads(FEEDFORWARD_PROTOCOL_ARTIFACT.read_text())
    factors = protocol.get("frozen_run_factors", {})
    source_protocol = json.loads(BCOD_PROTOCOL_ARTIFACT.read_text())
    source_factors = source_protocol.get("frozen_run_factors", {})
    if protocol.get("task_contract_content_sha256") != FROZEN_CONTRACT_SHA256:
        raise RuntimeError("feedforward protocol does not match the frozen task contract")
    if factors.get("algorithm") != "PPO" or factors.get("policy") != "MlpPolicy":
        raise RuntimeError("feedforward protocol algorithm does not match the portable harness")
    if source_protocol.get("status") != "PREREGISTERED_BEFORE_ANY_15_FIELD_RUN":
        raise RuntimeError("bcod-sim protocol artifact is not the frozen 15-field preregistration")
    if float(factors["ent_coef"]) != float(source_factors["ent_coef"]):
        raise RuntimeError("feedforward entropy coefficient differs from the recurrent baseline")
    config = deepcopy(_SHARED_ALGORITHM_CONFIG)
    config["ent_coef"] = float(source_factors["ent_coef"])
    return config


def algorithm_config_bytes(backend: str) -> bytes:
    """Canonical bytes used by tests and run metadata to prove parity."""
    return json.dumps(algorithm_config(backend), sort_keys=True,
                      separators=(",", ":")).encode("utf-8")


def ppo_kwargs(backend: str) -> tuple[str, dict]:
    config = algorithm_config(backend)
    policy = config.pop("policy")
    algorithm = config.pop("algorithm")
    if algorithm != "PPO":
        raise RuntimeError("portable harness only supports PPO")
    return policy, config


def algorithm_provenance(backend: str) -> dict:
    payload = algorithm_config_bytes(backend)
    return {
        "config": algorithm_config(backend),
        "canonical_sha256": hashlib.sha256(payload).hexdigest(),
        "ent_coef_source": str(BCOD_PROTOCOL_ARTIFACT.relative_to(ROOT)),
        "execution_protocol": str(FEEDFORWARD_PROTOCOL_ARTIFACT.relative_to(ROOT)),
        "contract_sha256": FROZEN_CONTRACT_SHA256,
    }


def detect_host_class():
    """Require an explicit valid class when supplied; recognize SLURM as cluster."""
    configured = os.environ.get("BCOD_HOST_CLASS")
    if configured is not None:
        if configured not in ("local", "cluster", "synthetic"):
            raise RuntimeError("BCOD_HOST_CLASS must be local, cluster, or synthetic")
        return configured
    return "cluster" if os.environ.get("SLURM_JOB_ID") else "local"


def assert_execution_authorized() -> None:
    protocol = json.loads(FEEDFORWARD_PROTOCOL_ARTIFACT.read_text())
    if protocol.get("task_contract_content_sha256") != FROZEN_CONTRACT_SHA256:
        raise RuntimeError("feedforward execution protocol has a stale contract hash")
    if not protocol.get("execution_authorized", False):
        raise RuntimeError(
            "Feedforward training is disabled; explicitly authorize it in "
            "portable-feedforward-training-protocol.json before launch."
        )


def assert_calm_disturbance(info: dict) -> None:
    """Refuse comparison training without exact applied-disturbance evidence."""
    required = ("applied_current_ned_mps", "applied_wind_ned_mps")
    missing = [field for field in required if field not in info]
    if missing:
        raise RuntimeError(
            "disturbance parity gate missing reset evidence: " + ", ".join(missing)
        )
    for field in required:
        vector = np.asarray(info[field], dtype=float)
        if (
            vector.shape != (3,)
            or not np.all(np.isfinite(vector))
            or not np.allclose(vector, 0.0, atol=0.0, rtol=0.0)
        ):
            raise RuntimeError(
                f"disturbance parity gate requires exact zero {field}, got {vector.tolist()}"
            )


def episode_metric_row(*, seed, env, info, total_return, wall_clock_s, policy_id, algorithm):
    reason = str(info["termination_reason"])
    collision = "grounding" if reason == "grounding" else "object" if reason in ("collision","object_collision") else "none"
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


def append_aligned_evaluation(path: Path, row: dict) -> None:
    """Append one fixed-timestep policy measurement with a stable schema."""
    if tuple(row) != ALIGNED_EVALUATION_COLUMNS:
        raise RuntimeError("aligned evaluation row does not match its declared schema")
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists()
    with path.open("a", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=ALIGNED_EVALUATION_COLUMNS)
        if not exists:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()


def aligned_evaluation_callback_class(base_callback):
    """Build a callback without importing Stable-Baselines3 at module import time."""
    class AlignedEvaluationCallback(base_callback):
        def __init__(self, *, run_dir, evaluation_env, first_seed, episodes,
                     frequency_timesteps):
            super().__init__(verbose=0)
            self.run_dir = Path(run_dir)
            self.evaluation_env = evaluation_env
            self.first_seed = int(first_seed)
            self.episodes = int(episodes)
            self.frequency_timesteps = int(frequency_timesteps)
            self.next_checkpoint = 0
            self.rows = []

        def _measure(self, checkpoint_timesteps):
            result = evaluate_feedforward(
                self.model, self.evaluation_env, self.first_seed, self.episodes)
            returns = [float(row["return"]) for row in result["rows"]]
            row = {
                "checkpoint_timesteps": int(checkpoint_timesteps),
                "actual_model_timesteps": int(self.model.num_timesteps),
                "episodes": self.episodes,
                "median_return": float(result["median_return"]),
                "mean_return": float(statistics.fmean(returns)),
                "success_rate": float(result["success_rate"]),
            }
            self.rows.append(row)
            append_aligned_evaluation(
                self.run_dir / "metrics" / "aligned-evaluation.csv", row)
            atomic_json(
                self.run_dir / "metrics" / "aligned-evaluation.json",
                {"schema_version": 1,
                 "measurement": "deterministic feedforward policy evaluation",
                 "first_seed": self.first_seed,
                 "episodes_per_checkpoint": self.episodes,
                 "frequency_timesteps": self.frequency_timesteps,
                 "rows": self.rows},
            )
            print(json.dumps({"aligned_evaluation": row}), flush=True)

        def _on_training_start(self):
            prior = self.run_dir / "metrics" / "aligned-evaluation.json"
            if self.model.num_timesteps:
                if prior.exists():
                    self.rows = json.loads(prior.read_text()).get("rows", [])
                else:
                    self._measure(self.model.num_timesteps)
                self.next_checkpoint = (
                    self.model.num_timesteps // self.frequency_timesteps + 1
                ) * self.frequency_timesteps
            else:
                self._measure(0)
                self.next_checkpoint = self.frequency_timesteps

        def _on_step(self):
            while self.num_timesteps >= self.next_checkpoint:
                self._measure(self.next_checkpoint)
                self.next_checkpoint += self.frequency_timesteps
            return True

    return AlignedEvaluationCallback


def bcod_factory(rank: int, curriculum: bool):
    return lambda: CommonWaypointEnv(
        ROOT, base_seed=200_000 + rank * 1_000_000,
        final_leg_curriculum=curriculum,
    )


def make_env(args):
    common = dict(base_seed=args.base_seed, fixed_reset_seed=args.fixed_reset_seed,
                  final_leg_curriculum=args.final_leg_curriculum,
                  condition_contract_path=getattr(args, "condition_contract_path", None))
    if args.backend == "bcod-sim":
        return CommonWaypointEnv(ROOT, disturbance_mode=args.disturbance_mode, **common)
    if args.backend == "holoocean":
        return HoloOceanVehicleAEnv(
            ROOT, base_seed=args.base_seed,
            fixed_reset_seed=args.fixed_reset_seed,
            disturbance_mode=args.disturbance_mode,
            wind_mode=args.holoocean_wind_mode,
            condition_contract_path=getattr(args, "condition_contract_path", None),
        )
    if args.backend == "stonefish":
        required = {
            "--stonefish-executable": args.stonefish_executable,
            "--stonefish-data-dir": args.stonefish_data_dir,
            "--stonefish-lib": args.stonefish_lib,
            "--stonefish-deps-lib": args.stonefish_deps_lib,
        }
        missing = [name for name, value in required.items() if value is None]
        if missing:
            raise SystemExit(f"Stonefish requires: {', '.join(missing)}")
        gate = json.loads((ROOT / "stonefish/gate_d_validation.json").read_text())
        if gate.get("status") != "PASS_READY_FOR_TRAINING_REVIEW":
            raise RuntimeError("Stonefish Gate D has not passed; PPO training is blocked")
        return StonefishGymEnv(
            ROOT, executable=args.stonefish_executable,
            data_dir=args.stonefish_data_dir,
            library_dirs=(args.stonefish_lib, args.stonefish_deps_lib),
            physics_threads=args.stonefish_physics_threads,
            sensor_noise=args.stonefish_sensor_noise,
            base_seed=args.base_seed, fixed_reset_seed=args.fixed_reset_seed,
            condition_contract_path=getattr(args, "condition_contract_path", None),
        )
    if args.backend == "gazebo-harmonic" and not args.runtime_command:
        common["disturbance_mode"] = "zero"
        runtime = [sys.executable, str(
            ROOT / "validation/rl-campaign/ports/gazebo_gym_runtime.py")]
        return GazeboGymEnv(
            ROOT, runtime, allow_unconformant_diagnostic=args.diagnostic_only, **common)
    if not args.runtime_command:
        raise SystemExit("--runtime-command is required for an external backend")
    runtime = shlex.split(args.runtime_command)
    cls = VrxGymEnv if args.backend == "vrx" else GazeboGymEnv
    if args.backend == "gazebo-harmonic":
        common["disturbance_mode"] = "zero"
    return cls(ROOT, runtime, allow_unconformant_diagnostic=args.diagnostic_only, **common)


class RewardComponentTrace(gym.Wrapper):
    """Stream every portable reward component without changing environment data."""

    def __init__(self, env: gym.Env, filename: Path) -> None:
        super().__init__(env)
        self.filename = Path(filename)
        self._stream = self.filename.open("w", newline="")
        self._writer = csv.DictWriter(self._stream, fieldnames=REWARD_COMPONENT_COLUMNS)
        self._writer.writeheader()
        self._episode_index = -1
        self._episode_seed = None
        self._control_step = 0

    def reset(self, **kwargs):
        observation, info = self.env.reset(**kwargs)
        self._episode_index += 1
        self._episode_seed = info.get("seed")
        self._control_step = 0
        return observation, info

    def step(self, action):
        observation, reward, terminated, truncated, info = self.env.step(action)
        components = info.get("reward_components")
        required = {
            "progress", "cross_track", "action_delta", "terminal",
            "base_reward", "potential_shaping", "shaped_reward",
        }
        if not isinstance(components, dict) or not required <= components.keys():
            raise RuntimeError("portable environment omitted required reward components")
        metric_fields = ("completion_fraction", "success", "waypoints_reached", "termination_reason")
        missing_metrics = [name for name in metric_fields if name not in info]
        if missing_metrics:
            raise RuntimeError(
                "portable environment omitted completion metrics: " + ", ".join(missing_metrics)
            )
        self._control_step += 1
        self._writer.writerow({
            "episode_index": self._episode_index,
            "episode_seed": self._episode_seed,
            "control_step": self._control_step,
            "physics_steps": info.get("physics_steps"),
            **{name: float(components[name]) for name in required},
            "completion_fraction": float(info["completion_fraction"]),
            "success": bool(info["success"]),
            "waypoints_reached": int(info["waypoints_reached"]),
            "termination_reason": info["termination_reason"],
            "terminated": bool(terminated),
            "truncated": bool(truncated),
        })
        self._stream.flush()
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.close()
        super().close()


def training_env_factory(args, rank: int, run_dir: Path):
    """Build one isolated monitored environment for a vector training run."""
    def factory():
        from stable_baselines3.common.monitor import Monitor
        ranked = argparse.Namespace(**vars(args))
        ranked.base_seed = args.base_seed + rank * 1_000_000
        traced = RewardComponentTrace(
            make_env(ranked),
            run_dir / "metrics" / f"reward-components-env-{rank}.csv",
        )
        return Monitor(traced, filename=str(
            run_dir / "metrics" / f"episodes-env-{rank}"))
    return factory


def default_output(backend: str) -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return ROOT / "artifacts/rl-campaign/training-runs" / f"{backend}-feedforward-{stamp}"


def git_revision() -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
    )
    return result.stdout.strip() or None if result.returncode == 0 else None


def write_episode_rows(path: Path, rows: list[dict]) -> None:
    atomic_json(path / "evaluation-episodes.json", {
        "schema_version": 1, "columns": list(EPISODE_COLUMNS), "rows": rows,
    })
    with (path / "evaluation-episodes.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EPISODE_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def evaluate_feedforward(model, env_factory, first_seed: int, episodes: int):
    """Evaluate a deterministic memoryless policy."""
    rows = []
    for seed in range(first_seed, first_seed + episodes):
        env = env_factory(seed)
        try:
            started = time.perf_counter()
            observation, _ = env.reset()
            total_return = 0.0
            while True:
                action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                total_return += float(reward)
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


def run_feedforward_protocol():
    """Feedforward Phase 1 gate followed by Phase 2 only on convergence."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv

    FEEDFORWARD_PROTOCOL_OUT.mkdir(parents=True, exist_ok=True)
    phase1_env = SubprocVecEnv([bcod_factory(i, True) for i in range(16)], start_method="fork")
    policy, kwargs = ppo_kwargs("bcod-sim")
    model = PPO(policy, phase1_env, **kwargs, verbose=1, device="cpu")
    curve, consecutive = [], 0
    atomic_json(FEEDFORWARD_PROTOCOL_OUT / "training-state.json", {
        "status": "phase-1-running", "timesteps": 0,
        "algorithm": "PPO", "policy": "MlpPolicy",
        "algorithm_provenance": algorithm_provenance("bcod-sim"),
        "convergence_rule": "success_rate >= 0.9 at two consecutive 250k checkpoints",
    })
    for steps in range(250_000, 1_500_001, 250_000):
        model.learn(total_timesteps=250_000, reset_num_timesteps=False, progress_bar=False)
        model.save(FEEDFORWARD_PROTOCOL_OUT / f"phase1-{steps}")
        result = evaluate_feedforward(
            model,
            lambda seed: CommonWaypointEnv(ROOT, fixed_reset_seed=seed,
                                            final_leg_curriculum=True),
            30000, 50,
        )
        curve.append({"checkpoint_steps": steps, "evaluation": result})
        consecutive = consecutive + 1 if result["success_rate"] >= .9 else 0
        atomic_json(FEEDFORWARD_PROTOCOL_OUT / "phase-1-report.json", {
            "schema_version": 1, "converged": consecutive >= 2, "curve": curve})
        atomic_json(FEEDFORWARD_PROTOCOL_OUT / "training-state.json", {
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
        atomic_json(FEEDFORWARD_PROTOCOL_OUT / "training-state.json", {
            "status": "phase-1-failed", "timesteps": curve[-1]["checkpoint_steps"],
            "success_rate": curve[-1]["evaluation"]["success_rate"],
            "full_episode_training_started": False,
            "required_next_step": "Diagnose the feedforward failure before any architecture or budget change.",
        })
        return 2

    phase2_env = SubprocVecEnv([bcod_factory(i, False) for i in range(16)], start_method="fork")
    model.set_env(phase2_env)
    model.learn(total_timesteps=3_000_000, reset_num_timesteps=True, progress_bar=False)
    model.save(FEEDFORWARD_PROTOCOL_OUT / "feedforward-ppo-final")
    phase2_env.close()
    evaluation = evaluate_feedforward(
        model, lambda seed: CommonWaypointEnv(ROOT, fixed_reset_seed=seed), 10000, 50)
    atomic_json(FEEDFORWARD_PROTOCOL_OUT / "report.json", {
        "schema_version": 1, "artifact_kind": "surveyor-portable-feedforward-ppo",
        "phase_1_steps": curve[-1]["checkpoint_steps"],
        "phase_1_curve": curve, "phase_2_steps": 3_000_000,
        "evaluation": evaluation,
    })
    atomic_json(FEEDFORWARD_PROTOCOL_OUT / "training-state.json", {
        "status": "completed", "phase_2_started": True,
        "report": str((FEEDFORWARD_PROTOCOL_OUT / "report.json").relative_to(ROOT)),
    })
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("bcod-sim", "vrx", "gazebo-harmonic", "holoocean", "stonefish"), required=True)
    parser.add_argument("--runtime-command", help="Persistent normalized JSONL runtime command")
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--fixed-reset-seed", type=int)
    parser.add_argument("--final-leg-curriculum", action="store_true")
    parser.add_argument("--diagnostic-only", action="store_true",
                        help="Smoke-test an unpassed external port; training remains prohibited")
    parser.add_argument("--smoke-steps", type=int, default=0)
    parser.add_argument("--timesteps", type=int, default=0)
    parser.add_argument("--output", type=Path,
                        help="Run directory (default: timestamped artifacts/rl-campaign/training-runs directory)")
    parser.add_argument("--device", default="auto", help="Torch device: auto, cpu, cuda, or cuda:N")
    parser.add_argument("--n-envs", type=int, default=1,
                        help="Parallel isolated simulator environments")
    parser.add_argument("--checkpoint-freq", type=int, default=250_000,
                        help="Checkpoint interval in training timesteps")
    parser.add_argument("--resume-from", type=Path,
                        help="Load a saved PPO .zip checkpoint and train additional timesteps")
    parser.add_argument("--holoocean-wind-mode", choices=("off", "surge_equivalent"), default="off")
    parser.add_argument("--disturbance-mode", choices=("zero", "seeded"), default="zero",
                        help="Applied disturbance condition; portable three-way training requires zero")
    parser.add_argument("--stonefish-executable", type=Path)
    parser.add_argument("--stonefish-data-dir", type=Path)
    parser.add_argument("--stonefish-lib", type=Path)
    parser.add_argument("--stonefish-deps-lib", type=Path)
    parser.add_argument("--stonefish-physics-threads", type=int, default=1)
    parser.add_argument("--stonefish-sensor-noise", action="store_true")
    parser.add_argument("--eval-first-seed", type=int, default=10000)
    parser.add_argument("--eval-episodes", type=int, default=0)
    parser.add_argument("--curve-eval-freq", type=int, default=16_384,
                        help="Global timestep spacing for aligned learning-curve evaluations")
    parser.add_argument("--curve-eval-episodes", type=int, default=10,
                        help="Deterministic episodes per aligned checkpoint; 0 disables")
    parser.add_argument("--run-feedforward-protocol", action="store_true")
    args = parser.parse_args()
    if args.timesteps < 0 or args.smoke_steps < 0 or args.eval_episodes < 0:
        parser.error("timestep and episode counts must be non-negative")
    if args.checkpoint_freq <= 0:
        parser.error("--checkpoint-freq must be positive")
    if args.resume_from is not None:
        args.resume_from = args.resume_from.expanduser().resolve()
        if not args.resume_from.is_file():
            parser.error(f"--resume-from checkpoint does not exist: {args.resume_from}")
        if not args.timesteps:
            parser.error("--resume-from requires positive --timesteps to add")
    if args.n_envs <= 0:
        parser.error("--n-envs must be positive")
    if args.curve_eval_freq <= 0:
        parser.error("--curve-eval-freq must be positive")
    if args.curve_eval_episodes < 0:
        parser.error("--curve-eval-episodes must be non-negative")
    if args.timesteps and args.curve_eval_episodes and args.curve_eval_freq % args.n_envs:
        parser.error("--curve-eval-freq must be divisible by --n-envs for exact alignment")
    if args.stonefish_physics_threads <= 0:
        parser.error("--stonefish-physics-threads must be positive")
    if args.timesteps and args.backend in {"bcod-sim", "holoocean", "stonefish"}:
        if args.disturbance_mode != "zero":
            parser.error("bcod-sim/HoloOcean/Stonefish comparison training requires --disturbance-mode zero")
        if args.holoocean_wind_mode != "off":
            parser.error("calm comparison training requires --holoocean-wind-mode off")
    if args.backend == "stonefish":
        required = {
            "--stonefish-executable": args.stonefish_executable,
            "--stonefish-data-dir": args.stonefish_data_dir,
            "--stonefish-lib": args.stonefish_lib,
            "--stonefish-deps-lib": args.stonefish_deps_lib,
        }
        missing = [name for name, value in required.items() if value is None]
        if missing:
            parser.error(f"Stonefish requires: {', '.join(missing)}")
    if args.run_feedforward_protocol:
        if args.backend != "bcod-sim":
            raise SystemExit("The feedforward protocol currently applies only to bcod-sim")
        if args.timesteps or args.diagnostic_only or args.smoke_steps:
            raise SystemExit("--run-feedforward-protocol cannot be combined with ad-hoc run options")
        assert_execution_authorized()
        raise SystemExit(run_feedforward_protocol())
    if args.diagnostic_only and args.timesteps:
        raise SystemExit("diagnostic-only runtimes cannot train a policy")
    if args.timesteps:
        assert_execution_authorized()
    run_dir = (args.output or default_output(args.backend)).resolve()
    if args.timesteps:
        run_dir.mkdir(parents=True, exist_ok=False)
        (run_dir / "checkpoints").mkdir()
        (run_dir / "metrics").mkdir()
        atomic_json(run_dir / "run-manifest.json", {
            "schema_version": 1, "status": "running",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "backend": args.backend, "requested_timesteps": args.timesteps,
            "timestep_request_semantics": (
                "additional" if args.resume_from is not None else "new-run-total"),
            "resumed_from": None if args.resume_from is None else str(args.resume_from),
            "seed": args.base_seed, "device": args.device,
            "parallel_environments": args.n_envs,
            "disturbance_mode": args.disturbance_mode,
            "action_fairness_condition": "topology-native normalized actuators",
            "checkpoint_frequency_timesteps": args.checkpoint_freq,
            "evaluation": {"episodes": args.eval_episodes,
                           "first_seed": args.eval_first_seed},
            "aligned_curve_evaluation": {
                "enabled": bool(args.curve_eval_episodes),
                "frequency_timesteps": args.curve_eval_freq,
                "episodes_per_checkpoint": args.curve_eval_episodes,
                "first_seed": args.eval_first_seed,
            },
            "algorithm_provenance": algorithm_provenance(args.backend),
            "git_revision": git_revision(),
            "command": [sys.executable, *sys.argv],
        })
    env = None
    try:
        env = make_env(args)
        observation, info = env.reset()
        if args.timesteps and args.backend in {"bcod-sim", "holoocean", "stonefish"}:
            assert_calm_disturbance(info)
        for _ in range(args.smoke_steps):
            observation, _, terminated, truncated, info = env.step(env.action_space.sample())
            if terminated or truncated:
                observation, info = env.reset()
        if args.timesteps:
            from stable_baselines3 import PPO
            from stable_baselines3.common.callbacks import (BaseCallback, CallbackList,
                                                             CheckpointCallback)
            from stable_baselines3.common.logger import configure
            from stable_baselines3.common.vec_env import SubprocVecEnv
            env.close()
            if args.n_envs == 1:
                env = training_env_factory(args, 0, run_dir)()
            else:
                env = SubprocVecEnv(
                    [training_env_factory(args, rank, run_dir)
                     for rank in range(args.n_envs)],
                    start_method="fork",
                )
            policy, kwargs = ppo_kwargs(args.backend)
            # A user-selected training seed is a run factor, not an algorithm
            # difference between simulators.
            kwargs["seed"] = args.base_seed
            if args.resume_from is None:
                model = PPO(policy, env, **kwargs, verbose=1,
                                     device=args.device)
                initial_timesteps = 0
            else:
                model = PPO.load(args.resume_from, env=env,
                                          device=args.device)
                initial_timesteps = int(model.num_timesteps)
            model.set_logger(configure(str(run_dir / "metrics"), ["stdout", "csv", "json"]))
            callback = CheckpointCallback(
                save_freq=max(1, args.checkpoint_freq // args.n_envs),
                save_path=str(run_dir / "checkpoints"),
                name_prefix=f"{args.backend}-feedforward-ppo",
            )
            callbacks = [callback]
            if args.curve_eval_episodes:
                def curve_evaluation_env(seed):
                    curve_args = argparse.Namespace(**vars(args))
                    curve_args.fixed_reset_seed = seed
                    return make_env(curve_args)
                callbacks.append(aligned_evaluation_callback_class(BaseCallback)(
                    run_dir=run_dir,
                    evaluation_env=curve_evaluation_env,
                    first_seed=args.eval_first_seed,
                    episodes=args.curve_eval_episodes,
                    frequency_timesteps=args.curve_eval_freq,
                ))
            started = time.time()
            model.learn(total_timesteps=args.timesteps,
                        reset_num_timesteps=args.resume_from is None,
                        callback=CallbackList(callbacks), progress_bar=False)
            model.logger.dump(model.num_timesteps)
            model.save(run_dir / "model-final")
            evaluation = None
            if args.eval_episodes:
                def evaluation_env(seed):
                    eval_args = argparse.Namespace(**vars(args))
                    eval_args.fixed_reset_seed = seed
                    return make_env(eval_args)
                evaluation = evaluate_feedforward(
                    model, evaluation_env, args.eval_first_seed, args.eval_episodes)
                write_episode_rows(run_dir, evaluation["rows"])
                atomic_json(run_dir / "evaluation-summary.json", {
                    key: value for key, value in evaluation.items() if key != "rows"
                })
            finished = datetime.now(timezone.utc).isoformat()
            manifest = json.loads((run_dir / "run-manifest.json").read_text())
            manifest.update({
                "status": "completed", "finished_at": finished,
                "actual_timesteps": int(model.num_timesteps),
                "initial_timesteps": initial_timesteps,
                "additional_timesteps_completed": int(model.num_timesteps) - initial_timesteps,
                "wall_clock_s": time.time() - started,
                "model": "model-final.zip",
                "training_metrics": {"csv": "metrics/progress.csv", "json": "metrics/progress.json",
                                     "episodes_csv_glob": "metrics/episodes-env-*.monitor.csv",
                                     "reward_components_csv_glob": "metrics/reward-components-env-*.csv",
                                     "reward_component_columns": list(REWARD_COMPONENT_COLUMNS),
                                     "aligned_evaluation_csv": (
                                         "metrics/aligned-evaluation.csv"
                                         if args.curve_eval_episodes else None),
                                     "aligned_evaluation_json": (
                                         "metrics/aligned-evaluation.json"
                                         if args.curve_eval_episodes else None)},
                "evaluation_summary": None if evaluation is None else {
                    key: value for key, value in evaluation.items() if key != "rows"
                },
            })
            atomic_json(run_dir / "run-manifest.json", manifest)
            print(json.dumps({"run_directory": str(run_dir), "status": "completed",
                              "actual_timesteps": int(model.num_timesteps)}, indent=2))
        print({"backend": args.backend, "reset": info, "observation_size": len(observation),
               "smoke_steps": args.smoke_steps, "trained_steps": args.timesteps})
    except Exception as error:
        manifest_path = run_dir / "run-manifest.json"
        if args.timesteps and manifest_path.exists():
            manifest = json.loads(manifest_path.read_text())
            manifest.update({
                "status": "failed",
                "finished_at": datetime.now(timezone.utc).isoformat(),
                "error": {"type": type(error).__name__, "message": str(error)},
            })
            atomic_json(manifest_path, manifest)
        raise
    finally:
        if env is not None:
            env.close()


if __name__ == "__main__":
    main()
