#!/usr/bin/env python3
"""Benchmark full VRX episodes on native CodeNimbus hardware; never train."""
from __future__ import annotations

import argparse
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim import VrxGymEnv  # noqa: E402

RUNTIME = [sys.executable, str(ROOT / "validation/rl-campaign/ports/vrx_cluster_gym_runtime.py")]
OUT = ROOT / "artifacts/rl-campaign/vrx-gate-d-throughput.json"


def command(*args):
    try:
        return subprocess.check_output(args, text=True, stderr=subprocess.DEVNULL).strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


def worker(index, mode, barrier, queue):
    started = time.perf_counter()
    env = None
    try:
        env = VrxGymEnv(
            ROOT, RUNTIME, allow_unconformant_diagnostic=True,
            fixed_reset_seed=30000 + index, disturbance_mode=mode,
        )
        env.reset()
        ready = time.perf_counter()
        barrier.wait()
        rollout_started = time.perf_counter()
        action = np.zeros(2, dtype=np.float32)
        terminal = None
        for step in range(env.max_control_steps):
            _, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                terminal = info.get("termination_reason")
                break
        finished = time.perf_counter()
        queue.put({
            "index": index, "seed": 30000 + index,
            "completed_control_steps": step + 1,
            "completed_physics_steps": env.steps,
            "startup_s": ready - started,
            "rollout_start_s": rollout_started,
            "rollout_end_s": finished,
            "termination_reason": terminal,
            "error": None,
        })
    except Exception as error:
        # Always satisfy the parent queue so a failed startup is a recorded
        # capacity result, never an indefinitely hanging benchmark.
        try:
            barrier.abort()
        except Exception:
            pass
        queue.put({
            "index": index, "seed": 30000 + index,
            "completed_control_steps": 0, "completed_physics_steps": 0,
            "startup_s": time.perf_counter() - started,
            "rollout_start_s": None, "rollout_end_s": None,
            "termination_reason": None,
            "error": f"{type(error).__name__}: {error}",
        })
    finally:
        if env is not None:
            env.close()


def benchmark(count, mode):
    context = mp.get_context("spawn")
    barrier = context.Barrier(count)
    queue = context.Queue()
    processes = [context.Process(target=worker, args=(i, mode, barrier, queue)) for i in range(count)]
    wall_started = time.perf_counter()
    for process in processes:
        process.start()
    rows = [queue.get() for _ in processes]
    for process in processes:
        process.join()
        if process.exitcode != 0:
            raise RuntimeError(f"benchmark worker exited with {process.exitcode}")
    wall_finished = time.perf_counter()
    errors = [row for row in rows if row["error"] is not None]
    if errors:
        return {
            "disturbance_mode": mode, "parallel_environments": count,
            "status": "STARTUP_CAPACITY_FAILURE", "errors": errors,
            "full_episodes": 0, "aggregate_control_steps": 0,
            "aggregate_physics_steps": 0,
        }
    control_steps = sum(row["completed_control_steps"] for row in rows)
    physics_steps = sum(row["completed_physics_steps"] for row in rows)
    rollout_wall = max(row["rollout_end_s"] for row in rows) - min(row["rollout_start_s"] for row in rows)
    return {
        "disturbance_mode": mode,
        "parallel_environments": count,
        "status": "COMPLETE",
        "full_episodes": count,
        "aggregate_control_steps": control_steps,
        "aggregate_physics_steps": physics_steps,
        "steady_state_wall_s": rollout_wall,
        "control_steps_per_s": control_steps / rollout_wall,
        "physics_steps_per_s": physics_steps / rollout_wall,
        "end_to_end_wall_s_including_startup": wall_finished - wall_started,
        "startup_s_range": [min(row["startup_s"] for row in rows), max(row["startup_s"] for row in rows)],
        "termination_counts": {
            reason: sum((row["termination_reason"] or "none") == reason for row in rows)
            for reason in sorted(set(row["termination_reason"] or "none" for row in rows))
        },
        "workers": sorted(rows, key=lambda row: row["index"]),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--parallel", type=int, nargs="+", default=[1, 4, 8, 16])
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()
    if platform.machine() != "x86_64" or not os.environ.get("SLURM_JOB_ID"):
        raise RuntimeError("Gate D is valid only inside a native x86_64 Slurm allocation")
    results = []
    for count in args.parallel:
        results.append(benchmark(count, "zero"))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            "schema_version": 1, "artifact_kind": "vrx-gate-d-throughput-and-protocol",
            "status": "RUNNING", "completed_results": results,
            "gate_d_complete": False, "training_started": False,
        }, indent=2) + "\n")
        print(f"completed zero/{count}", flush=True)
        results.append(benchmark(count, "seeded"))
        args.output.write_text(json.dumps({
            "schema_version": 1, "artifact_kind": "vrx-gate-d-throughput-and-protocol",
            "status": "RUNNING", "completed_results": results,
            "gate_d_complete": False, "training_started": False,
        }, indent=2) + "\n")
        print(f"completed seeded/{count}", flush=True)
    complete = [row for row in results if row["status"] == "COMPLETE"]
    disturbed = [row for row in complete if row["disturbance_mode"] == "seeded"]
    if len(complete) != len(results):
        raise RuntimeError("one or more Gate D cells failed; inspect the incremental artifact")
    selected = max(disturbed, key=lambda row: row["control_steps_per_s"])
    comparisons = []
    for count in args.parallel:
        calm = next(row for row in results if row["parallel_environments"] == count and row["disturbance_mode"] == "zero")
        seeded = next(row for row in results if row["parallel_environments"] == count and row["disturbance_mode"] == "seeded")
        delta = (seeded["control_steps_per_s"] / calm["control_steps_per_s"] - 1) * 100
        comparisons.append({
            "parallel_environments": count,
            "disturbed_vs_calm_percent": delta,
            "meaningful_over_5_percent": abs(delta) > 5,
        })
    source_paths = [
        ROOT / "validation/rl-campaign/run_vrx_gate_d.py",
        ROOT / "validation/rl-campaign/ports/vrx_cluster_gym_runtime.py",
        ROOT / "validation/rl-campaign/ports/vrx_gym_runtime.py",
        ROOT / "validation/rl-campaign/ports/actuator-jsonl-bridge.ts",
    ]
    report = {
        "schema_version": 1,
        "artifact_kind": "vrx-gate-d-throughput-and-protocol",
        "status": "COMPLETE_NO_TRAINING",
        "runtime": {
            "hostname": platform.node(), "architecture": platform.machine(),
            "cpu": command("bash", "-lc", "lscpu | sed -n 's/^Model name:[[:space:]]*//p' | head -1"),
            "logical_cpus": os.cpu_count(),
            "allocated_cpus": int(os.environ.get("SLURM_CPUS_ON_NODE", os.cpu_count() or 0)),
            "gpu": command("bash", "-lc", "nvidia-smi --query-gpu=name --format=csv,noheader | paste -sd ',' -"),
            "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
            "slurm_partition": os.environ.get("SLURM_JOB_PARTITION"),
            "image": "leadcat/vrx:surveyor-patched-v3.0.1",
            "source_docker_image_id": "sha256:5fd48b867528a1db91eaf566ca7bfdb7447c98a4e64707706f77b61429a06970",
            "container_runtime": "Slurm Pyxis backed by Enroot",
            "gazebo_sim_version": command("gz", "sim", "--version"),
        },
        "method": {
            "one_full_120_second_episode_per_environment": True,
            "control_interval_s": .1, "physics_timestep_s": .05,
            "action": [0., 0.],
            "environment_counts": args.parallel,
            "conditions": ["zero", "seeded"],
            "timing": "synchronized steady-state full-episode step loop; startup also reported",
            "training": False,
        },
        "results": results,
        "disturbed_vs_calm": comparisons,
        "selection": {
            "parallel_environments": selected["parallel_environments"],
            "selection_rule": "Highest measured disturbed-condition aggregate control-step throughput on the allocated node.",
            "measured_disturbed_control_steps_per_s": selected["control_steps_per_s"],
        },
        "budget_decision": {
            "basis": "equal_timesteps",
            "decision": "Use the same frozen training-transition budget for bcod-sim, Gazebo, and VRX; report wall-clock cost separately.",
            "rationale": "The benchmark establishes a feasible native VRX rate. Equal timesteps preserves algorithmic sample-budget comparability; equal wall clock would expose each simulator to a different number of transitions and confound sample efficiency.",
        },
        "source_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in source_paths
        },
        "gate_d_complete": True,
        "dry_run_started": False,
        "training_started": False,
        "next_step": "Stop for review before Task 3 dry run.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
