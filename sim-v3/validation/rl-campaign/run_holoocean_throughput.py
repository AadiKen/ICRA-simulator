"""Benchmark HoloOcean Vehicle A wrapper throughput without training."""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))
from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


def worker(index: int, steps: int, barrier, queue) -> None:
    started = time.perf_counter()
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=7319 + index)
    try:
        env.reset()
        # Benchmark sustained stepping, not episode reset or training overhead.
        env.timeout_steps = 2**31 - 1
        ready = time.perf_counter()
        barrier.wait()
        rollout_started = time.perf_counter()
        action = np.zeros(2, dtype=np.float32)
        unexpected_termination = None
        for step in range(steps):
            _, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                unexpected_termination = {
                    "step": step,
                    "reason": info.get("termination_reason"),
                }
                break
        finished = time.perf_counter()
        queue.put(
            {
                "index": index,
                "requested_steps": steps,
                "completed_steps": step + 1,
                "startup_s": ready - started,
                "rollout_start_s": rollout_started,
                "rollout_end_s": finished,
                "unexpected_termination": unexpected_termination,
            }
        )
    finally:
        env.close()


def benchmark(parallel_envs: int, aggregate_steps: int) -> dict:
    context = mp.get_context("spawn")
    base, remainder = divmod(aggregate_steps, parallel_envs)
    counts = [base + (index < remainder) for index in range(parallel_envs)]
    barrier = context.Barrier(parallel_envs)
    queue = context.Queue()
    processes = [
        context.Process(target=worker, args=(index, counts[index], barrier, queue))
        for index in range(parallel_envs)
    ]
    wall_started = time.perf_counter()
    for process in processes:
        process.start()
    rows = [queue.get() for _ in processes]
    for process in processes:
        process.join()
        if process.exitcode != 0:
            raise RuntimeError(f"benchmark worker exited with {process.exitcode}")
    wall_finished = time.perf_counter()
    completed = sum(row["completed_steps"] for row in rows)
    rollout_wall_s = max(row["rollout_end_s"] for row in rows) - min(
        row["rollout_start_s"] for row in rows
    )
    return {
        "parallel_environments": parallel_envs,
        "aggregate_control_steps": completed,
        "aggregate_physics_steps": completed * 2,
        "steady_state_wall_s": rollout_wall_s,
        "control_steps_per_s": completed / rollout_wall_s,
        "physics_steps_per_s": completed * 2 / rollout_wall_s,
        "end_to_end_wall_s_including_startup": wall_finished - wall_started,
        "startup_s_range": [
            min(row["startup_s"] for row in rows),
            max(row["startup_s"] for row in rows),
        ],
        "unexpected_terminations": [
            row for row in rows if row["unexpected_termination"] is not None
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parallel", type=int, nargs="+", default=[1, 2, 4, 8])
    parser.add_argument("--aggregate-steps", type=int, default=4096)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/rl-campaign/holoocean-vehicle-a-throughput.json",
    )
    args = parser.parse_args()
    gpu = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=name,uuid", "--format=csv,noheader"], text=True
    ).strip()
    results = [benchmark(count, args.aggregate_steps) for count in args.parallel]
    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-gate-d-throughput",
        "status": "COMPLETE_NO_TRAINING",
        "runtime": {
            "hostname": platform.node(),
            "gpu": gpu,
            "python": platform.python_version(),
            "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
            "holoocean": "2.3.0",
            "world": "Ocean/OpenWater",
            "viewport": False,
        },
        "method": {
            "aggregate_control_steps_per_configuration": args.aggregate_steps,
            "physics_steps_per_control_step": 2,
            "action": [0.0, 0.0],
            "timing": "synchronized steady-state wrapper step loop; engine startup reported separately",
            "training": False,
            "buoyancy_determinism_state": "unfixed packaged binary; random unauthored SurfacePoints remain",
        },
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(json.dumps(artifact, indent=2))


if __name__ == "__main__":
    main()
