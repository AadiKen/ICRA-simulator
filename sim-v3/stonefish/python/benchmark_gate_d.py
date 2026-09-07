"""Process-isolated Stonefish Gate D throughput benchmark."""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import queue
import subprocess
import time

from stonefish_bridge import StonefishBridge


def worker(index: int, count: int, args: dict, barrier, results) -> None:
    started = time.perf_counter()
    with StonefishBridge(
        args["executable"], args["data_dir"],
        library_dirs=(args["stonefish_lib"], args["deps_lib"]),
        physics_threads=1,
    ) as bridge:
        bridge.reset(8100 + index)
        ready = time.perf_counter()
        barrier.wait()
        rollout_started = time.perf_counter()
        for step in range(count):
            bridge.step(0.35, 0.35, physics_steps=50)
        rollout_finished = time.perf_counter()
    results.put({
        "index": index, "completed_control_steps": count,
        "startup_s": ready - started,
        "rollout_start_s": rollout_started,
        "rollout_end_s": rollout_finished,
    })


def benchmark(parallel: int, aggregate_steps: int, args: dict) -> dict:
    context = mp.get_context("spawn")
    base, remainder = divmod(aggregate_steps, parallel)
    counts = [base + int(i < remainder) for i in range(parallel)]
    barrier = context.Barrier(parallel)
    results = context.Queue()
    processes = [context.Process(target=worker, args=(i, counts[i], args, barrier, results))
                 for i in range(parallel)]
    wall_start = time.perf_counter()
    for process in processes:
        process.start()
    rows = []
    for _ in processes:
        try:
            rows.append(results.get(timeout=600))
        except queue.Empty as error:
            raise RuntimeError("throughput worker timed out") from error
    for process in processes:
        process.join()
        if process.exitcode != 0:
            raise RuntimeError(f"throughput worker exited {process.exitcode}")
    wall_end = time.perf_counter()
    completed = sum(row["completed_control_steps"] for row in rows)
    rollout_wall = max(row["rollout_end_s"] for row in rows) - min(row["rollout_start_s"] for row in rows)
    return {
        "parallel_instances": parallel,
        "aggregate_control_steps": completed,
        "aggregate_internal_physics_steps": completed * 50,
        "steady_state_wall_s": rollout_wall,
        "control_steps_per_s": completed / rollout_wall,
        "internal_physics_steps_per_s": completed * 50 / rollout_wall,
        "end_to_end_wall_s_including_startup": wall_end - wall_start,
        "startup_s_range": [min(row["startup_s"] for row in rows),
                            max(row["startup_s"] for row in rows)],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    for name in ("executable", "data-dir", "stonefish-lib", "deps-lib", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--parallel", type=int, nargs="+", default=[1, 2, 4, 8, 16])
    parser.add_argument("--aggregate-steps", type=int, default=4096)
    ns = parser.parse_args()
    values = {key: str(value) for key, value in vars(ns).items()
              if key not in ("parallel", "aggregate_steps")}
    rows = [benchmark(count, ns.aggregate_steps, values) for count in ns.parallel]
    peak = max(rows, key=lambda row: row["control_steps_per_s"])
    try:
        gpu = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
            text=True, stderr=subprocess.STDOUT,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        gpu = f"unavailable during CPU benchmark: {error}"
    artifact = {
        "schema_version": 1,
        "artifact_kind": "stonefish-gate-d-throughput",
        "status": "COMPLETE_NO_TRAINING",
        "runtime": {
            "hostname": platform.node(), "cpu_visible": os.cpu_count(),
            "gpu": gpu,
            "bridge": "Stonefish 1.6.0 console, one process per instance",
        },
        "method": {
            "aggregate_control_steps_per_configuration": ns.aggregate_steps,
            "internal_steps_per_control_step": 50,
            "action": [0.35, 0.35], "training": False,
            "instance_physics_threads": 1,
            "budget_note": "Equal timesteps remain fixed; throughput changes wall-clock cost only.",
        },
        "results": rows,
        "peak": peak,
        "regressions_after_peak": [row for row in rows
                                   if row["parallel_instances"] > peak["parallel_instances"]
                                   and row["control_steps_per_s"] < peak["control_steps_per_s"]],
    }
    ns.output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(json.dumps({"results": rows, "peak": peak,
                      "regressions_after_peak": artifact["regressions_after_peak"]}, indent=2))


if __name__ == "__main__":
    main()
