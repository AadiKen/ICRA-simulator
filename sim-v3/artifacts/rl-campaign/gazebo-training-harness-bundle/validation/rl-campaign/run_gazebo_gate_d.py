#!/usr/bin/env python3
"""Native CodeNimbus Gazebo Gate D throughput and 10k correctness gate."""
from __future__ import annotations

import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim import GazeboGymEnv  # noqa: E402

RUNTIME = [sys.executable, str(ROOT / "validation/rl-campaign/ports/gazebo_gym_runtime.py")]
PROTOCOL = ROOT / "artifacts/rl-campaign/gazebo-still-water-protocol.json"


def worker(rank: int, steps: int) -> dict:
    env = GazeboGymEnv(ROOT, RUNTIME, allow_unconformant_diagnostic=True,
                       base_seed=60000 + rank * 1000, disturbance_mode="zero")
    started = time.perf_counter()
    try:
        observation, _ = env.reset()
        reset_s = time.perf_counter() - started
        step_started = time.perf_counter()
        for _ in range(steps):
            observation, _, terminated, truncated, _ = env.step(np.zeros(2, np.float32))
            if terminated or truncated:
                observation, _ = env.reset()
        return {"rank": rank, "reset_wall_s": reset_s,
                "step_wall_s": time.perf_counter() - step_started,
                "finite": bool(np.isfinite(observation).all())}
    finally:
        env.close()


def atomic(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n")
    temporary.replace(path)


def main() -> int:
    if not os.environ.get("SLURM_JOB_ID") or os.environ.get("BCOD_GAZEBO_NATIVE") != "1":
        raise RuntimeError("Gazebo Gate D requires a native Slurm/Pyxis allocation")
    counts, steps, throughput = (1, 4, 8, 16), 32, []
    for count in counts:
        started = time.perf_counter()
        with concurrent.futures.ProcessPoolExecutor(max_workers=count) as pool:
            futures = {pool.submit(worker, rank, steps): rank for rank in range(count)}
            rows = []
            for future, rank in ((future, futures[future]) for future in futures):
                try:
                    rows.append(future.result())
                except Exception as error:
                    rows.append({"rank": rank, "finite": False,
                                 "error_type": type(error).__name__,
                                 "error": str(error)})
        elapsed = time.perf_counter() - started
        cell = {"parallel_environments": count, "steps_per_environment": steps,
                "aggregate_control_steps": count * steps, "wall_clock_s": elapsed,
                "aggregate_control_steps_per_s": count * steps / elapsed,
                "all_observations_finite": len(rows) == count and all(row["finite"] for row in rows),
                "workers": rows}
        throughput.append(cell)
        print(json.dumps(cell), flush=True)
    eligible = [row for row in throughput if row["all_observations_finite"]]
    selected = max(eligible, key=lambda row: row["aggregate_control_steps_per_s"])
    subprocess.run([sys.executable,
                    str(ROOT / "validation/rl-campaign/run_gazebo_correctness_dry_run.py"),
                    "--timesteps", "10000"], cwd=ROOT, check=True)
    dry_path = ROOT / "artifacts/rl-campaign/gazebo-correctness-dry-run/report.json"
    dry = json.loads(dry_path.read_text())
    passed = dry.get("status") == "PASS" and bool(eligible)
    protocol = json.loads(PROTOCOL.read_text())
    protocol["gate_d"] = {
        "measurement_host": os.uname().nodename,
        "slurm_job_id": os.environ["SLURM_JOB_ID"],
        "gazebo_sim_version": subprocess.run(
            ["gz", "sim", "--version"], text=True, capture_output=True, check=True).stdout.strip(),
        "environment_counts": list(counts), "steps_per_environment": steps,
        "throughput": throughput,
        "parallel_environments": selected["parallel_environments"],
        "selection_rule": "maximum measured aggregate control steps/s among finite cells",
        "budget_decision": "portable runner budget remains user-selected and equal across backends",
        "dry_run": str(dry_path.relative_to(ROOT)),
        "status": "PASS" if passed else "FAIL",
    }
    protocol["status"] = "COMPLETE_PASS" if passed else "GATE_D_FAIL"
    protocol["training_authorized"] = passed
    atomic(PROTOCOL, protocol)
    print(json.dumps({"status": protocol["status"],
                      "training_authorized": passed,
                      "selected_parallel_environments": selected["parallel_environments"]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
