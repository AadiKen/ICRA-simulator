"""Evaluation-only v6-method failure breakdown for the three Task 5 models."""
from __future__ import annotations

import json
import math
import statistics
from collections import Counter
from pathlib import Path
import sys

import numpy as np
from sb3_contrib import RecurrentPPO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "validation/rl-campaign"))
from p3_v6_failure_diagnostic import DT, SEEDS, classify, summary  # noqa: E402
from run_task5_conditions import CONDITIONS, Task5ConditionEnv  # noqa: E402

RUN = ROOT / "artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k"
OUT = RUN / "seed-7319-failure-diagnostic.json"


def wrap(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def replay(model, condition, seed):
    env = Task5ConditionEnv(
        ROOT, condition=condition, fixed_reset_seed=seed,
        final_leg_curriculum=True)
    observation, _ = env.reset()
    state = None
    episode_start = np.ones((1,), dtype=bool)
    distances, heading_errors, speeds = [], [], []
    success = False
    try:
        for _ in range(env.max_control_steps):
            action, state = model.predict(
                observation, state=state, episode_start=episode_start,
                deterministic=True)
            observation, _, terminated, truncated, info = env.step(action)
            episode_start = np.asarray([terminated or truncated], dtype=bool)
            truth = info["terminal_state"]
            north, east = truth["position_ned_m"][:2]
            goal = env.route[-1]
            error = wrap(math.atan2(goal[1] - east, goal[0] - north)
                         - truth["attitude_rad"][2])
            distances.append(float(info["distance_to_final_waypoint_m"]))
            heading_errors.append(abs(error))
            speeds.append(float(info["speed_mps"]))
            success = bool(info["success"])
            if terminated or truncated:
                break
    finally:
        env.close()
    closest_index = int(np.argmin(distances))
    classification, within6, reversals = classify(distances, success)
    return {
        "seed": seed,
        "success": success,
        "classification": classification,
        "closest_approach_m": distances[closest_index],
        "closest_approach_time_s": (closest_index + 1) * DT,
        "remaining_time_at_closest_s": 120 - (closest_index + 1) * DT,
        "speed_at_closest_m_s": speeds[closest_index],
        "absolute_heading_error_at_closest_deg": math.degrees(
            heading_errors[closest_index]),
        "median_absolute_heading_error_deg": math.degrees(
            statistics.median(heading_errors)),
        "time_within_6m_s": within6,
        "radial_direction_reversals": reversals,
    }


def main():
    conditions = {}
    for condition in CONDITIONS:
        checkpoint = RUN / condition / "recurrent-ppo-250k.zip"
        model = RecurrentPPO.load(checkpoint, device="cpu")
        rows = [replay(model, condition, seed) for seed in SEEDS]
        failures = [row for row in rows if not row["success"]]
        conditions[condition] = {
            "checkpoint": str(checkpoint.relative_to(ROOT)),
            "successes": len(rows) - len(failures),
            "failures": len(failures),
            "success_rate": (len(rows) - len(failures)) / len(rows),
            "failure_classification_counts": dict(Counter(
                row["classification"] for row in failures)),
            "failure_closest_approach_m": summary([
                row["closest_approach_m"] for row in failures]),
            "failure_heading_error_at_closest_deg": summary([
                row["absolute_heading_error_at_closest_deg"] for row in failures]),
            "failure_median_heading_error_deg": summary([
                row["median_absolute_heading_error_deg"] for row in failures]),
            "raw": rows,
        }
    report = {
        "schema_version": 1,
        "artifact_kind": "task-5-seed-7319-failure-mode-diagnostic",
        "status": "COMPLETE_EVALUATION_ONLY",
        "method": "Classification thresholds and DT match p3-v6-failure-diagnostic.json.",
        "training_performed": False,
        "training_seed": 7319,
        "evaluation_seeds": [30000, 30049],
        "conditions": conditions,
    }
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({name: {
        "success_rate": row["success_rate"],
        "failure_classification_counts": row["failure_classification_counts"],
        "failure_closest_approach_median": row["failure_closest_approach_m"]["median"],
        "failure_heading_at_closest_median_deg": row["failure_heading_error_at_closest_deg"]["median"],
    } for name, row in conditions.items()}, indent=2))


if __name__ == "__main__":
    main()
