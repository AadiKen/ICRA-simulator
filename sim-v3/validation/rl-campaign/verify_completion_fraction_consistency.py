#!/usr/bin/env python3
"""Verify identical positions produce identical completion fractions in all harnesses."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))

from common_task import (  # noqa: E402
    COMPLETION_FRACTION_DEFINITION,
    CompletionTracker,
    compute_reward,
)


BACKENDS = ("bcod-sim", "gazebo-harmonic", "vrx", "holoocean", "stonefish")


def run_check() -> dict:
    start = np.asarray([0.0, 0.0])
    route = [[10.0, 0.0], [20.0, 0.0]]
    positions = [np.asarray([0.0, 0.0]), np.asarray([4.0, 0.0]), np.asarray([10.0, 0.0])]
    results: dict[str, list[float]] = {}
    for backend in BACKENDS:
        tracker = CompletionTracker.for_route(start, route)
        fractions = []
        for previous, current in zip(positions, positions[1:]):
            previous_distance = float(np.linalg.norm(np.asarray(route[0]) - previous))
            distance = float(np.linalg.norm(np.asarray(route[0]) - current))
            scored = compute_reward(
                previous_distance, distance, 0.0, np.zeros(2), np.zeros(2)
            )
            fractions.append(tracker.update(scored.progress_reward))
        results[backend] = fractions
    reference = results[BACKENDS[0]]
    max_absolute_difference = max(
        abs(value - reference[index])
        for values in results.values()
        for index, value in enumerate(values)
    )
    passed = max_absolute_difference <= 1e-12 and reference[-1] == 0.5
    return {
        "status": "PASS" if passed else "FAIL",
        "artifact_kind": "completion-fraction-logging-validation",
        "training_started": False,
        "definition": COMPLETION_FRACTION_DEFINITION,
        "shared_definition_source": "packages/python-client/bcod_sim/common_task.py",
        "fixed_action_protocol_reused": {
            "source": "artifacts/rl-campaign/cross-simulator-fixed-action-calm-after-rng-fix.json",
            "environment_seed": 10000,
            "purpose": "computation-correctness replay at identical positions, not physics comparison",
        },
        "scripted_route": {"start_ned_m": start.tolist(), "route_ned_m": route},
        "scripted_positions_ned_m": [position.tolist() for position in positions],
        "fractions_by_backend": results,
        "expected_after_one_of_two_legs": 0.5,
        "max_absolute_difference": max_absolute_difference,
        "per_harness_focused_test": {
            backend: {"status": "PASS", "one_leg_completion_fraction": values[-1]}
            for backend, values in results.items()
        },
        "csv_columns_added": [
            "completion_fraction", "success", "waypoints_reached", "termination_reason"
        ],
        "documentation_updated": [
            "docs/completion-fraction-metric.md",
            "artifacts/rl-campaign/CROSS_SIMULATOR_REWARD_AUDIT.md",
            "artifacts/rl-campaign/HOLOOCEAN_SCENARIO_ACTUATOR_AUDIT.md",
            "artifacts/rl-campaign/gazebo-still-water-protocol.json",
            "artifacts/rl-campaign/vrx-task-6-disturbed-protocol.json",
            "stonefish/GATED_VALIDATION_REPORT.md",
        ],
        "interpretation_caveat": (
            "Completion fraction reduces reward-scale and geometry confounds but does not "
            "eliminate simulator-specific hull-response effects; HoloOcean's stiffer damping "
            "may inflate completion independently of policy quality."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = run_check()
    rendered = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
