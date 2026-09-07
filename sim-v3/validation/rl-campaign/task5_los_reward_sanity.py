#!/usr/bin/env python3
"""Score frozen LOS-PID-v2 under the corrected Task 5 reward; no training."""
from __future__ import annotations

import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from bcod_sim.common_task import compute_reward  # noqa: E402
from run_task5_conditions import Task5ConditionEnv  # noqa: E402

OUT = ROOT / "artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k/los-pid-v2-corrected-reward-sanity.json"
MAX_THRUST_N = 70.0


def wrap(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def run_episode(seed):
    env = Task5ConditionEnv(ROOT, fixed_reset_seed=seed,
                            final_leg_curriculum=True, condition="default-noise")
    try:
        env.reset()
        leg_start, goal = env.route[1], env.route[2]
        dx, dy = goal[0] - leg_start[0], goal[1] - leg_start[1]
        denominator = dx * dx + dy * dy
        previous_distance = env._distance()
        previous_action = np.zeros(2)
        total = 0.0
        component_totals = {name: 0.0 for name in (
            "progress", "cross_track", "action_delta", "terminal",
            "base_reward", "potential_shaping")}
        trace = []
        success = False
        for step in range(env.max_control_steps):
            truth = env.last_truth
            position = truth["position_ned_m"]
            yaw = float(truth["attitude_rad"][2])
            surge_speed = float(truth["velocity_body_mps"][0])
            yaw_rate = float(truth["angular_rate_body_rad_s"][2])
            cross_signed = (-dy * (position[0] - leg_start[0])
                            + dx * (position[1] - leg_start[1])) / math.sqrt(denominator)
            desired_heading = math.atan2(dy, dx) - math.atan2(cross_signed, 4.0)
            heading_error = wrap(desired_heading - yaw)
            surge = max(-150.0, min(150.0, 100.0 * (1.5 - surge_speed)))
            yaw_wrench = max(-100.0, min(100.0, 70.0 * heading_error - 35.0 * yaw_rate))
            command = {"active_sensors": ["imu", "gps"],
                       "actuators": {"desiredWrench": [surge, 0, 0, 0, 0, yaw_wrench]}}
            result = None
            for _ in range(env.physics_steps_per_action):
                result = env.bridge.step([command])
            env.last_truth = env.bridge.ground_truth()
            distance = env._distance()
            track = env._cross_track()
            actuator = env.bridge.checkpoint()["checkpoints"][0]["payload"]["actuatorState"]
            allocated = actuator["lastEffectorCommands"]
            action = np.asarray([allocated["port"]["thrust"] / MAX_THRUST_N,
                                 allocated["starboard"]["thrust"] / MAX_THRUST_N])
            success = distance <= env.final_radius_m
            timed_out = step == env.max_control_steps - 1 or bool(result["truncated"][0])
            reason = "success" if success else "timeout" if timed_out else "running"
            scored = compute_reward(
                previous_distance, distance, track, action, previous_action, reason,
                previous_final_distance_m=previous_distance,
                final_distance_m=distance, shaping_k=env.shaping_k,
                shaping_gamma=env.shaping_gamma, shaping_enabled=True)
            total += scored.reward
            components = scored.components()
            for name in component_totals:
                component_totals[name] += components[name]
            trace.append({"step": step + 1, "time_s": (step + 1) * env.control_interval_s,
                          "distance_m": distance, "reward": scored.reward,
                          "termination_reason": reason,
                          "components": {name: components[name] for name in component_totals}})
            previous_distance = distance
            previous_action = action
            if success:
                break
        return {"seed": seed, "success": success,
                "termination_reason": "success" if success else "timeout",
                "shaped_return": total, "base_return": component_totals["base_reward"],
                "component_totals": component_totals,
                "final_distance_m": previous_distance, "trace": trace}
    finally:
        env.close()


def main():
    rows = [run_episode(seed) for seed in range(30000, 30050)]
    ppo = {}
    base = ROOT / "artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k"
    for condition in ("control", "noise-zero", "default-noise"):
        report = json.loads((base / condition / "report.json").read_text())
        ppo[condition] = {"success_rate": report["evaluation"]["success_rate"],
                          "median_shaped_return": report["evaluation"]["median_return"]}
    los_median = statistics.median(row["shaped_return"] for row in rows)
    component_medians = {name: statistics.median(
        row["component_totals"][name] for row in rows)
        for name in rows[0]["component_totals"]}
    report = {"schema_version": 1, "artifact_kind": "task-5-los-pid-v2-corrected-reward-sanity",
              "training_performed": False,
              "contract_sha256": Task5ConditionEnv.EXPECTED_CONTRACT_SHA256,
              "evaluation_seeds": [30000, 30049],
              "los_pid_v2": {"success_rate": sum(row["success"] for row in rows) / len(rows),
                             "median_shaped_return": los_median,
                             "median_base_return": statistics.median(row["base_return"] for row in rows),
                             "median_component_contributions": component_medians,
                             "rows": rows},
              "ppo_fresh_baseline": ppo,
              "gate_passed": all(los_median > value["median_shaped_return"] for value in ppo.values())}
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"los_pid_v2": {"success_rate": report["los_pid_v2"]["success_rate"],
                                      "median_shaped_return": los_median,
                                      "median_base_return": report["los_pid_v2"]["median_base_return"],
                                      "median_component_contributions": component_medians},
                      "ppo": ppo, "gate_passed": report["gate_passed"],
                      "output": str(OUT.relative_to(ROOT))}, indent=2))


if __name__ == "__main__":
    main()
