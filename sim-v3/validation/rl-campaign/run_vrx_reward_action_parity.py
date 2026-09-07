#!/usr/bin/env python3
"""Task-1 VRX reward/action parity diagnostic; does not start Gate D or training."""
from __future__ import annotations

import hashlib
import inspect
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim import CommonWaypointEnv, VrxGymEnv  # noqa: E402
from bcod_sim import common_task, common_task_env  # noqa: E402

RUNTIME = [sys.executable, str(ROOT / "validation/rl-campaign/ports/vrx_gym_runtime.py")]
OUT = ROOT / "artifacts/rl-campaign/vrx-reward-action-parity.json"
SEED = 30000
COMPONENTS = (
    "progress", "cross_track", "action_delta", "terminal",
    "potential_shaping", "base_reward", "shaped_reward",
)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def circular_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def summarize(values: list[float]) -> dict[str, float]:
    return {
        "median_abs": statistics.median(values),
        "max_abs": max(values),
        "mean_abs": statistics.fmean(values),
    }


class StepRecorder:
    """Observe the exact bridge calls made by CommonWaypointEnv.step()."""

    def __init__(self, bridge):
        self.bridge = bridge
        self.rows: list[dict] = []
        self.original = bridge.step

    def install(self) -> None:
        def recorded(actions):
            result = self.original(actions)
            self.rows.append({
                "command": actions[0],
                "info": result["infos"][0],
                "truth": self.bridge.ground_truth(),
            })
            return result

        self.bridge.step = recorded


def reward_trace(disturbance_mode: str) -> dict:
    # This is the same mixed command trajectory used by the final Gate C trace.
    actions = [np.asarray([0.2 + 0.01 * i, 0.2 - 0.005 * i], np.float32) for i in range(12)]
    bcod = CommonWaypointEnv(ROOT, fixed_reset_seed=SEED, disturbance_mode=disturbance_mode)
    vrx = VrxGymEnv(
        ROOT, RUNTIME, allow_unconformant_diagnostic=True,
        fixed_reset_seed=SEED, disturbance_mode=disturbance_mode,
    )
    rows = []
    try:
        bcod.reset()
        vrx.reset()
        for index, action in enumerate(actions, 1):
            _, b_reward, b_term, b_trunc, b_info = bcod.step(action)
            _, v_reward, v_term, v_trunc, v_info = vrx.step(action)
            b_components = b_info["reward_components"]
            v_components = v_info["reward_components"]
            rows.append({
                "control_step": index,
                "action": action.tolist(),
                "bcod": {
                    "reward": b_reward,
                    "components": b_components,
                    "position_ned_m": bcod.last_truth["position_ned_m"],
                    "yaw_rad": bcod.last_truth["attitude_rad"][2],
                },
                "vrx": {
                    "reward": v_reward,
                    "components": v_components,
                    "position_ned_m": vrx.last_truth["position_ned_m"],
                    "yaw_rad": vrx.last_truth["attitude_rad"][2],
                },
                "absolute_component_divergence": {
                    name: abs(float(b_components[name]) - float(v_components[name]))
                    for name in COMPONENTS
                },
            })
            if b_term or b_trunc or v_term or v_trunc:
                break
    finally:
        bcod.close()
        vrx.close()
    divergence = {
        name: summarize([row["absolute_component_divergence"][name] for row in rows])
        for name in COMPONENTS
    }
    return {
        "disturbance_mode": disturbance_mode,
        "seed": SEED,
        "protocol": "identical 12-control-step mixed action trajectory at 10 Hz",
        "component_divergence": divergence,
        "per_step": rows,
    }


def known_action(action: tuple[float, float], label: str) -> dict:
    env = VrxGymEnv(
        ROOT, RUNTIME, allow_unconformant_diagnostic=True,
        fixed_reset_seed=SEED, disturbance_mode="zero",
    )
    recorder = StepRecorder(env.bridge)
    recorder.install()
    try:
        reset_obs, _ = env.reset()
        initial_truth = json.loads(json.dumps(env.last_truth))
        observations = []
        for _ in range(12):
            obs, _, terminated, truncated, _ = env.step(np.asarray(action, np.float32))
            observations.append(obs.tolist())
            if terminated or truncated:
                break
        final_truth = json.loads(json.dumps(env.last_truth))
        heading = initial_truth["attitude_rad"][2]
        dn = final_truth["position_ned_m"][0] - initial_truth["position_ned_m"][0]
        de = final_truth["position_ned_m"][1] - initial_truth["position_ned_m"][1]
        forward_displacement = dn * math.cos(heading) + de * math.sin(heading)
        truth_yaw_delta = circular_delta(final_truth["attitude_rad"][2], heading)
        fresh_yaws = [row[6] for row in observations if any(abs(x) > 0 for x in row[:7])]
        observation_yaw_delta = None if not fresh_yaws else circular_delta(fresh_yaws[-1], fresh_yaws[0])
        return {
            "label": label,
            "action": list(action),
            "control_steps": len(observations),
            "physics_steps": len(recorder.rows),
            "forward_displacement_m": forward_displacement,
            "final_body_surge_mps": final_truth["velocity_body_mps"][0],
            "truth_yaw_delta_rad": truth_yaw_delta,
            "observation_yaw_delta_rad": observation_yaw_delta,
            "passed": (
                forward_displacement > 0 and final_truth["velocity_body_mps"][0] > 0
                if label == "symmetric_positive"
                else truth_yaw_delta > 0 and observation_yaw_delta is not None and observation_yaw_delta > 0
            ),
            "physics_step_actuation": [row["info"] for row in recorder.rows],
            "reset_observation": reset_obs.tolist(),
        }
    finally:
        env.close()


def actuator_routing(forward: dict) -> dict:
    actual = [row["applied_thrust_newtons"] for row in forward["physics_step_actuation"]]
    expected = []
    state = [0.0, 0.0]
    alpha = 1 - math.exp(-0.05 / 0.35)
    for _ in actual:
        state = [value + (70.0 - value) * alpha for value in state]
        expected.append(list(state))
    errors = [abs(a - e) for pair_a, pair_e in zip(actual, expected) for a, e in zip(pair_a, pair_e)]
    sources = {
        "reward_function": Path(inspect.getsourcefile(common_task.compute_reward) or "").resolve(),
        "reward_call_site": Path(inspect.getsourcefile(common_task_env.CommonWaypointEnv.step) or "").resolve(),
        "vrx_runtime": ROOT / "validation/rl-campaign/ports/vrx_gym_runtime.py",
        "actuator_bridge": ROOT / "validation/rl-campaign/ports/actuator-jsonl-bridge.ts",
        "shared_actuator": ROOT / "validation/rl-campaign/ports/shared-actuators.ts",
    }
    runtime_source = sources["vrx_runtime"].read_text()
    reward_source = sources["reward_call_site"].read_text()
    return {
        "live_path": "CommonWaypointEnv.step -> VrxGymEnv bridge -> vrx_gym_runtime.py -> actuator-jsonl-bridge.ts -> FrozenActuatorBank(resolveSurveyorActuatorSpec()) -> VRX thrust topics",
        "reward_module": common_task.compute_reward.__module__,
        "reward_source_file": str(sources["reward_function"]),
        "single_reward_call_site_uses_shared_compute_reward": "compute_reward(" in reward_source,
        "reward_path_contains_no_actuator_bank": "FrozenActuatorBank" not in reward_source,
        "runtime_starts_one_actuator_bridge": runtime_source.count("actuator-jsonl-bridge.ts") == 1,
        "actuator_bridge_owns_frozen_bank": "FrozenActuatorBank(resolveSurveyorActuatorSpec())" in sources["actuator_bridge"].read_text(),
        "lag_model": {"target_n_each": 70.0, "time_constant_s": 0.35, "physics_dt_s": 0.05},
        "max_abs_lag_trace_error_n": max(errors),
        "lag_trace_matches_shared_bank": max(errors) <= 1e-9,
        "interpretation": "There is one process-local FrozenActuatorBank in the live VRX runtime. Reward evaluation consumes the resulting transition and does not instantiate or emulate a second actuator model.",
        "source_sha256": {str(path.relative_to(ROOT)): sha(path) for path in sources.values()},
    }


def main() -> int:
    calm = reward_trace("zero")
    disturbed = reward_trace("seeded")
    forward = known_action((1.0, 1.0), "symmetric_positive")
    yaw = known_action((1.0, -1.0), "positive_differential")
    routing = actuator_routing(forward)
    exact_components = ("action_delta", "terminal")
    shared_exact = all(
        trace["component_divergence"][name]["max_abs"] <= 1e-12
        for trace in (calm, disturbed) for name in exact_components
    )
    checks = {
        "shared_reward_module_no_vrx_reimplementation": (
            routing["single_reward_call_site_uses_shared_compute_reward"]
            and routing["reward_path_contains_no_actuator_bank"]
        ),
        "identical_action_dependent_reward_components": shared_exact,
        "symmetric_positive_moves_forward": forward["passed"],
        "positive_differential_matches_positive_observation_yaw": yaw["passed"],
        "reward_path_uses_single_shared_frozen_actuator_bank": (
            routing["runtime_starts_one_actuator_bridge"]
            and routing["actuator_bridge_owns_frozen_bank"]
            and routing["lag_trace_matches_shared_bank"]
        ),
    }
    report = {
        "schema_version": 1,
        "artifact_kind": "vrx-reward-action-parity",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "task": 1,
        "seed": SEED,
        "reward_parity": {"calm": calm, "disturbed": disturbed},
        "known_answer_actions": {"symmetric_positive": forward, "positive_differential": yaw},
        "actuator_routing": routing,
        "checks": checks,
        "all_checks_pass": all(checks.values()),
        "gate_d_started": False,
        "training_started": False,
        "next_step": "Stop for review before Task 2, as required by INSTRUCTIONS_vrx_harness_finish.md.",
    }
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["all_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
