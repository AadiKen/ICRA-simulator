"""Gate C dynamics, determinism, sensor seeding, and termination diagnostics."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Callable

from contract_mapping import FIELD_NAMES, StonefishContractMapper
from stonefish_bridge import StonefishBridge
from termination import StonefishTerminationMonitor


def bridge_factory(args: argparse.Namespace, *, threads: int | None = 1,
                   noise: bool = False) -> StonefishBridge:
    return StonefishBridge(
        args.executable, args.data_dir,
        library_dirs=(args.stonefish_lib, args.deps_lib),
        physics_threads=threads, sensor_noise=noise,
    )


def speed_curve(args: argparse.Namespace, command: float) -> dict:
    with bridge_factory(args) as bridge:
        initial = bridge.reset(9001)
        previous = initial["observation"]["gps"][2:4]
        speeds = []
        duration_s = 60 if command == 1.0 else 25
        for _ in range(duration_s):
            response = bridge.step(command, command, physics_steps=500)
            position = response["observation"]["gps"][2:4]
            speeds.append(math.hypot(position[0] - previous[0],
                                     position[1] - previous[1]))
            previous = position
    averaging_window_s = 10 if duration_s >= 60 else 5
    steady = sum(speeds[-averaging_window_s:]) / averaging_window_s
    tolerance = 0.02 * steady
    settling = None
    # Require at least a five-second observed dwell after entry; otherwise a
    # late horizon crossing is not evidence of settling.
    for index in range(max(0, len(speeds) - 4)):
        if all(abs(value - steady) <= tolerance for value in speeds[index:]):
            settling = float(index + 1)
            break
    return {
        "normalized_command_each": command,
        "thrust_each_n": response["diagnostics"]["port_thrust"],
        "one_second_interval_speeds_mps": speeds,
        "reported_cruise_speed_tail_mean_mps": steady,
        "tail_averaging_window_s": averaging_window_s,
        "two_percent_settling_time_s": settling,
    }


def turning_point(args: argparse.Namespace, demand: float) -> dict:
    port, starboard = 0.6 + demand, 0.6 - demand
    with bridge_factory(args) as bridge:
        bridge.reset(9001)
        bridge.step(0.6, 0.6, physics_steps=5000)
        previous = bridge.step(port, starboard, physics_steps=1)["observation"]["gps"][2:4]
        interval_speeds = []
        response = None
        for _ in range(10):
            response = bridge.step(port, starboard, physics_steps=500)
            position = response["observation"]["gps"][2:4]
            interval_speeds.append(math.hypot(position[0] - previous[0],
                                              position[1] - previous[1]))
            previous = position
    assert response is not None
    speed = sum(interval_speeds[-5:]) / 5.0
    yaw_rate = response["observation"]["imu"][5]
    return {
        "yaw_demand": demand,
        "action": [port, starboard, 0.0, 0.0],
        "achieved_speed_final_5s_mean_mps": speed,
        "achieved_yaw_rate_rad_s": yaw_rate,
        "turn_radius_m": speed / abs(yaw_rate),
    }


ACTIONS = (
    *((0.55, 0.55, 0.0, 0.0),) * 20,
    *((0.70, -0.30, 0.0, 0.0),) * 30,
    *((-0.30, 0.70, 0.0, 0.0),) * 30,
    *((0.0, 0.0, 0.0, 0.0),) * 20,
)


def deterministic_trace(args: argparse.Namespace, threads: int | None,
                        seed: int) -> list[list[float]]:
    with bridge_factory(args, threads=threads, noise=True) as bridge:
        mapper = StonefishContractMapper(bridge, goal_north_m=20, goal_east_m=5)
        trace = [list(mapper.reset(seed).observation)]
        trace.extend(list(mapper.step(action).observation) for action in ACTIONS)
    return trace


def characterize_repeats(args: argparse.Namespace, threads: int | None) -> dict:
    traces = [deterministic_trace(args, threads, 24680) for _ in range(8)]
    reference = traces[0]
    fields = []
    for field_index, field_name in enumerate(FIELD_NAMES):
        maximum = 0.0
        worst_steps = []
        for trace in traces[1:]:
            differences = [abs(sample[field_index] - reference[step][field_index])
                           for step, sample in enumerate(trace)]
            run_maximum = max(differences)
            maximum = max(maximum, run_maximum)
            worst_steps.append(differences.index(run_maximum))
        fields.append({
            "field": field_name,
            "maximum_absolute_divergence": maximum,
            "worst_step_by_repeat": worst_steps,
            "worst_step_consistent": len(set(worst_steps)) == 1,
        })
    worst = max(fields, key=lambda item: item["maximum_absolute_divergence"])
    return {
        "thread_setting": "default_physical_cores" if threads is None else threads,
        "repeats": len(traces),
        "samples_per_repeat": len(reference),
        "exactly_equal": all(item["maximum_absolute_divergence"] == 0.0
                             for item in fields),
        "worst_field": worst["field"],
        "worst_divergence": worst["maximum_absolute_divergence"],
        "per_field": fields,
    }


def seed_hook_check(args: argparse.Namespace) -> dict:
    with bridge_factory(args, noise=True) as bridge:
        a = bridge.reset(111)["observation"]
        b = bridge.reset(111)["observation"]
        c = bridge.reset(112)["observation"]
    fields: Callable[[dict], list[float]] = lambda x: [*x["gps"], *x["imu"], *x["compass"]]
    return {
        "noise_enabled": True,
        "same_seed_exactly_equal": fields(a) == fields(b),
        "different_seed_changes_draws": fields(a) != fields(c),
        "same_seed_sample": fields(a),
        "different_seed_sample": fields(c),
    }


def native_contact_case(args: argparse.Namespace, scenario: str) -> dict:
    with bridge_factory(args) as bridge:
        bridge.reset(700, scenario=scenario)
        response = bridge.step(0.0, 0.0, physics_steps=50)
    monitor = StonefishTerminationMonitor()
    reason = monitor.update(response, dt_s=0.1,
                            requested_action=(0.0, 0.0, 0.0, 0.0))
    return {"contacts": response["diagnostics"]["contacts"],
            "termination_reason": reason}


def synthetic_termination_checks() -> dict:
    base = {
        "observation": {"imu": [0.0] * 9},
        "diagnostics": {"port_thrust": 0.0, "starboard_thrust": 0.0,
                        "contacts": {"grounding": False,
                                     "object_collision": False}},
    }
    unstable = StonefishTerminationMonitor()
    tilted = json.loads(json.dumps(base))
    tilted["observation"]["imu"][0] = math.radians(61.0)
    reason = None
    for _ in range(10):
        reason = unstable.update(tilted, dt_s=0.1,
                                 requested_action=(0, 0, 0, 0))
    allocation = StonefishTerminationMonitor().update(
        base, dt_s=0.1, requested_action=(math.nan, 0, 0, 0))
    return {"tilt_61deg_for_1s": reason,
            "nonfinite_direct_mapping": allocation}


def main() -> None:
    parser = argparse.ArgumentParser()
    for name in ("executable", "data-dir", "stonefish-lib", "deps-lib", "output"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    args = parser.parse_args()

    dynamics = {
        "hull_changes": [],
        "decision": "No geometry/drag change: the existing hull exceeds the frozen route envelope at the 1 m/s working speed.",
        "bcod_theoretical_full_speed_mps": 5.45,
        "frozen_route_centerline_m": 80.387,
        "frozen_timeout_s": 120.0,
        "minimum_average_speed_mps": 80.387 / 120.0,
        "speed_force_curve": [speed_curve(args, x) for x in (0.25, 0.5, 0.75, 1.0)],
        "turning_radius_vs_yaw_demand": [turning_point(args, x) for x in (0.1, 0.2, 0.3, 0.4)],
    }
    determinism = {
        "preregistered_prediction_artifact": "gate_c_determinism_preregistration.json",
        "sensor_rng_hook": seed_hook_check(args),
        "default_threads": characterize_repeats(args, None),
        "single_thread": characterize_repeats(args, 1),
    }
    terminations = {
        "normal": native_contact_case(args, "normal"),
        "grounding": native_contact_case(args, "grounding"),
        "object_collision": native_contact_case(args, "object_collision"),
        "synthetic_threshold_checks": synthetic_termination_checks(),
        "allocation_gap": "Vehicle A has no allocator or native infeasibility signal; only non-finite direct command/output can map cleanly to allocation_failure. Rotor lag is not allocation failure.",
    }
    result = {"dynamics": dynamics, "determinism": determinism,
              "terminations": terminations}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({
        "speed_force_curve": [
            {key: value for key, value in item.items()
             if key != "one_second_interval_speeds_mps"}
            for item in dynamics["speed_force_curve"]
        ],
        "turning": dynamics["turning_radius_vs_yaw_demand"],
        "determinism": {
            "sensor_rng_hook": {
                key: value for key, value in determinism["sensor_rng_hook"].items()
                if not key.endswith("_sample")
            },
            "default_threads": {
                key: value for key, value in determinism["default_threads"].items()
                if key != "per_field"
            },
            "single_thread": {
                key: value for key, value in determinism["single_thread"].items()
                if key != "per_field"
            },
        },
        "terminations": terminations,
    }, indent=2))


if __name__ == "__main__":
    main()
