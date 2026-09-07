"""Measure HoloOcean Vehicle A turning response at calibrated cruise speed."""
from __future__ import annotations

import json
import math
from pathlib import Path
import statistics

import numpy as np

from run_holoocean_reward_validation import (
    GAINS,
    SEEDS,
    THRUST_CEILING_N,
    calm_environment,
    clamp,
    pi_surge_force,
    scheduled_speed_mps,
)


ROOT = Path(__file__).resolve().parents[2]
TURN_DEMANDS = (25.0, 50.0, 100.0, 250.0, 500.0)


def probe(turn_demand: float) -> dict:
    env = calm_environment(SEEDS[0])
    controller_state = {"surge_integral_error": 0.0}
    observation, _ = env.reset()
    try:
        for _ in range(round(30.0 / env.control_interval_s)):
            heading = float(observation[6])
            forward = np.asarray([math.cos(heading), math.sin(heading)])
            surge_speed = float(np.dot(env._ground_velocity_ned_mps, forward))
            surge = pi_surge_force(surge_speed, controller_state, env.control_interval_s)
            observation, _, _, _, _ = env.step(
                np.asarray([surge, surge], dtype=np.float32) / THRUST_CEILING_N
            )

        rows = []
        for _ in range(round(30.0 / env.control_interval_s)):
            heading = float(observation[6])
            forward = np.asarray([math.cos(heading), math.sin(heading)])
            surge_speed = float(np.dot(env._ground_velocity_ned_mps, forward))
            target_speed = scheduled_speed_mps(turn_demand)
            surge = pi_surge_force(
                surge_speed,
                controller_state,
                env.control_interval_s,
                target_speed,
            )
            action = np.asarray(
                [
                    clamp((surge + turn_demand) / THRUST_CEILING_N, -1.0, 1.0),
                    clamp((surge - turn_demand) / THRUST_CEILING_N, -1.0, 1.0),
                ],
                dtype=np.float32,
            )
            observation, _, _, _, info = env.step(action)
            rows.append(
                {
                    "surge_speed_mps": surge_speed,
                    "ground_speed_mps": float(np.linalg.norm(env._ground_velocity_ned_mps)),
                    "yaw_rate_rad_s": float(observation[5]),
                    "requested_surge_force_n": surge,
                    "action": action.tolist(),
                    "applied_force_n": info["applied_thruster_force_n"],
                }
            )
    finally:
        env.close()
    steady = rows[-100:]
    rate = statistics.fmean(abs(row["yaw_rate_rad_s"]) for row in steady)
    speed = statistics.fmean(row["ground_speed_mps"] for row in steady)
    port = statistics.fmean(row["applied_force_n"][0] for row in steady)
    starboard = statistics.fmean(row["applied_force_n"][1] for row in steady)
    return {
        "turn_demand": turn_demand,
        "scheduled_speed_mps": scheduled_speed_mps(turn_demand),
        "meaning_in_holoocean_adapter": "added to port force and subtracted from starboard force before per-thruster clipping",
        "steady_ground_speed_mps": speed,
        "steady_surge_speed_mps": statistics.fmean(row["surge_speed_mps"] for row in steady),
        "steady_abs_yaw_rate_rad_s": rate,
        "steady_turn_radius_m": None if rate == 0.0 else speed / rate,
        "steady_requested_surge_force_n": statistics.fmean(row["requested_surge_force_n"] for row in steady),
        "steady_applied_port_force_n": port,
        "steady_applied_starboard_force_n": starboard,
        "effective_yaw_moment_nm": port - starboard,
        "maximum_abs_yaw_rate_rad_s": max(abs(row["yaw_rate_rad_s"]) for row in rows),
    }


def bcod_expected(turn_moment_nm: float) -> dict:
    # Vehicle A source: Iz=30 kg m^2; yaw damping = 2r + |r|r.
    steady_rate = -1.0 + math.sqrt(1.0 + abs(turn_moment_nm))
    return {
        "commanded_yaw_moment_nm": turn_moment_nm,
        "steady_abs_yaw_rate_rad_s": steady_rate,
        "turn_radius_at_1_mps_m": 1.0 / steady_rate,
        "initial_yaw_acceleration_rad_s2": turn_moment_nm / 30.0,
    }


def main() -> None:
    holo = [probe(value) for value in TURN_DEMANDS]
    bcod = [bcod_expected(value) for value in TURN_DEMANDS]
    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-turning-diagnostic",
        "status": "diagnostic-only-no-gain-changes",
        "cruise_target_mps": GAINS["speed"],
        "holoocean": holo,
        "bcod_sim": {
            "source": "packages/vehicle-sdk/src/index.ts and packages/python-client/bcod_sim/linear_mpc.py",
            "yaw_inertia_kg_m2": 30.0,
            "yaw_damping": "2*r + abs(r)*r Nm",
            "thruster_lateral_offset_m": 0.395,
            "ideal_requested_moment_response": bcod,
        },
        "guidance": {
            "lookahead_m": 8.0,
            "desired_heading": "leg_heading - atan2(cross_track, lookahead)",
            "heading_kp_nm_per_rad": 100.0,
            "yaw_rate_kd_nm_per_rad_s": 35.0,
            "explicit_cross_track_gain": None,
        },
    }
    output = ROOT / "artifacts/rl-campaign/holoocean-vehicle-a-turning-diagnostic.json"
    output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(json.dumps(artifact, indent=2))


if __name__ == "__main__":
    main()
