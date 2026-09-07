#!/usr/bin/env python3
"""Compare HoloOcean reset scenarios with bcod-sim's frozen seed mapping."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim.common_task_env import CommonWaypointEnv  # noqa: E402
from bcod_sim.holoocean_vehicle_a_env import HoloOceanVehicleAEnv  # noqa: E402


class Formula(CommonWaypointEnv):
    def __init__(self):
        super().__init__(ROOT, bridge=object())


def angle_error_degrees(value: float, reference: float) -> float:
    return math.degrees(math.atan2(math.sin(value - reference), math.cos(value - reference)))


def geometry(start, heading, route, current, wind):
    relative_route = [
        [float(point[0] - start[0]), float(point[1] - start[1])]
        for point in route
    ]
    first_bearing = math.atan2(relative_route[0][1], relative_route[0][0])
    return {
        "start_offset_m": [float(start[0]), float(start[1])],
        "route_relative_m": relative_route,
        "initial_distance_to_first_waypoint_m": float(math.hypot(*relative_route[0])),
        "first_leg_bearing_deg": math.degrees(first_bearing),
        "initial_heading_deg": math.degrees(heading),
        "initial_heading_error_deg": angle_error_degrees(heading, first_bearing),
        "current_ned_mps": [float(current[0]), float(current[1]), float(current[2])],
        "wind_ned_mps": [float(wind[0]), float(wind[1]), float(wind[2])],
    }


def error_summary(a, b):
    return {
        "route_relative_max_m": float(np.max(np.abs(
            np.asarray(a["route_relative_m"]) - np.asarray(b["route_relative_m"])
        ))),
        "initial_distance_m": abs(
            a["initial_distance_to_first_waypoint_m"]
            - b["initial_distance_to_first_waypoint_m"]
        ),
        "first_leg_bearing_deg": abs(a["first_leg_bearing_deg"] - b["first_leg_bearing_deg"]),
        "initial_heading_deg": abs(a["initial_heading_deg"] - b["initial_heading_deg"]),
        "initial_heading_error_deg": abs(
            a["initial_heading_error_deg"] - b["initial_heading_error_deg"]
        ),
        "current_vector_max_mps": float(np.max(np.abs(
            np.asarray(a["current_ned_mps"]) - np.asarray(b["current_ned_mps"])
        ))),
        "wind_vector_max_mps": float(np.max(np.abs(
            np.asarray(a["wind_ned_mps"]) - np.asarray(b["wind_ned_mps"])
        ))),
    }


def main() -> int:
    seeds = list(range(10000, 10005))
    tolerance = 1e-9
    bcod = Formula()
    holo = HoloOceanVehicleAEnv(ROOT)
    rows = []
    for seed in seeds:
        b_start, b_heading, b_route, b_current, b_wind = bcod._randomization(seed)
        randomization = holo._draw_randomization(seed)
        h_start = [randomization["spawn_nwu_m"][0], -randomization["spawn_nwu_m"][1]]
        h_route = [[point[0], -point[1]] for point in randomization["route_nwu_m"]]
        h_current = [
            randomization["current_nwu_mps"][0],
            -randomization["current_nwu_mps"][1],
            randomization["current_nwu_mps"][2],
        ]
        b_values = geometry(
            [b_start[0] - 10000.0, b_start[1] - 10000.0],
            b_heading, [[p[0] - 10000.0, p[1] - 10000.0] for p in b_route],
            b_current, b_wind,
        )
        h_values = geometry(
            h_start, randomization["heading_ned_rad"], h_route, h_current,
            randomization["wind_ned_mps"],
        )
        errors = error_summary(b_values, h_values)
        rows.append({
            "seed": seed,
            "bcod_sim": b_values,
            "holoocean": h_values,
            "absolute_errors": errors,
            "match": all(error <= tolerance for error in errors.values()),
        })
    report = {
        "schema_version": 1,
        "artifact_kind": "holoocean-bcod-reset-scenario-parity",
        "status": "PASS" if all(row["match"] for row in rows) else "FAIL",
        "training_run": False,
        "contract_path": "artifacts/rl-campaign/surveyor/task-contract-frozen.json",
        "required_mapping": "shared Mulberry32 draws in bcod-sim contract order",
        "holoocean_mapping": "numpy.default_rng draws in a different field order",
        "comparison_frame": "NED; route positions expressed relative to the sampled start so the backend's arbitrary world-origin translation is excluded",
        "absolute_tolerance": tolerance,
        "seeds": seeds,
        "rows": rows,
        "all_match": all(row["match"] for row in rows),
        "maximum_absolute_errors": {
            key: max(row["absolute_errors"][key] for row in rows)
            for key in rows[0]["absolute_errors"]
        },
    }
    output = ROOT / "artifacts/rl-campaign/holoocean-reset-scenario-parity.json"
    temporary = output.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n")
    temporary.replace(output)
    print(json.dumps(report, indent=2))
    return 0 if report["all_match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
