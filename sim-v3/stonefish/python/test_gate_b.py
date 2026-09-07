"""Run Gate B sign checks and the 15-field diagnostic trace."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from contract_mapping import CONTRACT_SHA256, FIELD_NAMES, StonefishContractMapper
from stonefish_bridge import StonefishBridge


def maneuver(bridge: StonefishBridge, port: float, starboard: float) -> dict:
    initial = bridge.reset(4242)
    final = bridge.step(port, starboard, physics_steps=500)
    obs = final["observation"]
    return {
        "action": [port, starboard],
        "duration_s": final["simulation_time"],
        "yaw_rate_rad_s": obs["imu"][5],
        "yaw_rad_compass": obs["compass"][0],
        "north_displacement_m": obs["gps"][2] - initial["observation"]["gps"][2],
        "east_displacement_m": obs["gps"][3] - initial["observation"]["gps"][3],
        "port_thrust_n": final["diagnostics"]["port_thrust"],
        "starboard_thrust_n": final["diagnostics"]["starboard_thrust"],
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--stonefish-lib", type=Path, required=True)
    parser.add_argument("--deps-lib", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    contract_document = json.loads(args.contract.read_text())
    contract_digest = contract_document.get("content_sha256")
    if contract_digest != CONTRACT_SHA256:
        raise RuntimeError(f"frozen contract hash mismatch: {contract_digest}")

    with StonefishBridge(
        args.executable,
        args.data_dir,
        library_dirs=(args.stonefish_lib, args.deps_lib),
    ) as bridge:
        signs = {
            "port_forward_starboard_reverse": maneuver(bridge, 1.0, -1.0),
            "port_reverse_starboard_forward": maneuver(bridge, -1.0, 1.0),
            "equal_forward": maneuver(bridge, 1.0, 1.0),
        }
        positive = signs["port_forward_starboard_reverse"]["yaw_rate_rad_s"]
        negative = signs["port_reverse_starboard_forward"]["yaw_rate_rad_s"]
        if not positive > 0.0 or not negative < 0.0:
            raise AssertionError("differential thrust yaw signs violate NED convention")

        mapper = StonefishContractMapper(
            bridge, goal_north_m=20.0, goal_east_m=5.0
        )
        samples = [("reset", mapper.reset(731).observation)]
        sequence = (
            ("forward", (0.55, 0.55, 0.0, 0.0), 10),
            ("right_turn", (0.70, -0.35, 0.0, 0.0), 10),
            ("left_turn", (-0.35, 0.70, 0.0, 0.0), 10),
            ("coast", (0.0, 0.0, 0.0, 0.0), 5),
        )
        for label, action, count in sequence:
            for index in range(count):
                sample = mapper.step(action)
                samples.append((f"{label}_{index + 1}", sample.observation))

    trace = [
        {
            "sample": index,
            "phase": label,
            "time_s": index * StonefishContractMapper.CONTROL_INTERVAL_S,
            "observation": list(values),
        }
        for index, (label, values) in enumerate(samples)
    ]
    flat = [value for item in trace for value in item["observation"]]
    yaw = [item["observation"][6] for item in trace]
    fix = [item["observation"][9] for item in trace]
    result = {
        "contract_sha256": contract_digest,
        "field_names": list(FIELD_NAMES),
        "frame_transform": {
            "world": "Stonefish NED -> contract NED: identity diag(1,1,1)",
            "body": "Stonefish FRD -> contract FRD: identity diag(1,1,1)",
            "yaw_source": "Compass[0], unchanged; positive clockwise about Down",
        },
        "sign_checks": signs,
        "sign_magnitude_relative_difference": abs(abs(positive) - abs(negative))
        / max(abs(positive), abs(negative)),
        "trace_summary": {
            "sample_count": len(trace),
            "shape_each": 15,
            "all_finite": all(math.isfinite(value) for value in flat),
            "nan_count": sum(math.isnan(value) for value in flat),
            "gps_valid_count": sum(value == 1.0 for value in fix),
            "gps_invalid_count": sum(value != 1.0 for value in fix),
            "yaw_min_rad": min(yaw),
            "yaw_max_rad": max(yaw),
            "initial_time_remaining": trace[0]["observation"][14],
            "final_time_remaining": trace[-1]["observation"][14],
        },
        "trace": trace,
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: result[key] for key in ("sign_checks", "trace_summary")}, indent=2))


if __name__ == "__main__":
    main()
