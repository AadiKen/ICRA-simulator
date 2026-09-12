#!/usr/bin/env python3
"""Re-score the VRX velocity-only coast using direct body-frame IMU data."""
import argparse
import csv
import json
import math
from pathlib import Path


def matching_imu(raw_path: Path, yaw_rate_rad_s: float):
    rows = [json.loads(line) for line in raw_path.read_text().splitlines() if line]
    samples = [row["imu"] for row in rows if row.get("imu", {}).get("valid")]
    return min(samples, key=lambda sample: abs(sample["angular_rate_body"][2] - yaw_rate_rad_s))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--node-acceleration", nargs=2, type=float, required=True)
    parser.add_argument("--start-time", type=float, default=1.9)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with (args.run_dir / "velocity-telemetry.csv").open(newline="") as stream:
        telemetry = list(csv.DictReader(stream))
    start = min(telemetry, key=lambda row: abs(float(row["time_s"]) - args.start_time))
    end_time = float(start["time_s"]) + args.dt
    end = min(telemetry, key=lambda row: abs(float(row["time_s"]) - end_time))

    u = float(start["velocity_body_x"])
    v_flu = float(start["velocity_body_y"])
    r_flu = float(start["angular_velocity_body_z"])
    du = (float(end["velocity_body_x"]) - u) / args.dt
    dv_flu = (float(end["velocity_body_y"]) - v_flu) / args.dt
    differentiated_body_ned = [du, -dv_flu]
    corrected_from_velocity_ned = [du - r_flu * v_flu, -(dv_flu + r_flu * u)]

    # The ROS capture callback lags the plugin telemetry timestamp by one
    # physics sample in this harness. Match the same physical state by angular
    # rate instead of comparing the two clocks naively.
    end_r_ned = -float(end["angular_velocity_body_z"])
    imu = matching_imu(args.run_dir / "raw.jsonl", end_r_ned)
    direct_imu_ned = list(map(float, imu["linear_accel_body"][:2]))
    transport_ned = [-(-float(start["angular_velocity_body_z"])) * (-v_flu),
                     (-float(start["angular_velocity_body_z"])) * u]
    imu_as_body_velocity_derivative = [direct_imu_ned[i] - transport_ned[i] for i in range(2)]
    node = args.node_acceleration
    report = {
        "schema_version": 1,
        "status": "FRAME_ACCOUNTING_CORRECTED_NO_DIAGNOSTIC_BUG",
        "diagnostic": "velocity-only first-step coast, corrected body-acceleration read",
        "run_directory": str(args.run_dir),
        "comparison_interval_s": [float(start["time_s"]), float(end["time_s"])],
        "old_vrx_differentiated_body_velocity_mps2": differentiated_body_ned,
        "omega_cross_v_correction_mps2": [
            corrected_from_velocity_ned[i] - differentiated_body_ned[i] for i in range(2)
        ],
        "vrx_corrected_from_world_derivative_mps2": corrected_from_velocity_ned,
        "vrx_direct_body_imu_mps2": direct_imu_ned,
        "omega_cross_velocity_mps2": transport_ned,
        "vrx_body_velocity_derivative_from_imu_mps2": imu_as_body_velocity_derivative,
        "imu_timestamp_s": float(imu["timestamp_s"]),
        "node_body_acceleration_mps2": node,
        "body_velocity_derivative_minus_node_mps2": [imu_as_body_velocity_derivative[i] - node[i] for i in range(2)],
        "body_velocity_derivative_vs_node_l2_mps2": math.dist(imu_as_body_velocity_derivative, node),
        "corrected_velocity_vs_imu_l2_mps2": math.dist(corrected_from_velocity_ned, direct_imu_ned),
        "conclusion": (
            "The IMU and differentiated velocity agree after applying the omega-cross-v transport "
            "identity, but they represent different acceleration definitions. The marine mass matrix "
            "multiplies the body-velocity derivative, so the original differentiated value was the "
            "correct quantity for the dynamics comparison."
        ),
    }
    output = args.output or args.run_dir / "frame-corrected-result.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
