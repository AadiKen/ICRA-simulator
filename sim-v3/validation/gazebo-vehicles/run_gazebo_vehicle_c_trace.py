#!/usr/bin/env python3
"""Replay a two-pod Vehicle C force/azimuth schedule in fixed-step Gazebo."""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from pathlib import Path

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.world_control_pb2 import WorldControl


def call(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, text=True, capture_output=True, check=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--schedule", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--world", required=True)
    args = parser.parse_args()
    samples = json.loads(Path(args.schedule).read_text())["samples"]
    model = "vehicle-c-azimuth"
    server = subprocess.Popen(
        ["gz", "sim", "--force-version", "8", "-s", "-v", "1", args.world],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        odom_topic = f"/model/{model}/odometry"
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if odom_topic in call(["gz", "topic", "-l"]).stdout:
                break
            time.sleep(.2)
        else:
            raise RuntimeError("Gazebo odometry topic did not appear")

        messages: list[Odometry] = []
        node = Node()
        node.subscribe(Odometry, odom_topic, lambda message, *_: messages.append(message))
        publishers = {}
        for side in ("port", "starboard"):
            publishers[f"{side}_thrust"] = node.advertise(
                f"/model/{model}/joint/{side}_propeller_joint/cmd_thrust", Double)
            publishers[f"{side}_azimuth"] = node.advertise(
                f"/model/{model}/joint/{side}_azimuth_joint/0/cmd_pos", Double)
        time.sleep(.3)
        for index, row in enumerate(samples):
            for side in ("port", "starboard"):
                publishers[f"{side}_thrust"].publish(Double(data=row[f"{side}_thrust_n"]))
                # bcod uses body NED / FRD (positive azimuth toward starboard),
                # while SDF joints use FLU (positive rotation toward port).
                publishers[f"{side}_azimuth"].publish(Double(data=-row[f"{side}_azimuth_rad"]))
            ok, response = node.request(
                "/world/vehicle_c_check/control", WorldControl(multi_step=5),
                WorldControl, Boolean, 5000)
            if not ok or not response.data:
                raise RuntimeError("Gazebo fixed-step request failed")
            deadline = time.monotonic() + 2
            while len(messages) < index + 1 and time.monotonic() < deadline:
                time.sleep(.002)

        deadline = time.monotonic() + 10
        while len(messages) < len(samples) and time.monotonic() < deadline:
            time.sleep(.02)
        if len(messages) != len(samples):
            raise RuntimeError(f"Expected {len(samples)} odometry rows, received {len(messages)}")

        rows = []
        for index, message in enumerate(messages):
            pose, twist = message.pose, message.twist
            q = pose.orientation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
            rows.append({
                "step": index, "time_s": index * .05,
                "enu": {"x": pose.position.x, "y": pose.position.y,
                        "yaw_rad": yaw, "vx": twist.linear.x,
                        "vy": twist.linear.y, "angular_z": twist.angular.z},
                "command": samples[index],
            })
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({"schema_version": 1, "rows": rows}, indent=2) + "\n")
    finally:
        server.send_signal(2)
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
