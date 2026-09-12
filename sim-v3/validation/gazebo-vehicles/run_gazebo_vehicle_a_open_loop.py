#!/usr/bin/env python3
"""Replay one Vehicle A schedule with iteration-authoritative Gazebo stepping."""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node


def seconds(value) -> float:
    return value.sec + value.nsec * 1e-9


def wait_until(predicate, timeout_s: float, message: str) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.002)
    if not predicate():
        raise RuntimeError(message)


def publish_wrench(publisher, entity_name: str, command: dict, yaw_ned: float) -> None:
    body = command["body_wrench_n_nm"]
    surge, sway, yaw = body["surge_n"], body["sway_n"], body["yaw_nm"]
    cosine, sine = math.cos(yaw_ned), math.sin(yaw_ned)
    message = EntityWrench()
    message.entity.name = entity_name
    message.entity.type = 3  # gz.msgs.Entity.LINK
    message.wrench.force.x = surge * sine + sway * cosine
    message.wrench.force.y = surge * cosine - sway * sine
    message.wrench.torque.z = -yaw
    publisher.publish(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--schedule-source", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    manifest_path = Path(args.manifest).resolve()
    manifest = json.loads(manifest_path.read_text())
    old_comparison = json.loads(Path(args.schedule_source).read_text())
    scenario = next((item for item in old_comparison["scenarios"] if item["id"] == manifest["maneuver"]), None)
    if scenario is not None:
        schedule = scenario["actuation"]["requested_schedule"]
    elif manifest["maneuver"] == "impulse-hold":
        schedule = [{
            "time_s": index * manifest["dt"],
            "body_wrench_n_nm": {"surge_n": 60 if index * manifest["dt"] < 1 else 0, "sway_n": 0, "yaw_nm": 0},
        } for index in range(manifest["steps"])]
    else:
        raise ValueError(f"No schedule source found for {manifest['maneuver']}")
    if len(schedule) != manifest["steps"] or manifest["dt"] != 0.05:
        raise ValueError("Vehicle A comparison requires one schedule row per 50 ms sample")

    generated = manifest_path.parent.parent
    world = generated / manifest["gazebo"]["world"]
    resource_path = str(generated / "models")
    environment = dict(os.environ)
    environment["GZ_SIM_RESOURCE_PATH"] = resource_path + (os.pathsep + environment["GZ_SIM_RESOURCE_PATH"] if environment.get("GZ_SIM_RESOURCE_PATH") else "")
    world_name = f"bcod_parity_{manifest['maneuver']}"
    server = subprocess.Popen(
        ["gz", "sim", "--force-version", "8", "-s", "-v", "1", str(world)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=environment,
    )
    try:
        poses, stats = [], []
        node = Node()
        pose_topic = f"/world/{world_name}/dynamic_pose/info"
        stats_topic = f"/world/{world_name}/stats"
        node.subscribe(Pose_V, pose_topic, lambda message, *_: poses.append(message))
        node.subscribe(WorldStatistics, stats_topic, lambda message, *_: stats.append(message))
        # A non-persistent wrench is applied once to the next physics iteration.
        # This is the exact discrete counterpart of one body-wrench command per step.
        wrench_publisher = node.advertise(manifest["gazebo"]["wrenchTopic"], EntityWrench)
        current_publisher = node.advertise(manifest["gazebo"]["currentTopic"], Vector3d)
        wait_until(lambda: bool(stats), 20, "Gazebo world statistics did not appear")
        time.sleep(0.1)  # Allow transport discovery before the first commanded step.

        before_pause = len(stats)
        node.request(f"/world/{world_name}/control", WorldControl(pause=True), WorldControl, Boolean, 5000)
        wait_until(lambda: len(stats) > before_pause and stats[-1].paused, 5, "Gazebo did not enter paused mode")
        node.request(f"/world/{world_name}/control", WorldControl(pause=True, multi_step=1), WorldControl, Boolean, 5000)
        stable_since, last_iteration = time.monotonic(), stats[-1].iterations
        while time.monotonic() - stable_since < 0.2:
            time.sleep(0.002)
            if stats[-1].iterations != last_iteration:
                last_iteration, stable_since = stats[-1].iterations, time.monotonic()
        initial_iteration = stats[-1].iterations
        initial_time = seconds(stats[-1].sim_time)

        current = manifest["gazebo"].get("currentEnu", {})
        current_publisher.publish(Vector3d(x=current.get("x", 0), y=current.get("y", 0), z=current.get("z", 0)))
        captured = []
        yaw_ned = 0.0
        for index, command in enumerate(schedule):
            publish_wrench(wrench_publisher, manifest["gazebo"]["wrenchEntity"]["name"], command, yaw_ned)
            time.sleep(0.01)  # Deliver the command while paused; this is not a clock or retry condition.
            target = initial_iteration + index + 1
            if stats[-1].iterations != target - 1:
                raise RuntimeError(f"Unexpected pre-step iteration {stats[-1].iterations}; expected {target - 1}")
            before_poses = len(poses)
            # Exactly one request is issued. Transport acknowledgement is not used as
            # a retry signal; the authoritative absolute iteration target is decisive.
            node.request(f"/world/{world_name}/control", WorldControl(pause=True, multi_step=1), WorldControl, Boolean, 100)
            wait_until(lambda: stats[-1].iterations >= target, 10, f"Gazebo did not reach target iteration {target}")
            if stats[-1].iterations != target:
                raise RuntimeError(f"Gazebo overshot target iteration {target}: {stats[-1].iterations}")
            wait_until(lambda: len(poses) > before_poses, 2, "Gazebo pose stream did not advance")
            elapsed = seconds(stats[-1].sim_time) - initial_time
            expected = (index + 1) * 0.05
            if abs(elapsed - expected) > 1e-9:
                raise RuntimeError(f"Simulation-time mismatch at sample {index}: {elapsed} != {expected}")
            named = {pose.name: pose for pose in poses[-1].pose}
            message = named.get("otter")
            if message is None:
                raise RuntimeError("Gazebo dynamic pose did not contain the otter model")
            q = message.orientation
            yaw_enu = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            yaw_ned = math.atan2(math.sin(math.pi / 2 - yaw_enu), math.cos(math.pi / 2 - yaw_enu))
            captured.append({
                "step": index,
                "time_s": elapsed,
                "gazebo_iteration": stats[-1].iterations,
                "gazebo_sim_time_s": elapsed,
                "iteration_delta": 1,
                "sim_time_delta_s": 0.05,
                "north_m": message.position.y,
                "east_m": message.position.x,
                "heading_rad": yaw_ned,
                "command": command,
            })

        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps({
            "schema_version": 2,
            "scenario": manifest["maneuver"],
            "synchronization": {
                "source": stats_topic,
                "initial_iteration": initial_iteration,
                "initial_sim_time_s": initial_time,
                "target_policy": "absolute initial_iteration + (sample_index+1)",
                "blind_retries": False,
                "step_requests_per_target": 1,
                "asserted_iteration_delta": 1,
                "asserted_sim_time_delta_s": 0.05,
                "iteration_delta_reason": "Vehicle A's Gazebo worlds use max_step_size=0.05 s, so one physics iteration equals the 0.05 s sample interval; Vehicle C uses a 0.01 s physics step and therefore requires five iterations.",
            },
            "rows": captured,
        }, indent=2) + "\n")
    finally:
        server.send_signal(2)
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
