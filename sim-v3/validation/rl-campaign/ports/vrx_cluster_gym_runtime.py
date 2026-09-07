#!/usr/bin/env python3
"""Native in-container VRX runtime for CodeNimbus/Pyxis Gate D measurements."""
from __future__ import annotations

import json
import math
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

from vrx_gym_runtime import (
    CONTRACT_TICK_S, ODOMETRY_YAW_RATE_WARMUP_S, PHYSICS_STEPS_PER_TICK,
    ExternalGpsModel, ROOT, Runtime as DockerVrxRuntime, TerminationMonitor,
)


class DirectJsonTopic:
    def __init__(self, _container, topic):
        self.topic = topic
        self.latest = None
        self.items = queue.Queue()
        self.stalled = False
        self.accepted = []
        self.parse_errors = 0
        self.last_unparsed = None
        self.process = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic, "--json-output"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        assert self.process.stdout
        for line in self.process.stdout:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                self.parse_errors += 1
                self.last_unparsed = line[:500]
                continue
            if not self.stalled:
                self.latest = value
                self.items.put(value)
                self.accepted.append(value)

    def wait_at_least(self, target, extract, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = self.latest
            if value is not None and extract(value) >= target - 1e-9:
                return value
            try:
                self.items.get(timeout=min(.1, max(0, deadline - time.monotonic())))
            except queue.Empty:
                pass
        raise TimeoutError(f"{self.topic} did not reach simulation time {target}")

    def wait_after(self, previous, extract, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = self.latest
            if value is not None and extract(value) > previous + 1e-9:
                return value
            try:
                self.items.get(timeout=min(.1, max(0, deadline - time.monotonic())))
            except queue.Empty:
                pass
        raise TimeoutError(f"{self.topic} did not publish a sample newer than {previous}")

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()


class Runtime(DockerVrxRuntime):
    """Run Gazebo directly inside one isolated Pyxis container allocation."""

    def __init__(self):
        partition = f"bcod-vrx-gate-d-{os.getpid()}"
        os.environ["GZ_PARTITION"] = partition
        super().__init__()
        self.gz_server = None
        self.partition = partition

    def dexec(self, *args, **kwargs):
        return self.run(list(args), **kwargs)

    def _wait_service(self):
        # Sixteen simultaneous native servers can legitimately take longer
        # than the local single-container 30 s discovery allowance.
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            out = self.dexec("gz", "service", "-l", check=False, timeout=5)
            if self.world + "/control" in out.stdout:
                return
            if self.gz_server is not None and self.gz_server.poll() is not None:
                error = self.gz_server.stderr.read() if self.gz_server.stderr else ""
                raise RuntimeError(f"Gazebo server exited during discovery: {error[-2000:]}")
            time.sleep(.1)
        raise TimeoutError("Gazebo world control service did not appear within 180 s")

    def _stop(self):
        for topic in self.topics.values():
            topic.close()
        self.topics = {}
        for proc in (self.actuator, self.converter, self.transport, self.gz_server):
            if proc and proc.poll() is None:
                proc.terminate()
        for proc in (self.actuator, self.converter, self.transport, self.gz_server):
            if proc:
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self.actuator = self.converter = self.transport = self.gz_server = None
        if self.temp:
            self.temp.cleanup()
            self.temp = None

    def reset(self, config):
        self._stop()
        self.temp = tempfile.TemporaryDirectory(prefix="bcod-vrx-cluster-runtime-")
        seed = int(config["experiment"]["seed"])
        out = self.temp.name
        self.environment_requested = config.get("environment", {})
        environment_scale = 0 if all(
            float(x) == 0
            for key in ("current_mps", "wind_mps")
            for x in self.environment_requested.get(key, [0, 0, 0])
        ) else 1
        self.surface_mode = "both"
        self.surface_enabled = True
        self.run([
            "node", "--experimental-strip-types",
            str(ROOT / "validation/rl-campaign/ports/prepare-vrx-episode.ts"),
            str(seed), out, str(environment_scale), "1",
            str(environment_scale), str(environment_scale), "none", "both",
        ])
        schedule = json.loads((Path(out) / "transport.json").read_text())
        disturbance = schedule["reset"]["disturbance"]
        current_angle = math.radians(disturbance["current_direction_deg"])
        wind_angle = math.radians(disturbance["wind_direction_deg"])
        self.environment_applied = {
            "current_mps": [
                environment_scale * disturbance["current_speed_m_s"] * math.cos(current_angle),
                environment_scale * disturbance["current_speed_m_s"] * math.sin(current_angle), 0,
            ],
            "wind_mps": [
                environment_scale * disturbance["wind_speed_m_s"] * math.cos(wind_angle),
                environment_scale * disturbance["wind_speed_m_s"] * math.sin(wind_angle), 0,
            ],
            "wave": {"gain": 0, "mechanism": "flat-water common-task protocol"},
        }
        shutil.copy(
            ROOT / "validation/rl-campaign/ports/gazebo_transport_jsonl.py",
            Path(out) / "gazebo_transport_jsonl.py",
        )
        self.world = "/world/surveyor_vrx"
        child_env = os.environ.copy()
        child_env.update({
            "GZ_SIM_RESOURCE_PATH": f"{out}/models",
            "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib",
            "LD_LIBRARY_PATH": "/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib:/opt/ros/jazzy/lib",
        })
        os.environ.update({key: child_env[key] for key in (
            "GZ_SIM_RESOURCE_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH", "LD_LIBRARY_PATH",
        )})
        self.gz_server = subprocess.Popen(
            ["gz", "sim", "-s", str(Path(out) / "world.sdf")],
            env=child_env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
        )
        self._wait_service()
        self.service(self.world + "/control", "gz.msgs.WorldControl", "pause: true")
        self.transport = subprocess.Popen(
            ["python3", "-u", str(Path(out) / "gazebo_transport_jsonl.py")],
            text=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        )
        initial = config["initial_state"]["position_ned_m"]
        wanted = {
            "clock": self.world + "/clock", "odom": "/odometry", "imu": "/imu", "gps": "/gps",
            "contact0": "/surveyor/contacts/hull_0", "contact1": "/surveyor/contacts/hull_1",
            "world_contacts": self.world + "/physics/contacts",
        }
        self.topics = {name: DirectJsonTopic(self.container, topic) for name, topic in wanted.items()}
        time.sleep(1.)
        self.actuator = subprocess.Popen(
            ["node", "--experimental-strip-types", str(ROOT / "validation/rl-campaign/ports/actuator-jsonl-bridge.ts")],
            cwd=ROOT, text=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        )
        self.converter = subprocess.Popen(
            ["node", "--experimental-strip-types", str(ROOT / "validation/rl-campaign/ports/task-trace-jsonl-bridge.ts")],
            cwd=ROOT, text=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        )
        self._node_request(self.actuator, {"op": "reset"})
        self.termination = TerminationMonitor()
        self.roll_rad = self.pitch_rad = 0.
        self.diagnostic_termination_override = None
        self.imu_filter_stages = [[0., 0., 0.] for _ in range(4)]
        self.imu_filter_timestamp = None
        self.imu_filter_output = [0., 0., 0.]
        self.gps_model = ExternalGpsModel(
            seed, initial[0], initial[1],
            reference_latitude_deg=-33.72276876888639,
            reference_longitude_deg=150.67399110174387,
        )
        self.last_gps_stamp = None
        self.service(self.world + "/control", "gz.msgs.WorldControl", "pause: true multi_step: 1")
        self.topics["clock"].wait_at_least(CONTRACT_TICK_S, self._clock_time)
        self._wait_clock_stable()
        self.sim_time = self._clock_time(self.topics["clock"].latest or {})
        for attempt in range(6):
            try:
                self.topics["odom"].wait_after(-1., self._header_time, timeout=2.)
                break
            except TimeoutError:
                if attempt == 5:
                    raise
                self._advance_confirmed(PHYSICS_STEPS_PER_TICK)
                self.sim_time = self._clock_time(self.topics["clock"].latest or {})
        self.sim_time = self._wait_clock_stable()
        self.odometry_yaw_rate_valid_after_s = self.sim_time + ODOMETRY_YAW_RATE_WARMUP_S
        return self.response()

    def handle(self, request):
        response = super().handle(request)
        if request.get("op") == "diagnostic_status":
            response["cluster_runtime"] = {
                "execution": "native x86_64 Pyxis/Enroot; Gazebo launched directly",
                "gz_partition": self.partition,
            }
        return response


def main():
    runtime = Runtime()
    try:
        for line in sys.stdin:
            try:
                response = runtime.handle(json.loads(line))
            except Exception as error:
                response = {"ok": False, "error": f"{type(error).__name__}: {error}"}
            print(json.dumps(response, separators=(",", ":")), flush=True)
            if response.get("closed"):
                break
    finally:
        runtime._stop()


if __name__ == "__main__":
    main()
