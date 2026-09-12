"""Closed-loop Gym adapters for normalized external-simulator runtimes.

The runtime process is a persistent JSON-lines service.  It owns the simulator
lifecycle and must implement `reset`, `step`, `truth`, and `close`.  This keeps
the Gym/task implementation identical while allowing VRX to use ROS 2 and
standalone Gazebo to use Gazebo Transport inside their pinned containers.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
from typing import Sequence

from .common_task_env import CommonWaypointEnv


class ExternalRuntimeError(RuntimeError):
    pass


class JsonLineSimulatorBridge:
    """Adapt a persistent normalized simulator process to Node bridge semantics."""
    def __init__(self, command: Sequence[str], *, cwd: str | Path | None = None):
        if not command:
            raise ValueError("runtime command must not be empty")
        self.process = subprocess.Popen(
            list(command), cwd=cwd, text=True, bufsize=1,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        self._truth = None

    def _request(self, payload: dict) -> dict:
        if self.process.poll() is not None:
            error = self.process.stderr.read() if self.process.stderr else ""
            raise ExternalRuntimeError(f"simulator runtime exited with {self.process.returncode}: {error}")
        assert self.process.stdin is not None and self.process.stdout is not None
        self.process.stdin.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            error = self.process.stderr.read() if self.process.stderr else ""
            raise ExternalRuntimeError(f"simulator runtime closed its response stream: {error}")
        response = json.loads(line)
        if not response.get("ok", False):
            raise ExternalRuntimeError(str(response.get("error", "runtime request failed")))
        if response.get("truth") is not None:
            self._truth = response["truth"]
        return response

    def reset(self, configs: list[dict]) -> dict:
        if len(configs) != 1:
            raise ValueError("external Gym runtimes support one environment per process")
        return self._request({"op": "reset", "config": configs[0]})

    def step(self, actions: list[dict | None]) -> dict:
        if len(actions) != 1 or actions[0] is None:
            raise ValueError("external Gym runtime requires exactly one live action")
        response = self._request({"op": "step", "action": actions[0]})
        return {
            "observations": response.get("observations", []),
            "terminated": [bool(response.get("terminated", False))],
            "truncated": [bool(response.get("truncated", False))],
            "infos": [response.get("info", {})],
        }

    def ground_truth(self) -> dict:
        if self._truth is None:
            self._request({"op": "truth"})
        if self._truth is None:
            raise ExternalRuntimeError("runtime returned no normalized truth")
        return self._truth

    def close(self) -> None:
        if self.process.poll() is None:
            try:
                self._request({"op": "close"})
                if self.process.stdin is not None:
                    self.process.stdin.close()
            except (BrokenPipeError, ExternalRuntimeError):
                pass
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=5)
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            if stream is not None:
                stream.close()


def _assert_training_eligible(repository: Path, simulator: str) -> None:
    """Refuse external PPO use until conformance and action fairness are approved."""
    if simulator == "vrx":
        result = repository / "artifacts/rl-campaign/vrx-gate7-full/gate-7-path-conformance-result.json"
        document = json.loads(result.read_text()) if result.exists() else {}
        if document.get("status") != "ADOPTED_POST_HOC_30_SEED_VALIDATED" or not document.get("result", {}).get("pass", False):
            raise ExternalRuntimeError("VRX Gate 7 has not passed; closed-loop PPO is blocked")
    else:
        result = repository / "artifacts/rl-campaign/gazebo-still-water-protocol.json"
        document = json.loads(result.read_text()) if result.exists() else {}
        if document.get("status") != "COMPLETE_PASS" or document.get("training_authorized") is not True:
            raise ExternalRuntimeError("Gazebo still-water Gate D has not passed; closed-loop PPO is blocked")
    contract = json.loads((repository / "artifacts/rl-campaign/surveyor/task-contract-frozen.json").read_text())
    tasks = contract.get("tasks", [])
    task = next((item for item in tasks if item.get("task_id") == "common-waypoint-transit-v1"), None)
    fairness = (task or {}).get("action", {}).get("fairness", {})
    if fairness.get("status") == "OPEN_RESERVED_GATE_5":
        raise ExternalRuntimeError("Gate 5 action-space fairness is unresolved; cross-simulator PPO is blocked")


class ExternalCommonWaypointEnv(CommonWaypointEnv):
    def __init__(self, repository: str | Path, runtime_command: Sequence[str], *, simulator: str,
                 allow_unconformant_diagnostic: bool = False, **kwargs):
        root = Path(repository)
        if not allow_unconformant_diagnostic:
            _assert_training_eligible(root, simulator)
        bridge = JsonLineSimulatorBridge(runtime_command, cwd=root)
        super().__init__(root, bridge=bridge, backend_type=simulator, **kwargs)
        self.simulator = simulator
        self.diagnostic_only = allow_unconformant_diagnostic


class VrxGymEnv(ExternalCommonWaypointEnv):
    def __init__(self, repository: str | Path, runtime_command: Sequence[str], **kwargs):
        super().__init__(repository, runtime_command, simulator="vrx", **kwargs)


class GazeboGymEnv(ExternalCommonWaypointEnv):
    def __init__(self, repository: str | Path, runtime_command: Sequence[str], **kwargs):
        super().__init__(repository, runtime_command, simulator="gazebo-harmonic", **kwargs)
