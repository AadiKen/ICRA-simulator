"""Load either the legacy training task or a common-suite condition contract."""

from __future__ import annotations

import hashlib
import json
from copy import deepcopy
from pathlib import Path
from typing import Any


LEGACY_CONTRACT_SHA256 = "2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36"


def _content_hash(document: dict[str, Any]) -> str:
    payload = {
        key: value for key, value in document.items()
        if key not in {"content_sha256", "hash_scope"}
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode()).hexdigest()


def load_native_task_contract(
    repository: str | Path,
    condition_contract_path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return normalized task data plus binding metadata.

    Existing training callers retain the legacy contract. Gate 1 calibration
    callers explicitly pass a common-suite condition file; portable observation,
    action, and reward semantics are inherited while task-defining fields are
    taken from that condition.
    """
    root = Path(repository)
    legacy_path = root / "artifacts/rl-campaign/surveyor/task-contract-frozen.json"
    legacy = json.loads(legacy_path.read_text())
    if legacy.get("content_sha256") != LEGACY_CONTRACT_SHA256:
        raise ValueError("unexpected legacy portable-task contract hash")
    task = deepcopy(next(
        item for item in legacy["tasks"]
        if item["task_id"] == "common-waypoint-transit-v1"
    ))
    if condition_contract_path is None:
        return task, {
            "source": str(legacy_path),
            "content_sha256": legacy["content_sha256"],
            "condition_id": None,
        }

    path = Path(condition_contract_path)
    if not path.is_absolute():
        path = root / path
    condition = json.loads(path.read_text())
    actual_hash = _content_hash(condition)
    if actual_hash != condition.get("content_sha256"):
        raise ValueError(f"condition contract hash mismatch: {actual_hash}")
    if condition.get("condition_id") != "nominal":
        raise ValueError("Gate 1 native calibration requires the nominal condition")

    route = condition["route"]["waypoints_local_m"]
    if len(route) < 2:
        raise ValueError("nominal route requires a start and at least one goal")
    origin_n, origin_e = map(float, route[0])
    task["reset_randomization"].update({
        "route_rotation_deg": [0.0, 0.0],
        "start_position_offset_m": [0.0, 0.0],
        "route_relative_m": [
            [float(n) - origin_n, float(e) - origin_e] for n, e in route[1:]
        ],
        "current_speed_m_s": list(map(float, condition["disturbance"]["current_mps"])),
        "wind_speed_m_s": list(map(float, condition["disturbance"]["wind_mps"])),
    })
    time_cap_s = float(condition["termination"]["time_cap_s"])
    physics_dt = float(task["timing"]["physics_timestep_s"])
    episode_steps = round(time_cap_s / physics_dt)
    if not abs(episode_steps * physics_dt - time_cap_s) <= 1e-12:
        raise ValueError("time cap must be an integer number of physics steps")
    task["timing"].update({
        "episode_length_steps": episode_steps,
        "episode_length_s": time_cap_s,
    })
    terminal = task["learnability"]["absolute_success_rate_threshold"]["terminal_definition"]
    terminal.update({
        "radius_m": float(condition["termination"]["success_radius_m"]),
        "timeout_s": time_cap_s,
    })
    task["evaluation_termination"] = deepcopy(condition["termination"])
    return task, {
        "source": str(path),
        "content_sha256": condition["content_sha256"],
        "condition_id": condition["condition_id"],
    }
