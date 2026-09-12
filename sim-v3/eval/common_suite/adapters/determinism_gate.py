from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np

from .base import PolicyAdapter


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


@dataclass(frozen=True)
class GateResult:
    arm: str
    status: str
    passed: bool
    bit_identical: bool
    first_divergence_step: int | None
    max_abs_divergence: float
    reference_sha256: str
    candidate_sha256: str
    notes: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_determinism_gate(adapter: PolicyAdapter, replay_rows: Iterable[Mapping[str, Any]], *, atol: float = 1e-6) -> GateResult:
    """Round-trip a native replay. One row is consumed once: no simulator stepping occurs here."""
    reference, candidate, first, maximum = [], [], None, 0.0
    for step, row in enumerate(replay_rows):
        native_obs = dict(row["observation"])
        shared_obs = adapter.native_obs_to_shared(native_obs)
        obs_roundtrip = adapter.shared_obs_to_native(shared_obs)
        native_action = dict(row["action"])
        shared_action = adapter.native_action_to_shared(native_action)
        # Compare the static command transform without advancing the lag state a second time.
        action_roundtrip = adapter.shared_action_to_native(shared_action)
        ref_values = [float(native_obs[key]) for key in obs_roundtrip] + [float(native_action[key]) for key in action_roundtrip]
        got_values = list(obs_roundtrip.values()) + list(action_roundtrip.values())
        difference = float(np.max(np.abs(np.asarray(ref_values) - np.asarray(got_values))))
        maximum = max(maximum, difference)
        if difference > atol and first is None:
            first = step
        reference.append({"observation": {key: native_obs[key] for key in obs_roundtrip}, "action": {key: native_action[key] for key in action_roundtrip}})
        candidate.append({"observation": obs_roundtrip, "action": action_roundtrip})
    ref_hash = hashlib.sha256(_canonical(reference)).hexdigest()
    got_hash = hashlib.sha256(_canonical(candidate)).hexdigest()
    bit_identical = ref_hash == got_hash
    passed = first is None
    return GateResult(adapter.arm, "passed" if passed else "failed", passed, bit_identical, first, maximum, ref_hash, got_hash, "bit-identical" if bit_identical else ("within expected float tolerance" if passed else "round-trip divergence exceeds tolerance"))


def load_replay(path: Path) -> list[dict[str, Any]]:
    if path.suffix == ".jsonl":
        return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    value = json.loads(path.read_text())
    return value["rows"] if isinstance(value, dict) else value
