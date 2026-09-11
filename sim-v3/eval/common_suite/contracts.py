from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

CONTRACT_DIR = Path(__file__).with_name("contracts")
HASH_EXCLUDED = frozenset({"content_sha256", "hash_scope"})


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def content_hash(document: dict[str, Any]) -> str:
    payload = {key: value for key, value in document.items() if key not in HASH_EXCLUDED}
    return hashlib.sha256(canonical_bytes(payload)).hexdigest()


@dataclass(frozen=True)
class TaskContract:
    path: Path
    document: dict[str, Any]

    @property
    def condition_id(self) -> str:
        return str(self.document["condition_id"])

    @property
    def seeds(self) -> tuple[int, ...]:
        spec = self.document["episode_seeds"]
        if isinstance(spec, list):
            return tuple(int(seed) for seed in spec)
        if spec.get("generator") != "contiguous-v1":
            raise ValueError(f"{self.path}: unsupported episode seed generator")
        first, count = int(spec["first"]), int(spec["count"])
        return tuple(range(first, first + count))

    @property
    def content_sha256(self) -> str:
        return str(self.document["content_sha256"])

    @property
    def requires_sensors(self) -> bool:
        return bool(self.document.get("sensing", {}).get("degraded", False))

    def verify(self) -> None:
        required = {"schema_version", "contract_version", "condition_id", "episode_seeds", "content_sha256"}
        missing = required - self.document.keys()
        if missing:
            raise ValueError(f"{self.path}: missing fields {sorted(missing)}")
        if len(self.seeds) != len(set(self.seeds)) or not self.seeds:
            raise ValueError(f"{self.path}: episode seeds must be non-empty and unique")
        actual = content_hash(self.document)
        if actual != self.content_sha256:
            raise ValueError(f"{self.path}: content hash mismatch ({actual} != {self.content_sha256})")


def load_contracts(directory: Path = CONTRACT_DIR) -> list[TaskContract]:
    contracts = [TaskContract(path, json.loads(path.read_text())) for path in sorted(directory.glob("*.json"))]
    if len(contracts) != 5:
        raise ValueError(f"Expected exactly five common-suite contracts, found {len(contracts)}")
    for contract in contracts:
        contract.verify()
    seed_sets = {contract.seeds for contract in contracts}
    if len(seed_sets) != 1:
        raise ValueError("Every condition must use the identical versioned episode-seed set")
    return contracts
