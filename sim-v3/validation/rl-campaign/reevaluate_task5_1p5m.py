#!/usr/bin/env python3
"""Re-evaluate the saved Task-5 1.5M checkpoint into the 15-field schema."""
from __future__ import annotations
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "packages/python-client"))
sys.path.insert(0, str(HERE))

from run_task5_conditions import Task5ConditionEnv  # noqa: E402
from train_portable_ppo import atomic_json, detect_host_class, evaluate_recurrent  # noqa: E402

CHECKPOINT = ROOT / "artifacts/rl-campaign/surveyor/task-5-default-noise-ent-0p005-phase1-1p5m/seed-7319/phase1-1500000.zip"
OUTPUT = CHECKPOINT.parent / "phase1-1500000-episode-schema-v1.json"


def main():
    from sb3_contrib import RecurrentPPO
    model = RecurrentPPO.load(CHECKPOINT, device="cpu")
    result = evaluate_recurrent(
        model,
        lambda seed: Task5ConditionEnv(ROOT, fixed_reset_seed=seed,
                                       final_leg_curriculum=True,
                                       condition="default-noise"),
        30000, 50,
    )
    artifact = {
        "schema_version": 1,
        "artifact_kind": "task-5-checkpoint-schema-reevaluation",
        "status": "complete",
        "training_performed": False,
        "checkpoint": str(CHECKPOINT.relative_to(ROOT)),
        "checkpoint_sha256": hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest(),
        "evaluation_seeds": [30000, 30049],
        "host_class": detect_host_class(),
        "paper_measurement_eligible": False,
        "eligibility_reason": "Local checkpoint re-evaluation repairs row schema but is not a cluster headline measurement.",
        "evaluation": result,
    }
    atomic_json(OUTPUT, artifact)
    print(json.dumps({"output": str(OUTPUT.relative_to(ROOT)),
                      "success_rate": result["success_rate"],
                      "rows": len(result["rows"]),
                      "host_class": artifact["host_class"]}, indent=2))


if __name__ == "__main__":
    main()
