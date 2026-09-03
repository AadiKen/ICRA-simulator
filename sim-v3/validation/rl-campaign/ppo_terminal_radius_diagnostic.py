from __future__ import annotations

"""Diagnostic rescore only: no learning, task-contract mutation, or gate result."""
import hashlib
import json
import math
import sys
from pathlib import Path

from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim import CommonWaypointEnv

OUT = ROOT / "artifacts/rl-campaign/p3-local"
MODEL = OUT / "ppo-final.zip"
SWEEP = json.loads((OUT / "terminal-radius-sweep-n200.json").read_text())
RADII = [2, 3, 4, 5, 6, 8, 10]
SEEDS = range(10000, 10050)
model = PPO.load(MODEL, device="cpu")

rows = []
for seed in SEEDS:
    env = CommonWaypointEnv(ROOT, fixed_reset_seed=seed)
    obs, _ = env.reset()
    closest = math.inf
    final_leg_seen = False
    original_success = False
    for step in range(2400):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, info = env.step(action)
        if info["current_waypoint_index"] == 2:
            final_leg_seen = True
            closest = min(closest, float(info["distance_to_final_waypoint_m"]))
        original_success = original_success or bool(info["success"])
        if terminated or truncated:
            break
    rows.append({
        "seed": seed,
        "final_leg_seen": final_leg_seen,
        "closest_final_waypoint_m": None if math.isinf(closest) else closest,
        "pass_through": {str(radius): bool(final_leg_seen and closest <= radius) for radius in RADII},
        "original_terminal_success": original_success,
        "observed_steps": step + 1,
    })
    env.close()

cells = []
for radius in RADII:
    ref = next(c for c in SWEEP["cells"] if c["timeout_s"] == 120 and c["variant"] == "pass_through" and c["radius_m"] == radius)
    ppo_rate = sum(row["pass_through"][str(radius)] for row in rows) / len(rows)
    untrained = ref["rates"]["untrained"]
    classical = ref["rates"]["best_classical"]
    denominator = classical - untrained
    cells.append({
        "radius_m": radius,
        "ppo_success_rate": ppo_rate,
        "reference_rates": {"untrained": untrained, "best_classical": classical, "best_classical_policy": ref["best_classical_policy"]},
        "normalized_success_score": None if denominator == 0 else (ppo_rate - untrained) / denominator,
        "ppo_below_untrained": ppo_rate < untrained,
    })

selected = SWEEP.get("selected_candidate")
selected_diagnostic = next((c for c in cells if selected and c["radius_m"] == selected["radius_m"]), None)
report = {
    "schema_version": 1,
    "artifact_kind": "ppo-terminal-radius-diagnostic",
    "status": "complete-diagnostic-only-halt-before-contract-revision",
    "constraints": {
        "new_training_performed": False,
        "task_contract_modified": False,
        "gate_3_result": False,
        "model_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        "evaluation_seeds": [10000, 10049],
        "episodes": len(rows),
        "horizon_s": 120,
        "terminal_variant": "pass_through",
    },
    "source_n200_sweep_sha256": hashlib.sha256((OUT / "terminal-radius-sweep-n200.json").read_bytes()).hexdigest(),
    "cells": cells,
    "selected_candidate_diagnostic": selected_diagnostic,
    "warning": "This checkpoint was trained under prior 2 m advancement and terminal semantics. This rescore is diagnostic only and is not a Gate 3 result; P3-v3 requires fresh training after an approved, re-hashed contract.",
    "raw": rows,
    "decision": {"contract_revision_applied": False, "human_approval_required": True},
}
path = OUT / "ppo-terminal-radius-diagnostic.json"
tmp = Path(str(path) + ".tmp")
tmp.write_text(json.dumps(report, indent=2) + "\n")
tmp.replace(path)
print(json.dumps({"selected_candidate_diagnostic": selected_diagnostic, "cells": cells}, indent=2))
