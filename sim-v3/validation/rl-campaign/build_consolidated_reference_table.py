"""Build the approval-facing n=200 portable-task reference summary.

The retained replay logs terminal outcomes and final-leg activation, but not
returns.  This script deliberately records that distinction rather than
backfilling a value from another experiment.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "artifacts/rl-campaign/consolidated-reference-table.json"
PARTS = [ROOT / f"artifacts/rl-campaign/p3-local/reference-replay-part-{i}.json" for i in range(4)]
POLICIES = ("frozen-untrained-policy-v1", "LOS-PID-v2", "LOS-SPEEDCAP-v2")
RADII = (2, 3, 4, 5, 6, 8, 10, 12, 15)


def quantiles(values: list[float]) -> dict[str, float | int]:
    values = sorted(values)
    return {
        "count": len(values), "min": values[0], "q1": float(np.quantile(values, .25)),
        "median": float(np.quantile(values, .5)), "q3": float(np.quantile(values, .75)),
        "p95": float(np.quantile(values, .95)), "max": values[-1],
    }


def ci(values: np.ndarray, rng: np.random.Generator, samples: int = 20_000) -> list[float]:
    draws = values[rng.integers(0, len(values), size=(samples, len(values)))].mean(axis=1)
    return [float(x) for x in np.quantile(draws, [.025, .975])]


def main() -> None:
    raw = [row for part in PARTS for row in json.loads(part.read_text())["raw"]]
    by_policy = {p: sorted((r for r in raw if r["policy"] == p), key=lambda r: r["seed"]) for p in POLICIES}
    assert all(len(rows) == 200 for rows in by_policy.values())
    rng = np.random.default_rng(20270901)
    controllers = {}
    for policy, rows in by_policy.items():
        # The raw diagnostic kept running to 240 s, while the selected task
        # timeout is 120 s.  Activation must use that same horizon.
        activation = np.array([r["final_leg_time_s"] is not None and r["final_leg_time_s"] <= 120 for r in rows], dtype=float)
        closest = [r["closest_final_m_120"] for r in rows if r["closest_final_m_120"] is not None]
        controllers[policy] = {
            "episodes": len(rows),
            "final_leg_activation_rate": float(activation.mean()),
            "final_leg_activation_rate_95_ci": ci(activation, rng),
            "closest_final_m": {
                "conditioned_on_final_leg_activation": True,
                "timeout_s": 120,
                "distribution": quantiles(closest),
                "interpretation_guard": "Do not interpret this as an unconditional terminal-accuracy distribution; episodes without final-leg activation contribute no closest_final_m value.",
            },
            "median_return": {"status": "not_obtained", "reason": "The completed n=200 reference replay retained terminal/activation traces but did not log rewards; no return is inferred from terminal data."},
        }
    cells = []
    for radius in RADII:
        rates = {p: np.array([bool(r["pass_through_120"][str(radius)]) for r in by_policy[p]], dtype=float) for p in POLICIES}
        best = "LOS-PID-v2" if rates["LOS-PID-v2"].mean() >= rates["LOS-SPEEDCAP-v2"].mean() else "LOS-SPEEDCAP-v2"
        gap_draws = []
        # paired seed bootstrap: all policies share the identical 200 fixed episodes.
        for _ in range(20_000):
            index = rng.integers(0, 200, size=200)
            gap_draws.append(float(rates[best][index].mean() - rates["frozen-untrained-policy-v1"][index].mean()))
        cells.append({
            "radius_m": radius, "timeout_s": 120, "variant": "pass-through",
            "success_rates": {p: float(v.mean()) for p, v in rates.items()},
            "best_classical_policy": best,
            "best_classical_minus_untrained": float(rates[best].mean() - rates["frozen-untrained-policy-v1"].mean()),
            "gap_95_ci": [float(x) for x in np.quantile(gap_draws, [.025, .975])],
        })
    selected = next(cell for cell in cells if cell["radius_m"] == 6)
    result = {
        "schema_version": 1,
        "artifact_kind": "consolidated-portable-task-reference-table",
        "status": "complete-awaiting-human-approval-for-contract",
        "nomenclature": {"LOS-SPEEDCAP-v2": "Line-of-sight controller with speed cap; formerly mislabeled LOS-MPC-v2. MPC-v1 is a separate parked-incomplete track."},
        "seed_set": {"first": 10000, "count": 200, "shared_across_policies": True},
        "terminal_metric": {
            "definition": "Pass-through: after ordered leg advancement, enter the final-waypoint radius at any speed; no hold requirement.",
            "timeout_s": 120,
            "reporting_requirement": "Every policy result must report final_leg_activation_rate alongside success_rate.",
            "load_bearing_interpretation": "The closest_final_m statistics are conditional on final-leg activation. The success plateau therefore primarily measures completion of earlier legs plus terminal entry, not unconditional terminal-approach accuracy.",
        },
        "controllers": controllers,
        "radius_cells": cells,
        "selected_terminal_definition": {
            "radius_m": 6, "variant": "pass-through", "timeout_s": 120,
            "approval_status": "human-approved-selection; contract revision pending hash review",
            "selection_basis": "Smallest qualifying radius under the approved 120 s pass-through structure; LOS-SPEEDCAP-v2 success 40%, frozen-untrained success 8%, gap 32 percentage points.",
            "limitation": "The best classical rate is at the 40% band floor and its 95% bootstrap interval reaches below 40%; this is reported rather than resolved by choosing a looser radius.",
        },
        "success_calibration": {
            "method": "Calibrated before fresh RL evaluation so the best classical reference has a non-degenerate success rate; this is criterion calibration, not fit to PPO outcomes.",
            "formula": "S_untrained + 0.5 * (S_reference - S_untrained)",
            "untrained_rate": selected["success_rates"]["frozen-untrained-policy-v1"],
            "reference_policy": selected["best_classical_policy"],
            "reference_rate": selected["success_rates"][selected["best_classical_policy"]],
            "normalized_threshold": .5,
            "absolute_success_rate_threshold": .24,
            "reference_minus_untrained_95_ci": selected["gap_95_ci"],
            "freeze_rule": "Frozen before P3-v3 training; no change based on RL results.",
        },
        "mpc_v1": {"status": "parked-incomplete", "reason": "MPC-v1 structural defects were identified but the prescribed single-solve diagnostic was not run. It is excluded from calibration and is not a downstream dependency.", "handoff": "artifacts/rl-campaign/mpc-v1-handoff.md"},
    }
    OUT.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(OUT), "selected": result["selected_terminal_definition"], "threshold": result["success_calibration"]}, indent=2))


if __name__ == "__main__":
    main()
