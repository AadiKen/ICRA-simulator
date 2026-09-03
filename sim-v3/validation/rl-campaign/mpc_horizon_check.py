from __future__ import annotations

"""Amendment-8 audit: establish whether the deployed LOS-MPC baseline has a horizon."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "artifacts/rl-campaign/p3-local"
PROTOCOL = ROOT / "artifacts/rl-campaign/p2.5-corrected-corridor-protocol.json"
CALIBRATION = ROOT / "artifacts/rl-campaign/p2.5-corrected-corridor.json"
SWEEP = OUT / "terminal-radius-sweep-n200.json"
SOURCE = ROOT / "validation/rl-campaign/corrected-reference-calibration.ts"

protocol = json.loads(PROTOCOL.read_text())
calibration = json.loads(CALIBRATION.read_text())
sweep = json.loads(SWEEP.read_text())
source = SOURCE.read_text()

assert 'if(run.spec.policy==="LOS-SPEEDCAP-v2")target=Math.min(' in source
assert "prediction_horizon" not in source and "control_horizon" not in source
selected = calibration["tuning"]["los_mpc"]["selected"]
cells = [
    cell for cell in sweep["cells"]
    if cell["timeout_s"] == 120 and cell["variant"] == "pass_through" and cell["radius_m"] in (6, 8)
]

report = {
    "schema_version": 1,
    "artifact_kind": "p3-amendment-8-mpc-horizon-adequacy-check",
    "status": "reference-inadequate-halt-before-radius-selection",
    "constraints": {
        "new_training_performed": False,
        "new_rollouts_performed": False,
        "task_contract_modified": False,
        "radius_selection_applied": False,
    },
    "l0_time_starvation": {
        "conclusion": "ruled_out_for_the_task",
        "evidence": "Existing PPO diagnostic completes 76% of the corridor-corrected routes under 120 s at the provisional 6 m pass-through cell.",
    },
    "deployed_reference": {
        "name": "LOS-SPEEDCAP-v2",
        "actual_formulation": "Line-of-sight steering plus a closed-form speed cap: min(target_speed, sqrt(0.8*distance), target_speed*max(0.25, cos(clamped_heading_error))).",
        "prediction_horizon": None,
        "control_horizon": None,
        "is_receding_horizon_mpc": False,
        "selected_gain_tuple": selected["gains"],
        "source_file": "validation/rl-campaign/corrected-reference-calibration.ts",
    },
    "frozen_tuning_protocol": {
        "held_out_seeds": protocol["held_out_tuning_seeds"],
        "grid_size": 24,
        "swept_parameters": protocol["los"]["grid"],
        "prediction_or_control_horizon_swept": False,
        "reason": "No prediction or control horizon exists in the deployed controller implementation.",
    },
    "previous_6m_8m_rates_not_a_horizon_result": cells,
    "required_l1_horizon_sweep": {
        "performed": False,
        "reason": "Not applicable to the deployed reference: a horizon sweep cannot be performed without first replacing the LOS heuristic with an actual finite-horizon optimizer and predeclaring its model, cost, constraints, solver, prediction/control horizons, and held-out tuning protocol.",
    },
    "transit_time_comparison": {
        "performed": False,
        "reason": "Amendment 8 requests best-horizon LOS-MPC transit times. There is no best-horizon MPC configuration; reporting LOS-heuristic times as an MPC-horizon mechanism would misstate the evidence.",
    },
    "conclusion": {
        "adequacy_check_passed": False,
        "root_cause": "The reference is mislabeled: LOS-SPEEDCAP-v2 is not a receding-horizon controller, so its 40% completion rate cannot establish a genuine MPC capability gap or horizon limitation.",
        "next_authorized_boundary": "Halt. Obtain a decision whether to implement and independently tune a real MPC baseline before radius selection/contract revision, or to relabel and use this controller only as an LOS heuristic (which would require revisiting the stated classical-reference framing).",
    },
    "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    "source_artifacts": {
        "corridor_protocol_sha256": hashlib.sha256(PROTOCOL.read_bytes()).hexdigest(),
        "corridor_calibration_sha256": hashlib.sha256(CALIBRATION.read_bytes()).hexdigest(),
        "n200_radius_sweep_sha256": hashlib.sha256(SWEEP.read_bytes()).hexdigest(),
    },
}
path = OUT / "mpc-horizon-check.json"
tmp = Path(str(path) + ".tmp")
tmp.write_text(json.dumps(report, indent=2) + "\n")
tmp.replace(path)
print(json.dumps({"status": report["status"], "conclusion": report["conclusion"]}, indent=2))
