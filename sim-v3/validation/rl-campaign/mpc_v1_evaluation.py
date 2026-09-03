from __future__ import annotations

"""Amendment-9 held-out tuning and n=200 evaluation for the real MPC-v1 baseline.

Run with `nohup ... > artifacts/rl-campaign/logs/mpc-v1.log 2>&1 &`.  The checkpoint is
atomic and makes the expensive sweep resumable without repeating completed candidates.
"""
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim.linear_mpc import LinearPlanarMPC
from bcod_sim.node_bridge import PersistentNodeBridge
from bcod_sim.common_task_env import Mulberry32

OUT = ROOT / "artifacts/rl-campaign"
LOCAL = OUT / "p3-local"
# Do not reuse pre-LTV candidate results: their frozen-horizon model is a
# different controller formulation.
CHECKPOINT = LOCAL / "mpc-v1-ltv-relinearized-progress.json"
RESULT = OUT / "mpc-v1-evaluation.json"
DT, STEPS = .05, 2400
RADII = [2, 3, 4, 5, 6, 8, 10, 12, 15]
BASE_HORIZONS = [10, 20, 40, 80]
# Amendment 9 boundary expansion: the initial held-out optimum landed at 4.0,
# so add one higher point before allowing n=200 evaluation.
RATIOS = [.25, .5, 1., 2., 4.]


def atomic(path: Path, obj: object) -> None:
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2) + "\n")
    tmp.replace(path)


def episode(seed: int):
    r = Mulberry32(seed); u = lambda a, b: a + (b-a)*r.next()
    angle = math.radians(u(-20, 20)); start = [10000+u(-1, 1), 10000+u(-1, 1)]
    def rot(p): return [start[0]+p[0]*math.cos(angle)-p[1]*math.sin(angle), start[1]+p[0]*math.sin(angle)+p[1]*math.cos(angle)]
    speed, direction, wind, wd = u(0, 1), u(0, 2*math.pi), u(0, 8), u(0, 2*math.pi)
    return start, math.radians(u(-10, 10)), [rot(p) for p in ([20, 0], [35, 15], [50, 5])], [speed*math.cos(direction), speed*math.sin(direction), 0], [wind*math.cos(wd), wind*math.sin(wd), 0]


def config(seed: int):
    start, heading, route, current, wind = episode(seed)
    return {"schema_version": 1, "experiment": {"name": f"mpc-v1-{seed}", "seed": seed, "timestep_s": DT, "duration_s": 120}, "backend": {"type": "node"}, "vehicle": {"preset": "vehicle-a-otter", "plant": "planar3"}, "environment": {"current_mps": current, "wind_mps": wind}, "initial_state": {"position_ned_m": [*start, 0], "attitude_rad": [0, 0, heading]}, "mission": {"type": "rl-common-waypoint-v1", "waypoints": [{"north_m": 19000, "east_m": 19000}]}, "sensors": []}, start, route


def geom(start, route, waypoint, truth):
    a = start if waypoint == 0 else route[waypoint-1]; b = route[waypoint]
    dx, dy = b[0]-a[0], b[1]-a[1]; length = math.hypot(dx, dy); cn, ce = dx/length, dy/length
    n, e = truth["position_ned_m"][:2]; cross = -ce*(n-a[0]) + cn*(e-a[1]); along = (n-a[0])*cn + (e-a[1])*ce
    return math.hypot(b[0]-n, b[1]-e), cross, math.atan2(ce, cn), along >= length and abs(cross) <= 15.361124064575238


def reference(start, route, waypoint, truth, horizon):
    distance, cross, heading, _ = geom(start, route, waypoint, truth)
    desired_heading = heading - math.atan2(cross, 8.)
    # Reuse LOS guidance. Target speed is the reference, not a heuristic speed cap.
    speed=min(1.,max(.2,distance/6.)); a=start if waypoint==0 else route[waypoint-1]; b=route[waypoint]; dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy); along=max(0.,min(length,(truth["position_ned_m"][0]-a[0])*dx/length+(truth["position_ned_m"][1]-a[1])*dy/length)); return [(a[0]+min(length,along+speed*DT*k)*dx/length,a[1]+min(length,along+speed*DT*k)*dy/length,desired_heading,speed) for k in range(horizon)]


def run_batch(seeds, horizon, ratio):
    prepared = [config(seed) for seed in seeds]
    controllers = [LinearPlanarMPC(horizon, ratio, control_horizon=max(1,round(horizon*.2))) for _ in seeds]
    runs = [{"seed": seed, "start": start, "route": route, "waypoint": 0, "min_final": math.inf, "first_pass": {str(radius): None for radius in RADII}, "final_start": None, "return": 0., "cross_track": 0., "solve_times": [], "fallbacks": 0, "commands": 0, "done": False, "terminal_success": False, "previous_distance": math.hypot(route[0][0]-start[0], route[0][1]-start[1])} for seed, (_, start, route) in zip(seeds, prepared)]
    with PersistentNodeBridge(ROOT) as bridge:
        bridge.reset([item[0] for item in prepared])
        for step in range(STEPS):
            truths = bridge.ground_truth_all(); actions = []
            for run, controller, truth in zip(runs, controllers, truths):
                if run["done"]:
                    actions.append(None); continue
                solved = controller.solve(truth, reference(run["start"], run["route"], run["waypoint"], truth, horizon), run.setdefault("delivered",np.zeros(2)))
                run["solve_times"].append(solved.solve_time_s); run["fallbacks"] += int(solved.fallback); run["commands"] += 1
                port, starboard = solved.command
                actions.append({"actuators": {"desiredWrench": [float(port+starboard), 0, 0, 0, 0, float(.395*(starboard-port))]}})
            result = bridge.step(actions)
            truths = bridge.ground_truth_all()
            for i, (run, truth) in enumerate(zip(runs, truths)):
                if run["done"]: continue
                distance, cross, _, passed = geom(run["start"], run["route"], run["waypoint"], truth)
                run["return"] += 2*(run["previous_distance"]-distance) - .02*abs(cross); run["cross_track"] += abs(cross); run["previous_distance"] = distance
                if run["waypoint"] < 2 and (distance <= 6 or passed):
                    run["waypoint"] += 1; run["previous_distance"] = geom(run["start"], run["route"], run["waypoint"], truth)[0]
                if run["waypoint"] == 2:
                    run["final_start"] = run["final_start"] or step + 1; run["min_final"] = min(run["min_final"], distance)
                    for radius in RADII:
                        if distance <= radius and run["first_pass"][str(radius)] is None:
                            run["first_pass"][str(radius)] = (step + 1) * DT
                if result["terminated"][i]: run["done"] = True
                thrust={x.get("id"):x.get("thrust") for x in result["infos"][i].get("vehicle_diagnostics",{}).get("effectors",[])}
                if all(isinstance(thrust.get(k),(int,float)) for k in ("port","starboard")): run["delivered"][:]=[thrust["port"],thrust["starboard"]]
            if all(run["done"] for run in runs): break
    rows = []
    for run in runs:
        p = np.asarray(run["solve_times"])
        rows.append({"seed": run["seed"], "control_horizon_steps": max(1,round(horizon*.2)), "return": run["return"] - 10, "waypoints_reached": run["waypoint"], "closest_final_m": None if math.isinf(run["min_final"]) else run["min_final"], "final_leg_activated": run["final_start"] is not None, "transit_time_s_by_radius": run["first_pass"], "pass_through": {str(radius): bool(run["waypoint"] == 2 and run["min_final"] <= radius) for radius in RADII}, "mean_cross_track_m": run["cross_track"] / STEPS, "solver_fallbacks": run["fallbacks"], "solver_commands": run["commands"], "solve_time_median_s": float(np.median(p)), "solve_time_p95_s": float(np.quantile(p, .95))})
    return rows


def median(values): return float(statistics.median(values))
def candidate_summary(rows, horizon, ratio):
    control_horizon=max(1,round(horizon*.2))
    if any(row.get("control_horizon_steps") != control_horizon for row in rows): raise RuntimeError("recorded control horizon does not match live configuration")
    return {"prediction_horizon_steps": horizon, "prediction_horizon_s": horizon*DT, "control_horizon_steps": control_horizon, "position_effort_ratio": ratio, "success_count_at_2m_pass_through": sum(r["pass_through"]["2"] for r in rows), "waypoints_reached_sum": sum(r["waypoints_reached"] for r in rows), "median_return": median([r["return"] for r in rows]), "mean_cross_track_m": sum(r["mean_cross_track_m"] for r in rows)/len(rows), "raw": rows}


def better(a, b):
    return (a["success_count_at_2m_pass_through"], a["waypoints_reached_sum"], a["median_return"], -a["mean_cross_track_m"]) > (b["success_count_at_2m_pass_through"], b["waypoints_reached_sum"], b["median_return"], -b["mean_cross_track_m"])


def bootstrap_rate(values, seed=20270901):
    rng = np.random.default_rng(seed); values = np.asarray(values, dtype=float)
    draws = rng.choice(values, size=(20000, len(values)), replace=True).mean(axis=1)
    return [float(np.quantile(draws, .025)), float(np.quantile(draws, .975))]


def main():
    progress = json.loads(CHECKPOINT.read_text()) if CHECKPOINT.exists() else {"completed": {}}
    for horizon in BASE_HORIZONS:
        for ratio in RATIOS:
            key = f"N{horizon}-ratio{ratio}"
            if key not in progress["completed"]:
                rows = run_batch(range(20000, 20020), horizon, ratio)
                progress["completed"][key] = candidate_summary(rows, horizon, ratio)
                atomic(CHECKPOINT, progress)
                print(json.dumps({"completed": key}), flush=True)
    candidates = list(progress["completed"].values())
    selected = candidates[0]
    for candidate in candidates[1:]:
        if better(candidate, selected): selected = candidate
    boundary = selected["prediction_horizon_steps"] in (min(BASE_HORIZONS), max(BASE_HORIZONS)) or selected["position_effort_ratio"] in (min(RATIOS), max(RATIOS))
    if boundary:
        report = {"schema_version": 1, "artifact_kind": "mpc-v1-evaluation", "status": "tuning-boundary-optimum-halt-before-evaluation", "selected": selected, "tuning_candidates": candidates, "boundary_optimum": True, "decision": {"contract_revision_applied": False, "human_approval_required": True}}
        atomic(RESULT, report); print(json.dumps({"status": report["status"], "selected": selected}, indent=2)); return
    evaluation_rows = run_batch(range(10000, 10200), selected["prediction_horizon_steps"], selected["position_effort_ratio"])
    old = json.loads((LOCAL / "terminal-radius-sweep-n200.json").read_text())
    old_raw = [row for part in range(4) for row in json.loads((LOCAL / f"terminal-radius-sweep-n200-part-{part}.json").read_text())["raw"]]
    cells = []
    for radius in RADII:
        mpc_values = [int(row["pass_through"][str(radius)]) for row in evaluation_rows]
        references = {}
        for policy in ("frozen-untrained-policy-v1", "LOS-PID-v2", "LOS-SPEEDCAP-v2"):
            values = [int(row["pass_through_120"][str(radius)]) for row in old_raw if row["policy"] == policy]
            references[policy] = {"rate": sum(values)/len(values), "bootstrap_95_ci": bootstrap_rate(values)}
        cells.append({"radius_m": radius, "mpc_v1": {"success_rate": sum(mpc_values)/len(mpc_values), "bootstrap_95_ci": bootstrap_rate(mpc_values), "median_return": median([r["return"] for r in evaluation_rows])}, "references": references})
    solve_times = [v for row in evaluation_rows for v in [row["solve_time_median_s"]]]
    report = {"schema_version": 1, "artifact_kind": "mpc-v1-evaluation", "status": "complete-halt-before-radius-selection", "constraints": {"new_training_performed": False, "task_contract_modified": False, "evaluation_seed_set": [10000, 10199], "held_out_tuning_seed_set": [20000, 20019]}, "formulation": {"plant": "planar3 LTV-linearized numerically at every reference-horizon step", "prediction_state": ["north_m", "east_m", "heading_rad", "surge_mps", "sway_mps", "yaw_rate_rad_s", "delivered_port_thrust_n", "delivered_starboard_thrust_n"], "actuator_lag": {"time_constant_s": .35, "treatment": "first-order state in prediction model"}, "constraints": {"commanded_port_starboard_thrust_n": [-70, 70], "hard_rate_limit": None, "reason": "planar3 specifies no rateMax"}, "control_horizon_steps": 1, "execution": "OSQP resolves each control step; only first command is applied."}, "reward_cost_mapping": {"position_error": "quadratic Q position, swept relative to effort", "velocity_error": "fixed quadratic Q on surge/sway/yaw rate", "control_effort": "quadratic R with delivered-thrust regularization rebalanced to 2e-5", "command_delta": "soft quadratic penalty mirroring task action-change penalty; no hard slew constraint"}, "tuning": {"base_grid": {"prediction_horizon_steps": BASE_HORIZONS, "position_effort_ratio": RATIOS}, "objective": ["success_count_at_2m_pass_through_max", "waypoints_reached_sum_max", "median_return_max", "mean_cross_track_min"], "selected": selected, "boundary_optimum": False, "candidates": candidates}, "evaluation": {"raw_mpc_v1": evaluation_rows, "cells": cells, "solve_time_per_control_step_s": {"median": median(solve_times), "p95": float(np.quantile(solve_times, .95))}, "solver_failure_rate": sum(r["solver_fallbacks"] for r in evaluation_rows)/sum(r["solver_commands"] for r in evaluation_rows), "transit_time_completed_episodes_s_by_radius": {str(radius): [r["transit_time_s_by_radius"][str(radius)] for r in evaluation_rows if r["pass_through"][str(radius)]] for radius in RADII}}, "consolidated_reference_table_note": "LOS-SPEEDCAP-v2 is retained and renamed; it models no actuator lag. MPC-v1 models the plant's documented actuator lag, an inherent predictive-model distinction rather than an extra constraint or configuration advantage.", "decision": {"radius_selection_applied": False, "contract_revision_applied": False, "human_approval_required": True}}
    atomic(RESULT, report); print(json.dumps({"status": report["status"], "selected": selected, "solve_time": report["evaluation"]["solve_time_per_control_step_s"]}, indent=2))

if __name__ == "__main__": main()
