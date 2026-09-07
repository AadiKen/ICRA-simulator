"""Diagnostic-only comparison of recurrent PPO failures against the v6 baseline."""
from __future__ import annotations

import json
import statistics
from collections import Counter
from pathlib import Path

import numpy as np
from sb3_contrib import RecurrentPPO

from p3_v6_failure_diagnostic import (
    ROOT, SEEDS, DT, CommonWaypointEnv, classify, correlation, group,
    quartiles, relative_disturbance,
)

CHECKPOINT = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-recurrent-local/phase1-1500000.zip"
CURVE = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-recurrent-local/phase-1-report.json"
OUT = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-recurrent-failure-diagnostic.json"
V6 = ROOT / "artifacts/rl-campaign/surveyor/p3-v6-failure-diagnostic.json"


def replay(model, seed):
    env = CommonWaypointEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True)
    obs, _ = env.reset(); disturbance = relative_disturbance(env, seed)
    distances=[]; speeds=[]; heading_errors=[]; actions=[]; deltas=[]; base=0.; shaped=0.; previous=np.zeros(2)
    state=None; episode_start=np.ones((1,), dtype=bool); success=False
    try:
        for decision in range(env.max_control_steps):
            action, state=model.predict(obs, state=state, episode_start=episode_start, deterministic=True)
            action=np.asarray(action,float); obs,reward,terminated,truncated,info=env.step(action)
            episode_start=np.asarray([terminated or truncated],dtype=bool)
            t=info["terminal_state"]; n,e=t["position_ned_m"][:2]; yaw=t["attitude_rad"][2]; goal=env.route[-1]
            error=(np.arctan2(goal[1]-e,goal[0]-n)-yaw+np.pi)%(2*np.pi)-np.pi
            distances.append(float(info["distance_to_final_waypoint_m"])); speeds.append(float(info["speed_mps"])); heading_errors.append(abs(error)); actions.append(action); deltas.append(action-previous); previous=action
            base+=float(info["reward_components"]["base_reward"]); shaped+=float(reward); success=bool(info["success"])
            if terminated or truncated: break
    finally: env.close()
    ci=int(np.argmin(distances)); label,within6,reversals=classify(distances,success); near=[i for i,d in enumerate(distances) if d<=10]
    near_actions=np.asarray([actions[i] for i in near]) if near else np.empty((0,2)); near_deltas=np.asarray([deltas[i] for i in near]) if near else np.empty((0,2))
    return {"seed":seed,"success":success,"classification":label,"episode_duration_s":len(distances)*DT,"success_time_s":len(distances)*DT if success else None,"closest_approach_m":distances[ci],"closest_approach_time_s":(ci+1)*DT,"remaining_time_at_closest_s":120-(ci+1)*DT,"speed_at_closest_m_s":speeds[ci],"absolute_heading_error_at_closest_deg":float(np.degrees(heading_errors[ci])),"median_absolute_heading_error_deg":float(np.degrees(statistics.median(heading_errors))),"p95_absolute_heading_error_deg":float(np.degrees(np.quantile(heading_errors,.95))),"entered_10m":bool(near),"time_within_6m_s":within6,"radial_direction_reversals":reversals,"action_saturation_fraction":float(np.mean(np.abs(np.asarray(actions))>=.99)),"near_goal_action_saturation_fraction":float(np.mean(np.abs(near_actions)>=.99)) if len(near_actions) else None,"near_goal_median_action_delta":float(np.median(np.abs(near_deltas))) if len(near_deltas) else None,"base_return":base,"shaped_return":shaped,**disturbance}


def main():
    model=RecurrentPPO.load(CHECKPOINT,device="cpu"); rows=[replay(model,s) for s in SEEDS]; failures=[r for r in rows if not r["success"]]
    curve=json.loads(CURVE.read_text())["curve"]; rates=[r["evaluation"]["success_rate"] for r in curve]
    fields=["current_m_s","current_along_m_s","current_cross_m_s","wind_m_s","wind_along_m_s","wind_cross_m_s"]
    v6=json.loads(V6.read_text()); v6_failure=v6["success_failure_comparison"]["failure"]
    report={"schema_version":1,"artifact_kind":"surveyor-p3-v7-recurrent-phase-1-success-failure-diagnostic","status":"COMPLETE_NO_TRAINING","task_contract_content_sha256":CommonWaypointEnv.EXPECTED_CONTRACT_SHA256,"checkpoint":str(CHECKPOINT.relative_to(ROOT)),"evaluation_seeds":[30000,30049],"training_performed":False,"headline":{"successes":sum(r["success"] for r in rows),"failures":len(failures),"success_rate":sum(r["success"] for r in rows)/len(rows),"failure_classification_counts":dict(Counter(r["classification"] for r in failures))},"success_failure_comparison":{"success":group(rows,True),"failure":group(rows,False)},"disturbance_association":{"point_biserial_correlation_with_success":{f:correlation(rows,f) for f in fields},"quartile_success_rates":{f:quartiles(rows,f) for f in ("current_m_s","wind_m_s","current_along_m_s","wind_along_m_s")}},"checkpoint_trend":{"steps":[x["checkpoint_steps"] for x in curve],"success_rates":rates,"increments":[rates[i]-rates[i-1] for i in range(1,len(rates))]},"v6_failure_heading_reference":{"median_absolute_heading_error_deg":v6_failure["median_absolute_heading_error_deg"]["median"],"p95_absolute_heading_error_deg":v6_failure["p95_absolute_heading_error_deg"]["p95"]},"raw":rows}
    recurrence_failure=report["success_failure_comparison"]["failure"]
    report["conclusion"]={"heading_error_vs_v6":{"recurrent_failure_median_deg":recurrence_failure["median_absolute_heading_error_deg"]["median"],"v6_failure_median_deg":report["v6_failure_heading_reference"]["median_absolute_heading_error_deg"],"recurrent_failure_p95_deg":recurrence_failure["p95_absolute_heading_error_deg"]["p95"],"v6_failure_p95_deg":report["v6_failure_heading_reference"]["p95_absolute_heading_error_deg"]},"next_step":"No Phase 2, budget extension, or architecture revision is authorized. Interpret this diagnostic before selecting any new intervention."}
    OUT.write_text(json.dumps(report,indent=2)+"\n"); print(json.dumps({"output":str(OUT),"headline":report["headline"],"heading":report["conclusion"]["heading_error_vs_v6"]},indent=2))


if __name__ == "__main__": main()
