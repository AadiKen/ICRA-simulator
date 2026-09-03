from __future__ import annotations

import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim import CommonWaypointEnv  # noqa: E402

CHECKPOINT = ROOT / "artifacts/rl-campaign/surveyor/p3-v6-rate-shaped-local/phase1-1500000.zip"
CURVE = ROOT / "artifacts/rl-campaign/surveyor/p3-v6-rate-shaped-local/phase-1-report.json"
OUT = ROOT / "artifacts/rl-campaign/surveyor/p3-v6-failure-diagnostic.json"
SEEDS = range(30000, 30050)
DT = 0.1


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def quantile(values, p):
    return float(np.quantile(np.asarray(values, float), p)) if values else None


def summary(values):
    return {
        "n": len(values), "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "q1": quantile(values, .25), "q3": quantile(values, .75),
        "p95": quantile(values, .95), "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def classify(distances, success):
    closest = min(distances); i = distances.index(closest); after = distances[i:]
    within6 = sum(d <= 6 for d in distances) * DT
    radial = np.diff(distances); signs = np.sign(radial[np.abs(radial) > 1e-3])
    reversals = int(np.sum(signs[1:] != signs[:-1])) if len(signs) > 1 else 0
    if success: label = "success"
    elif closest <= 4 and max(after) >= closest + 2: label = "close_then_overshoot"
    elif closest <= 6 and within6 >= 5 and reversals >= 4: label = "near_goal_orbit_or_oscillation"
    elif closest > 6: label = "did_not_enter_6m"
    else: label = "approached_but_did_not_close"
    return label, within6, reversals


def relative_disturbance(env, seed):
    _, _, route, current, wind = env._randomization(seed)
    h = math.atan2(route[2][1] - route[1][1], route[2][0] - route[1][0])
    project = lambda v: (math.cos(h) * v[0] + math.sin(h) * v[1], -math.sin(h) * v[0] + math.cos(h) * v[1])
    ca, cc = project(current); wa, wc = project(wind)
    return {"current_m_s": math.hypot(*current[:2]), "current_along_m_s": ca, "current_cross_m_s": cc,
            "wind_m_s": math.hypot(*wind[:2]), "wind_along_m_s": wa, "wind_cross_m_s": wc}


def replay(model, seed):
    env = CommonWaypointEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True)
    obs, _ = env.reset(); disturbance = relative_disturbance(env, seed)
    distances=[]; speeds=[]; heading_errors=[]; actions=[]; deltas=[]; base=0.; shaped=0.
    previous=np.zeros(2); success=False
    for decision in range(env.max_control_steps):
        action, _ = model.predict(obs, deterministic=True); action=np.asarray(action, float)
        obs, reward, terminated, truncated, info = env.step(action)
        t=info["terminal_state"]; n,e=t["position_ned_m"][:2]; yaw=t["attitude_rad"][2]
        goal=env.route[-1]; heading_error=wrap(math.atan2(goal[1]-e, goal[0]-n)-yaw)
        distances.append(float(info["distance_to_final_waypoint_m"])); speeds.append(float(info["speed_mps"])); heading_errors.append(abs(heading_error)); actions.append(action); deltas.append(action-previous);previous=action
        base += float(info["reward_components"]["base_reward"]); shaped += float(reward);success=bool(info["success"])
        if terminated or truncated: break
    env.close(); ci=int(np.argmin(distances)); label,within6,reversals=classify(distances,success)
    near=[i for i,d in enumerate(distances) if d<=10]; near_actions=np.asarray([actions[i] for i in near]) if near else np.empty((0,2)); near_deltas=np.asarray([deltas[i] for i in near]) if near else np.empty((0,2))
    return {
        "seed":seed,"success":success,"classification":label,"episode_duration_s":len(distances)*DT,
        "success_time_s":len(distances)*DT if success else None,"closest_approach_m":distances[ci],
        "closest_approach_time_s":(ci+1)*DT,"remaining_time_at_closest_s":120-(ci+1)*DT,
        "speed_at_closest_m_s":speeds[ci],"absolute_heading_error_at_closest_deg":math.degrees(heading_errors[ci]),
        "median_absolute_heading_error_deg":math.degrees(statistics.median(heading_errors)),
        "p95_absolute_heading_error_deg":math.degrees(quantile(heading_errors,.95)),
        "entered_10m":bool(near),"time_within_6m_s":within6,"radial_direction_reversals":reversals,
        "action_saturation_fraction":float(np.mean(np.abs(np.asarray(actions))>=.99)),
        "near_goal_action_saturation_fraction":float(np.mean(np.abs(near_actions)>=.99)) if len(near_actions) else None,
        "near_goal_median_action_delta":float(np.median(np.abs(near_deltas))) if len(near_deltas) else None,
        "base_return":base,"shaped_return":shaped,**disturbance,
    }


def group(rows, key):
    subset=[r for r in rows if r["success"]==key]
    fields=["closest_approach_m","remaining_time_at_closest_s","speed_at_closest_m_s","absolute_heading_error_at_closest_deg","median_absolute_heading_error_deg","p95_absolute_heading_error_deg","action_saturation_fraction","current_m_s","current_along_m_s","current_cross_m_s","wind_m_s","wind_along_m_s","wind_cross_m_s","base_return"]
    return {"episodes":len(subset),**{f:summary([r[f] for r in subset]) for f in fields}}


def correlation(rows, field):
    x=np.asarray([r[field] for r in rows],float);y=np.asarray([float(r["success"]) for r in rows])
    return float(np.corrcoef(x,y)[0,1]) if np.std(x)>0 else None


def quartiles(rows, field):
    ordered=sorted(rows,key=lambda r:r[field]); groups=np.array_split(np.asarray(ordered,dtype=object),4);out=[]
    for i,g in enumerate(groups):
        vals=[r[field] for r in g];out.append({"quartile":i+1,"range":[min(vals),max(vals)],"episodes":len(g),"success_rate":sum(r["success"] for r in g)/len(g)})
    return out


def main():
    model=PPO.load(CHECKPOINT,device="cpu");rows=[replay(model,s) for s in SEEDS]
    failures=[r for r in rows if not r["success"]]; curve=json.loads(CURVE.read_text())["curve"]
    rates=[x["evaluation"]["success_rate"] for x in curve];increments=[rates[i]-rates[i-1] for i in range(1,len(rates))]
    disturbance_fields=["current_m_s","current_along_m_s","current_cross_m_s","wind_m_s","wind_along_m_s","wind_cross_m_s"]
    report={
        "schema_version":1,"artifact_kind":"surveyor-p3-v6-phase-1-success-failure-diagnostic","status":"COMPLETE_NO_TRAINING",
        "task_contract_content_sha256":CommonWaypointEnv.EXPECTED_CONTRACT_SHA256,
        "checkpoint":str(CHECKPOINT.relative_to(ROOT)),"evaluation_seeds":[30000,30049],"training_performed":False,
        "headline":{"successes":sum(r["success"] for r in rows),"failures":len(failures),"success_rate":sum(r["success"] for r in rows)/len(rows),"failure_classification_counts":dict(Counter(r["classification"] for r in failures))},
        "success_failure_comparison":{"success":group(rows,True),"failure":group(rows,False)},
        "disturbance_association":{
            "point_biserial_correlation_with_success":{f:correlation(rows,f) for f in disturbance_fields},
            "quartile_success_rates":{f:quartiles(rows,f) for f in ("current_m_s","wind_m_s","current_along_m_s","wind_along_m_s")},
            "interpretation_rule":"Large monotonic quartile changes and materially separated success/failure distributions indicate concentration; signed along components are positive toward the waypoint."
        },
        "checkpoint_trend":{"steps":[x["checkpoint_steps"] for x in curve],"success_rates":rates,"success_rate_increments":increments,"last_increment":increments[-1],"last_two_increment_mean":statistics.fmean(increments[-2:]),"interpretation":"The curve is monotonic, but the final increment is only 0.02 after a 0.24 increase. This is evidence of substantial learning followed by deceleration, not clean evidence that the 1.5M budget alone is binding."},
        "raw":rows
    }
    counts=report["headline"]["failure_classification_counts"]
    if counts.get("did_not_enter_6m",0)>=len(failures)/2: trajectory="Most failures remain non-approaches rather than terminal precision failures."
    elif counts.get("close_then_overshoot",0)+counts.get("near_goal_orbit_or_oscillation",0)>=len(failures)/2: trajectory="Most failures are near-goal overshoot/orbit cases."
    else: trajectory="Failure geometry is mixed."
    correlations=report["disturbance_association"]["point_biserial_correlation_with_success"]
    strongest=max(disturbance_fields,key=lambda f:abs(correlations[f] or 0))
    report["conclusion"]={
        "trajectory":trajectory,
        "strongest_linear_disturbance_association":{"field":strongest,"correlation":correlations[strongest]},
        "budget_assessment":"A bounded extension is not justified by the curve alone: success improved to 50%, but the final 250k interval added only one success (0.48 to 0.50), far below the 0.90 gate and sharply slower than the preceding interval.",
        "next_step":"No fifth revision or training extension is authorized. Review whether failures cluster by the reported signed disturbance components and control/heading metrics before selecting a mechanism-specific next action."
    }
    OUT.write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({"output":str(OUT),"headline":report["headline"],"strongest":report["conclusion"]["strongest_linear_disturbance_association"],"trend":report["checkpoint_trend"]},indent=2))


if __name__=="__main__":main()
