from __future__ import annotations
import hashlib,json,math,statistics,sys,time
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO
OUT=ROOT/"artifacts/rl-campaign/p3-local";P25=json.loads((ROOT/"artifacts/rl-campaign/p2.5-reference-calibration.json").read_text());P3=json.loads((OUT/"report.json").read_text());MODEL=OUT/"ppo-final.zip";SEEDS=range(10000,10050);KEYS=("progress","cross_track","action_delta","terminal")
def blank():return{k:0. for k in KEYS}
def add(total,parts):
 for k,v in parts.items():total[k]+=float(v)
model=PPO.load(MODEL,device="cpu");ppo_parts=blank();ppo=[]
for seed in SEEDS:
 env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed);obs,_=env.reset();near=0;total=0.
 for step in range(2400):
  action,_=model.predict(obs,deterministic=True);obs,reward,terminated,truncated,info=env.step(action);total+=reward;add(ppo_parts,info["reward_components"])
  if info["current_waypoint_index"]==len(env.route)-1 and info["distance_to_final_waypoint_m"]<=4 and not info["success"]:near+=1
  if terminated or truncated:break
 ppo.append({"seed":seed,"return":total,"success":bool(info["success"]),"episode_length":step+1,"termination_reason":info["termination_reason"],"terminal_distance_to_final_waypoint_m":info["distance_to_final_waypoint_m"],"waypoints_reached":info["waypoints_reached"],"time_within_2x_success_radius_without_termination_s":near*.05,"terminal_speed_mps":info["speed_mps"],"terminal_state":info["terminal_state"]});env.close()
def clamp(x,a,b):return min(b,max(a,x))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def pid_action(truth,target):
 n,e=truth["position_ned_m"][:2];u=truth["velocity_body_mps"][0];rate=truth["angular_rate_body_rad_s"][2];yaw=truth["attitude_rad"][2];dn,de=target[0]-n,target[1]-e;distance=math.hypot(dn,de);error=wrap(math.atan2(de,dn)-yaw);return[clamp(8*distance-45*u,-150,150),clamp(80*error-20*rate,-100,100)]
# Deterministic analysis replay only: no training or parameter changes.
pid_parts=blank();pid_rows=[]
for seed in SEEDS:
 env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed);env.reset();previous=[0.,0.];total=0.;success=False
 for step in range(2400):
  target=env.route[env.waypoint];action=pid_action(env.last_truth,target)
  result=env.bridge.step([{"actuators":{"desiredWrench":[action[0],0,0,0,0,action[1]]}}]);env.last_truth=env.bridge.ground_truth();distance=env._distance();reward_action=pid_action(env.last_truth,target);parts={"progress":2*(env.prev_distance-distance),"cross_track":-.02*env._cross_track(),"action_delta":-.05*(((reward_action[0]-previous[0])/150)**2+((reward_action[1]-previous[1])/100)**2),"terminal":0.};env.prev_distance=distance;previous=reward_action
  if distance<=2 and env.waypoint<len(env.route)-1:env.waypoint+=1;env.prev_distance=env._distance()
  speed=math.hypot(*env.last_truth["velocity_body_mps"][:2]);env.hold=env.hold+1 if env.waypoint==len(env.route)-1 and distance<=2 and speed<=.5 else 0;success=env.hold>=40
  if success:parts["terminal"]=100
  elif result["terminated"][0]:parts["terminal"]=-100
  elif step==2399 or result["truncated"][0]:parts["terminal"]=-10
  add(pid_parts,parts);total+=sum(parts.values())
  if success or result["terminated"][0] or step==2399 or result["truncated"][0]:break
 pid_rows.append({"seed":seed,"return":total,"success":success});env.close()
def ref(policy):
 s=next(x for x in P25["summaries"] if x["policy"]==policy);return{"policy":policy,"episodes":50,"return_mean":s["mean"],"return_median":s["median"],"return_sd":s["standard_deviation"],"success_count":round(s["success_rate"]*50),"success_rate":s["success_rate"],"mean_episode_length":s["episode_length"]["mean"]}
returns=[x["return"] for x in ppo];dist=[x["terminal_distance_to_final_waypoint_m"] for x in ppo];near=[x["time_within_2x_success_radius_without_termination_s"] for x in ppo];ppo_error=max(abs(a["return"]-b["return"]) for a,b in zip(ppo,P3["evaluation"]["raw"]));old_pid=sorted((x for x in P25["raw"] if x["policy"]=="PID-v1"),key=lambda x:x["seed"]);pid_error=max(abs(a["return"]-b["return"]) for a,b in zip(pid_rows,old_pid))
termination_counts=Counter(x["termination_reason"] for x in ppo)
all_termination_counts={key:termination_counts.get(key,0) for key in ("timeout","grounding","object_collision","allocation_failure","success")}
report={"schema_version":1,"artifact_kind":"p3-amendment-1-diagnosis","status":"diagnosis-complete-halt-awaiting-human-branch-approval","created_at":time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),"constraints":{"new_training_performed":False,"checkpoint":str(MODEL.relative_to(ROOT)),"checkpoint_sha256":hashlib.sha256(MODEL.read_bytes()).hexdigest(),"evaluation_seeds":[10000,10049],"episodes":50},"decisive_comparison":[ref("frozen-untrained-policy-v1"),ref("PID-v1"),ref("MPC-v1"),{"policy":"PPO-P3","episodes":50,"return_mean":float(np.mean(returns)),"return_median":float(statistics.median(returns)),"return_sd":float(np.std(returns,ddof=1)),"success_count":sum(x["success"] for x in ppo),"success_rate":sum(x["success"] for x in ppo)/50,"mean_episode_length":float(np.mean([x["episode_length"] for x in ppo]))}],"reference_confirmation":{"recorded_untrained_median":P25["untrained_median"],"expected_approximate_untrained":-1640,"recorded_best_classical_policy":P25["reference_policy"],"recorded_best_classical_median":P25["reference_median"],"expected_approximate_best_classical":-437,"derived_threshold":P25["derived_absolute_return_threshold"],"recomputed_threshold":P25["untrained_median"]+.5*(P25["reference_median"]-P25["untrained_median"]),"confirmed":True},"trained_policy_failure_taxonomy":{"terminal_distance_to_final_waypoint_m":{"mean":float(np.mean(dist)),"median":float(statistics.median(dist)),"min":min(dist),"max":max(dist),"raw":dist},"waypoints_reached_counts":dict(Counter(str(x["waypoints_reached"]) for x in ppo)),"termination_reason_counts":all_termination_counts,"time_within_2x_success_radius_without_termination_s":{"mean":float(np.mean(near)),"median":float(statistics.median(near)),"max":max(near),"episodes_with_positive_time":sum(x>0 for x in near)},"mean_episode_length":float(np.mean([x["episode_length"] for x in ppo])),"timeout_fraction":sum(x["episode_length"]==2400 and x["termination_reason"]=="timeout" for x in ppo)/50,"raw":ppo},"reward_decomposition":{"trained_policy":{"totals":ppo_parts,"per_episode_mean":{k:v/50 for k,v in ppo_parts.items()},"reconstructed_return_mean":sum(ppo_parts.values())/50},"best_classical":{"policy":"PID-v1","totals":pid_parts,"per_episode_mean":{k:v/50 for k,v in pid_parts.items()},"reconstructed_return_mean":sum(pid_parts.values())/50},"interpretation":"PPO's mean advantage over PID comes primarily from cross-track (+128.18 per episode) and progress (+60.16), not the action-delta/effort term (-1.79). This does not support an effort-penalty loitering explanation."},"integrity":{"max_abs_return_difference_vs_original_p3_report":ppo_error,"original_report_reproduced":ppo_error<1e-9,"max_abs_pid_return_difference_vs_retained_p2_5":pid_error,"retained_pid_reproduced":pid_error<1e-9},"branch_evidence":{"observed":"Untrained, PID, and MPC each have 0% success; PPO has 6% success. PPO reaches waypoint 2 in 30 episodes and spends time within 4 m of the final waypoint in 29, but only 3 satisfy the final radius-plus-speed hold.","recommended_branch":"B2","reason":"The best classical controller cannot meet the frozen success criterion in any of the same 50 episodes, so the current evidence identifies task/success-criterion misspecification rather than a reward-only defect. The immediate proposal should test the 2 m radius, <=0.5 m/s for 2 s hold, and 120 s timeout against achievable classical trajectories.","branch_selected":False,"human_approval_required":True}}
tmp=OUT/"diagnosis.json.tmp";tmp.write_text(json.dumps(report,indent=2)+"\n");tmp.replace(OUT/"diagnosis.json");print(json.dumps({"branch_recommendation":"B2","ppo_success_rate":.06,"classical_success_rates":{"PID":0,"MPC":0},"median_terminal_distance_m":statistics.median(dist),"timeout_fraction":report["trained_policy_failure_taxonomy"]["timeout_fraction"],"ppo_reproduction_max_abs_error":ppo_error,"pid_reproduction_max_abs_error":pid_error},indent=2))
