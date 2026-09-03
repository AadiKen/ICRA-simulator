from __future__ import annotations
import hashlib,json,math,statistics,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO
SOURCE=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-final-leg-trajectories.json"
GATE=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/report.json"
MODEL=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/ppo-final.zip"
OUT=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-reward-transition-diagnostic.json"
source=json.loads(SOURCE.read_text());gate=json.loads(GATE.read_text());expected={r["seed"]:r for r in gate["evaluation"]["raw"]};seeds=[e["seed"] for e in source["episodes"]];model=PPO.load(MODEL,device="cpu")
episodes=[]
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
for seed in seeds:
 env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed);obs,_=env.reset();steps=[];total=0.;transition_step=None
 for step in range(2400):
  before_waypoint=env.waypoint;action,_=model.predict(obs,deterministic=True);next_obs,reward,terminated,truncated,info=env.step(action);total+=reward
  truth=info["terminal_state"];n,e=truth["position_ned_m"][:2];yaw=truth["attitude_rad"][2];u,v=truth["velocity_body_mps"][:2];vn=math.cos(yaw)*u-math.sin(yaw)*v;ve=math.sin(yaw)*u+math.cos(yaw)*v;target=env.route[env.waypoint];bearing=math.atan2(target[1]-e,target[0]-n);course=math.atan2(ve,vn) if math.hypot(vn,ve)>1e-9 else yaw
  gps_goal_error=math.hypot(float(next_obs[6])-(target[0]-n),float(next_obs[7])-(target[1]-e));advanced=env.waypoint!=before_waypoint
  if advanced and env.waypoint==2:transition_step=step+1
  steps.append({"step":step+1,"time_s":(step+1)*.05,"leg_index_after_step":env.waypoint,"advanced_this_step":advanced,"reward":float(reward),"reward_components":info["reward_components"],"position_m":[n,e],"target_m":target,"distance_to_active_goal_m":info["distance_to_current_waypoint_m"],"distance_to_final_m":info["distance_to_final_waypoint_m"],"yaw_rad":yaw,"bearing_to_active_goal_rad":bearing,"heading_error_to_active_goal_rad":wrap(bearing-yaw),"course_error_to_active_goal_rad":wrap(bearing-course),"speed_mps":info["speed_mps"],"action":[float(action[0]),float(action[1])],"observation_relative_goal_m":[float(next_obs[6]),float(next_obs[7])],"observation_goal_vector_error_m":gps_goal_error});obs=next_obs
  if terminated or truncated:break
 env.close()
 if abs(total-expected[seed]["return"])>1e-6:raise RuntimeError(f"Return mismatch at {seed}: {total} vs {expected[seed]['return']}")
 by_leg=[]
 for leg in range(3):
  rows=[r for r in steps if r["leg_index_after_step"]==leg];by_leg.append({"leg_index":leg,"steps":len(rows),"seconds":len(rows)*.05,"total_reward":sum(r["reward"] for r in rows),"progress_reward":sum(r["reward_components"]["progress"] for r in rows),"cross_track_penalty":sum(r["reward_components"]["cross_track"] for r in rows),"action_delta_penalty":sum(r["reward_components"]["action_delta"] for r in rows),"terminal_reward":sum(r["reward_components"]["terminal"] for r in rows)})
 closest=next(x["closest_approach_m"] for x in source["episodes"] if x["seed"]==seed);episodes.append({"seed":seed,"return":total,"closest_approach_m":closest,"transition_to_final_step":transition_step,"transition_to_final_time_s":transition_step*.05 if transition_step else None,"reward_by_leg":by_leg,"steps":steps})

episodes.sort(key=lambda x:x["closest_approach_m"])
def corr(xs,ys):
 mx,my=statistics.mean(xs),statistics.mean(ys);num=sum((x-mx)*(y-my) for x,y in zip(xs,ys));den=math.sqrt(sum((x-mx)**2 for x in xs)*sum((y-my)**2 for y in ys));return num/den if den else None
closest=[e["closest_approach_m"] for e in episodes];returns=[e["return"] for e in episodes];final_rewards=[e["reward_by_leg"][2]["total_reward"] for e in episodes];early_rewards=[e["reward_by_leg"][0]["total_reward"]+e["reward_by_leg"][1]["total_reward"] for e in episodes]
def transition_summary(ep,seconds=10):
 start=ep["transition_to_final_step"];rows=[r for r in ep["steps"] if start is not None and start<=r["step"]<start+seconds/.05];sign_changes=lambda vals:sum(a*b<0 for a,b in zip(vals,vals[1:]));abs_heading=[abs(r["heading_error_to_active_goal_rad"]) for r in rows];abs_course=[abs(r["course_error_to_active_goal_rad"]) for r in rows]
 return {"seed":ep["seed"],"window_s":seconds,"samples":len(rows),"goal_observation_max_error_m":max((r["observation_goal_vector_error_m"] for r in rows),default=None),"distance_m":{"start":rows[0]["distance_to_final_m"] if rows else None,"end":rows[-1]["distance_to_final_m"] if rows else None,"minimum":min((r["distance_to_final_m"] for r in rows),default=None)},"absolute_heading_error_rad":{"start":abs_heading[0] if rows else None,"median":statistics.median(abs_heading) if rows else None,"end":abs_heading[-1] if rows else None},"absolute_course_error_rad":{"median":statistics.median(abs_course) if rows else None},"action":{"mean_port":statistics.mean(r["action"][0] for r in rows) if rows else None,"mean_starboard":statistics.mean(r["action"][1] for r in rows) if rows else None,"port_sign_changes":sign_changes([r["action"][0] for r in rows]),"starboard_sign_changes":sign_changes([r["action"][1] for r in rows]),"mean_differential":statistics.mean(r["action"][1]-r["action"][0] for r in rows) if rows else None},"first_20_samples":rows[:20]}
worst=[transition_summary(e) for e in sorted(episodes,key=lambda x:x["closest_approach_m"],reverse=True)[:2]]
table=[]
for e in episodes:
 early=e["reward_by_leg"][0]["total_reward"]+e["reward_by_leg"][1]["total_reward"];final=e["reward_by_leg"][2]["total_reward"];table.append({"seed":e["seed"],"closest_approach_m":e["closest_approach_m"],"return":e["return"],"legs_1_2_reward":early,"leg_3_reward":final,"leg_3_fraction_of_absolute_component_sum":abs(final)/(abs(early)+abs(final)) if abs(early)+abs(final) else None,"leg_3_seconds":e["reward_by_leg"][2]["seconds"]})
artifact={"schema_version":1,"artifact_kind":"surveyor-p3-v4-time-aware-reward-transition-diagnostic","status":"DIAGNOSTIC_COMPLETE_NO_FURTHER_TUNING","task_contract_content_sha256":gate["task_definition_content_sha256"],"source":{"trajectory_artifact_sha256":hashlib.sha256(SOURCE.read_bytes()).hexdigest(),"gate_report_sha256":hashlib.sha256(GATE.read_bytes()).hexdigest(),"model_sha256":hashlib.sha256(MODEL.read_bytes()).hexdigest()},"reward_analysis":{"episodes_sorted_by_closest_approach":table,"pearson_return_vs_closest_approach_m":corr(returns,closest),"pearson_return_vs_negative_closest_approach":corr(returns,[-x for x in closest]),"pearson_return_vs_leg_3_reward":corr(returns,final_rewards),"pearson_return_vs_legs_1_2_reward":corr(returns,early_rewards),"mean_legs_1_2_reward":statistics.mean(early_rewards),"mean_leg_3_reward":statistics.mean(final_rewards)},"transition_analysis":{"window_definition":"First 10 seconds beginning with the step that advances from leg 2 to leg 3.","worst_cases":worst},"episodes":episodes,"decision":{"task_changed_after_training":False,"gate_result_changed":False,"third_revision_authorized":False}}
tmp=Path(str(OUT)+".tmp");tmp.write_text(json.dumps(artifact,indent=2)+"\n");tmp.replace(OUT)
print(json.dumps({"reward_analysis":artifact["reward_analysis"],"transition_analysis":artifact["transition_analysis"]},indent=2))
