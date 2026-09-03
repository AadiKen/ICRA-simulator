from __future__ import annotations
import hashlib,json,math,statistics,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO

MODEL=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/ppo-final.zip"
REPORT=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/report.json"
OUT=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-final-leg-trajectories.json"
model=PPO.load(MODEL,device="cpu")
gate=json.loads(REPORT.read_text())
expected={row["seed"]:row for row in gate["evaluation"]["raw"]}
episodes=[]
replay_summary=[]

for seed in range(10000,10050):
 env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed)
 obs,_=env.reset();trajectory=[];total=0.;final_leg_ever=False;closest=float("inf");closest_time=None
 for step in range(2400):
  action,_=model.predict(obs,deterministic=True)
  obs,reward,terminated,truncated,info=env.step(action);total+=reward
  truth=info["terminal_state"];n,e=truth["position_ned_m"][:2];fn,fe=env.route[-1];distance=math.hypot(fn-n,fe-e)
  final_leg_ever=final_leg_ever or bool(info["final_leg_active"])
  if info["final_leg_active"] and distance<closest:closest=distance;closest_time=(step+1)*.05
  trajectory.append({"step":step+1,"time_s":(step+1)*.05,"N_m":n,"E_m":e,"yaw_rad":truth["attitude_rad"][2],"u_mps":truth["velocity_body_mps"][0],"v_mps":truth["velocity_body_mps"][1],"r_rad_s":truth["angular_rate_body_rad_s"][2],"waypoint_index":info["current_waypoint_index"],"final_leg_active":bool(info["final_leg_active"]),"distance_to_final_waypoint_m":distance,"action":[float(action[0]),float(action[1])]})
  if terminated or truncated:break
 env.close()
 prior=expected[seed]
 if abs(total-prior["return"])>1e-6 or final_leg_ever!=bool(prior["final_leg_active"]):raise RuntimeError(f"Replay mismatch at seed {seed}: return {total} vs {prior['return']}, final leg {final_leg_ever} vs {prior['final_leg_active']}")
 replay_summary.append({"seed":seed,"return":total,"final_leg_active":final_leg_ever,"steps":len(trajectory)})
 if final_leg_ever:episodes.append({"seed":seed,"closest_approach_m":closest,"closest_approach_time_s":closest_time,"terminal_distance_m":trajectory[-1]["distance_to_final_waypoint_m"],"episode_length_steps":len(trajectory),"trajectory":trajectory})

distances=[episode["closest_approach_m"] for episode in episodes]
def quantile(values,q):
 a=sorted(values);p=(len(a)-1)*q;i=int(p);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(p-i)
artifact={"schema_version":1,"artifact_kind":"surveyor-p3-v4-time-aware-final-leg-trajectory-diagnostic","status":"DIAGNOSTIC_REPLAY_COMPLETE_NO_FURTHER_TUNING","task_contract_content_sha256":gate["task_definition_content_sha256"],"source":{"gate_report":"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/report.json","gate_report_sha256":hashlib.sha256(REPORT.read_bytes()).hexdigest(),"model":"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/ppo-final.zip","model_sha256":hashlib.sha256(MODEL.read_bytes()).hexdigest()},"replay_integrity":{"evaluation_seeds":[10000,10049],"return_tolerance":1e-6,"all_returns_and_final_leg_flags_match":True},"summary":{"final_leg_episodes":len(episodes),"closest_approach_m":{"min":min(distances),"q1":quantile(distances,.25),"median":statistics.median(distances),"q3":quantile(distances,.75),"max":max(distances)},"count_2_5_to_4_m":sum(2.5<=x<=4 for x in distances),"count_below_2_5_m":sum(x<2.5 for x in distances),"count_above_4_m":sum(x>4 for x in distances)},"episodes":episodes,"all_episode_replay_summary":replay_summary,"decision":{"task_changed_after_training":False,"gate_result_changed":False,"third_revision_authorized":False}}
tmp=Path(str(OUT)+".tmp");tmp.write_text(json.dumps(artifact,indent=2)+"\n");tmp.replace(OUT)
print(json.dumps({"summary":artifact["summary"],"episodes":[{k:v for k,v in episode.items() if k!="trajectory"} for episode in episodes]},indent=2))
