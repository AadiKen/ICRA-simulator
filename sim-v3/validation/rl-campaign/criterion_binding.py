from __future__ import annotations
import hashlib,json,math,statistics,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from bcod_sim.common_task_env import Mulberry32
from stable_baselines3 import PPO
OUT=ROOT/"artifacts/rl-campaign/p3-local";MODEL=OUT/"ppo-final.zip";model=PPO.load(MODEL,device="cpu");DT=.05;ORIGINAL_STEPS=2400;DIAGNOSTIC_STEPS=4800;SEEDS=range(10000,10050)
def clamp(x,a,b):return min(b,max(a,x))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
rng=Mulberry32(7319);rand=lambda:2*rng.next()-1;W1=[[rand() for _ in range(9)] for _ in range(16)];b1=[rand() for _ in range(16)];W2=[[rand() for _ in range(16)] for _ in range(2)];b2=[rand() for _ in range(2)]
def classical(policy,truth,target,previous):
 n,e=truth["position_ned_m"][:2];u,v=truth["velocity_body_mps"][:2];rate=truth["angular_rate_body_rad_s"][2];yaw=truth["attitude_rad"][2];dn,de=target[0]-n,target[1]-e;distance=math.hypot(dn,de);error=wrap(math.atan2(de,dn)-yaw)
 if policy=="PID-v1":return[clamp(8*distance-45*u,-150,150),clamp(80*error-20*rate,-100,100)]
 if policy=="MPC-v1":
  target_speed=min(1.5,.3*distance,distance/8*1.5 if distance<8 else 1.5);return[clamp(100*(target_speed-u),-150,150),clamp(100*error-30*rate,-100,100)]
 obs=[clamp(distance/100,0,1),math.sin(error),math.cos(error),u/3,v/3,rate,previous[0]/150,previous[1]/100,1];hidden=[math.tanh(sum(w*x for w,x in zip(row,obs))+bias) for row,bias in zip(W1,b1)];return[150*math.tanh(sum(w*x for w,x in zip(W2[0],hidden))+b2[0]),100*math.tanh(sum(w*x for w,x in zip(W2[1],hidden))+b2[1])]
def reset_extended(env,seed):
 cfg=env._config(seed);cfg["experiment"]["duration_s"]=DIAGNOSTIC_STEPS*DT;env.bridge.reset([cfg]);env.waypoint=0;env.prev_action[:]=0;env.steps=0;env.hold=0;env.cross_track_sum=0;env.last_truth=env.bridge.ground_truth();env.prev_distance=env._distance()
def run(policy,seed):
 env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed);reset_extended(env,seed);previous=[0.,0.];inside_speeds=[];inside_steps=0;both_run=0;longest_both=0;radius_run=0;speed_run=0;first_arrival=None;min_distance=float("inf");min_time=None;original_success=False;original_success_step=None;cf={"radius_removed":False,"speed_removed":False,"hold_removed":False,"timeout_removed":False,"pass_through_2m":False};last_distances=[];terminal_step=ORIGINAL_STEPS;timeout_snapshot=None
 for step in range(DIAGNOSTIC_STEPS):
  target=env.route[env.waypoint]
  if policy=="PPO-P3":
   action,_=model.predict(env._obs(),deterministic=True);payload={"actuators":{"effectors":{"port":{"command":float(action[0])},"starboard":{"command":float(action[1])}}}}
  else:
   action=classical(policy,env.last_truth,target,previous);payload={"actuators":{"desiredWrench":[action[0],0,0,0,0,action[1]]}}
  result=env.bridge.step([payload]);env.last_truth=env.bridge.ground_truth()
  if policy=="PPO-P3":env.prev_action=np.asarray(action,float).copy();previous=[float(action[0]),float(action[1])]
  else:previous=classical(policy,env.last_truth,target,previous)
  distance=env._distance();speed=math.hypot(*env.last_truth["velocity_body_mps"][:2]);t=(step+1)*DT
  if env.waypoint==len(env.route)-1:
   if step<ORIGINAL_STEPS and distance<min_distance:min_distance,min_time=distance,t
   if step<ORIGINAL_STEPS:last_distances.append(distance)
   inside=distance<=2
   if inside and first_arrival is None:first_arrival=t
   if inside and step<ORIGINAL_STEPS:inside_steps+=1;inside_speeds.append(speed)
   both=inside and speed<=.5;both_run=both_run+1 if both else 0
   if step<ORIGINAL_STEPS:
    longest_both=max(longest_both,both_run);radius_run=radius_run+1 if inside else 0;speed_run=speed_run+1 if speed<=.5 else 0
    if inside:cf["pass_through_2m"]=True
    if both:cf["hold_removed"]=True
    if radius_run>=40:cf["speed_removed"]=True
    if speed_run>=40:cf["radius_removed"]=True
   if both_run>=40:
    if step<ORIGINAL_STEPS:original_success=True;original_success_step=step+1
    else:cf["timeout_removed"]=True
  if distance<=2 and env.waypoint<len(env.route)-1:env.waypoint+=1
  if result["terminated"][0]:terminal_step=step+1;break
  if step+1==ORIGINAL_STEPS:
   terminal_step=ORIGINAL_STEPS;window=last_distances[-20:];trend=(window[-1]-window[0])/((len(window)-1)*DT) if len(window)>1 else None;final=env.route[-1];penultimate=env.route[-2];pos=env.last_truth["position_ned_m"][:2];dx,dy=final[0]-penultimate[0],final[1]-penultimate[1];projection=((pos[0]-penultimate[0])*dx+(pos[1]-penultimate[1])*dy)/(dx*dx+dy*dy);timeout_snapshot={"distance_trend_mps":trend,"still_approaching":trend is not None and trend<0,"overshot_final_plane":projection>1,"along_track_projection":projection,"waypoints_reached":env.waypoint}
  if original_success:terminal_step=step+1;break
 env.close();timeout_snapshot=timeout_snapshot or {"distance_trend_mps":None,"still_approaching":False,"overshot_final_plane":False,"along_track_projection":None,"waypoints_reached":env.waypoint}
 return{"policy":policy,"seed":seed,"minimum_final_distance_m":min_distance if math.isfinite(min_distance) else None,"minimum_distance_time_s":min_time,"entered_radius_2m":first_arrival is not None and first_arrival<=120,"time_to_first_arrival_s":first_arrival if first_arrival is not None and first_arrival<=120 else None,"dwell_inside_2m_s":inside_steps*DT,"inside_2m_speed_mps":{"count":len(inside_speeds),"min":min(inside_speeds) if inside_speeds else None,"median":statistics.median(inside_speeds) if inside_speeds else None,"max":max(inside_speeds) if inside_speeds else None,"fraction_at_or_below_0_5":sum(x<=.5 for x in inside_speeds)/len(inside_speeds) if inside_speeds else None},"longest_radius_and_speed_interval_s":longest_both*DT,"original_success":original_success,"original_success_time_s":original_success_step*DT if original_success_step else None,"counterfactual_pass":cf,"timeout_state":{k:v for k,v in timeout_snapshot.items() if k!="waypoints_reached"},"waypoints_reached_before_timeout":timeout_snapshot["waypoints_reached"]+(1 if original_success else 0),"observed_steps":terminal_step if original_success else DIAGNOSTIC_STEPS}
policies=("frozen-untrained-policy-v1","PID-v1","MPC-v1","PPO-P3");rows=[]
for policy in policies:
 for seed in SEEDS:rows.append(run(policy,seed))
def wilson(successes,n,z=1.959963984540054):
 p=successes/n;den=1+z*z/n;center=(p+z*z/(2*n))/den;half=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/den;return[center-half,center+half]
summaries=[]
for policy in policies:
 selected=[x for x in rows if x["policy"]==policy];arrivals=[x["time_to_first_arrival_s"] for x in selected if x["time_to_first_arrival_s"] is not None];counter={key:sum(x["counterfactual_pass"][key] for x in selected)/50 for key in next(iter(selected))["counterfactual_pass"]};summaries.append({"policy":policy,"original_success_rate":sum(x["original_success"] for x in selected)/50,"entered_2m_rate":sum(x["entered_radius_2m"] for x in selected)/50,"time_to_first_arrival_s":{"count":len(arrivals),"median":statistics.median(arrivals) if arrivals else None,"min":min(arrivals) if arrivals else None,"max":max(arrivals) if arrivals else None},"counterfactual_pass_rates":counter,"counterfactual_wilson_95_ci":{k:wilson(round(v*50),50) for k,v in counter.items()},"still_approaching_at_timeout_rate":sum(x["timeout_state"]["still_approaching"] for x in selected)/50,"overshot_final_plane_at_timeout_rate":sum(x["timeout_state"]["overshot_final_plane"] for x in selected)/50})
expected={"frozen-untrained-policy-v1":0.,"PID-v1":0.,"MPC-v1":0.,"PPO-P3":.06};observed={x["policy"]:x["original_success_rate"] for x in summaries}
if observed!=expected:raise RuntimeError(f"Replay integrity failure: expected original rates {expected}, observed {observed}; criterion artifact not written")
conclusion={"primary_binding_constraint":"ordered-waypoint acquisition radius","finding":"PID and MPC never activated the final leg in any of 50 original-horizon episodes: PID reached zero intermediate waypoints in 43 episodes and one in 7; MPC reached zero in 38 and one in 12. The shared 2 m radius gates intermediate waypoint advancement before the final radius/speed/hold conjunction can be evaluated.","classical_final_condition_identifiability":"not identifiable from these trajectories","warning":"The zero classical rates for final radius/speed/hold counterfactuals must not be interpreted as each final conjunct independently failing. Those counterfactuals are unreachable because their final-leg antecedent is false.","preferred_2m_pass_through_result":"PID 0/50 and MPC 0/50; outside the required 40-70% classical calibration band, so the preferred change cannot be adopted as written.","ppo_conjunct_evidence":{"entered_final_2m_radius":"26/50","original_success":"3/50","speed_removed_pass":"20/50","radius_removed_pass":"23/50","hold_removed_pass":"7/50","timeout_removed_total_pass_by_240s":"13/50 (3 original plus 10 after 120 s)","median_time_to_first_2m_arrival_s":64.825},"next_step":"Halt for human review. A recalibration proposal must distinguish the intermediate waypoint-acquisition radius from the final terminal condition and calibrate the former against classical control before deriving a new success threshold."}
report={"schema_version":1,"artifact_kind":"p3-amendment-2-criterion-binding","status":"analysis-complete-halt-before-recalibration-proposal","created_at":time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),"constraints":{"new_training_performed":False,"reward_changed":False,"checkpoint_sha256":hashlib.sha256(MODEL.read_bytes()).hexdigest(),"seeds":[10000,10049],"episodes_per_controller":50,"original_horizon_s":120,"timeout_relaxation_diagnostic_ceiling_s":240},"counterfactual_definitions":{"radius_removed":"Final leg active and speed <=0.5 m/s continuously for 2 s; no final-radius test.","speed_removed":"Inside the 2 m final radius continuously for 2 s; no speed test.","hold_removed":"At least one sample simultaneously inside 2 m and at speed <=0.5 m/s.","timeout_removed":"Original radius+speed+hold conjunction first satisfied after 120 s but by the fixed 240 s diagnostic ceiling.","pass_through_2m":"Final 2 m radius entered at any speed with no hold; this jointly removes speed and hold and directly evaluates E3."},"integrity":{"expected_original_success_rates":expected,"observed_original_success_rates":observed,"passed":True},"binding_conclusion":conclusion,"summaries":summaries,"raw":rows,"decision":{"branch_selected":False,"recalibration_applied":False,"human_review_required":True}};tmp=OUT/"criterion-binding.json.tmp";tmp.write_text(json.dumps(report,indent=2)+"\n");tmp.replace(OUT/"criterion-binding.json");print(json.dumps(summaries,indent=2))
