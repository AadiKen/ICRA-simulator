#!/usr/bin/env python3
"""Evaluation-only audit of action signs, waypoint/reward alignment, and learned actions."""
from __future__ import annotations
import json,math,statistics,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_task5_conditions import Task5ConditionEnv  # noqa:E402
OUT=ROOT/'artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k/action-reward-policy-audit.json'
def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi
def corr(a,b):return float(np.corrcoef(a,b)[0,1]) if len(a)>1 and np.std(a)>0 and np.std(b)>0 else None
def los_action(e):
 t=e.last_truth;p=t['position_ned_m'];a=e.route[e.waypoint-1] if e.waypoint else e.start;b=e.route[e.waypoint];dx,dy=b[0]-a[0],b[1]-a[1];cross=(-dy*(p[0]-a[0])+dx*(p[1]-a[1]))/math.hypot(dx,dy);desired=math.atan2(dy,dx)-math.atan2(cross,4);err=wrap(desired-t['attitude_rad'][2]);surge=max(-150,min(150,100*(1.5-t['velocity_body_mps'][0])));yaw=max(-100,min(100,70*err-35*t['angular_rate_body_rad_s'][2]));diff=yaw/(2*.33);half=max(-(70-abs(diff)),min(70-abs(diff),surge/2));return np.asarray([(half+diff)/70,(half-diff)/70],np.float32)
def waypoint_audit(seed):
 e=Task5ConditionEnv(ROOT,fixed_reset_seed=seed,final_leg_curriculum=False,condition='default-noise');rows=[]
 try:
  obs,_=e.reset()
  for step in range(e.max_control_steps):
   pre_wp=e.waypoint;pre_target=e.route[pre_wp];pre_distance=e.prev_distance;action=los_action(e);next_obs,reward,term,trunc,info=e.step(action);post_position=e.last_truth['position_ned_m'];distance_to_pre=math.hypot(pre_target[0]-post_position[0],pre_target[1]-post_position[1]);expected_progress=2*(pre_distance-distance_to_pre);gps=e.last_observation.get('sensors',{}).get('gps');observed_target=None
   if gps and gps.get('valid') and gps.get('payload'):
    gp=gps['payload']['position_ned_m'];observed_target=[float(gp[0]+next_obs[7]),float(gp[1]+next_obs[8])]
   rows.append({'step':step+1,'pre_waypoint':pre_wp,'post_waypoint':e.waypoint,'advanced':e.waypoint!=pre_wp,'pre_action_target':list(map(float,pre_target)),'returned_observation_target':observed_target,'expected_returned_target':list(map(float,e.route[e.waypoint])),'reward_progress':float(info['reward_components']['progress']),'expected_progress_to_pre_action_target':float(expected_progress)})
   obs=next_obs
   if term or trunc:break
  return rows
 finally:e.close()
def policy_audit(training_seed=7322):
 from sb3_contrib import RecurrentPPO
 model=RecurrentPPO.load(ROOT/f'artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k/default-noise-ent-0p005/seed-{training_seed}/recurrent-ppo-250k.zip',device='cpu');rows=[]
 for seed in range(30000,30010):
  e=Task5ConditionEnv(ROOT,fixed_reset_seed=seed,final_leg_curriculum=True,condition='default-noise')
  try:
   obs,_=e.reset();state=None;start=np.ones(1,bool)
   while True:
    bearing=wrap(math.atan2(float(obs[8]),float(obs[7]))-float(obs[6]));yaw0=e.last_truth['attitude_rad'][2];action,state=model.predict(obs,state=state,episode_start=start,deterministic=True);next_obs,_,term,trunc,info=e.step(action);dyaw=wrap(e.last_truth['attitude_rad'][2]-yaw0);a=np.asarray(action,float).reshape(-1);yaw_command=float(a[0]-a[1]);rows.append({'seed':seed,'step':info['control_steps'],'observed_bearing_error_rad':bearing,'port_action':float(a[0]),'starboard_action':float(a[1]),'yaw_command_proxy_port_minus_starboard':yaw_command,'resulting_heading_change_rad':dyaw,'distance_m':info['distance_to_final_waypoint_m']});obs=next_obs;start=np.asarray([term or trunc],bool)
    if term or trunc:break
  finally:e.close()
 return rows
def main():
 waypoint=[r for s in (30000,30017,30029) for r in waypoint_audit(s)];policy=policy_audit();advanced=[r for r in waypoint if r['advanced']];target_errors=[math.dist(r['returned_observation_target'],r['expected_returned_target']) for r in waypoint if r['returned_observation_target']];progress_errors=[r['reward_progress']-r['expected_progress_to_pre_action_target'] for r in waypoint];bearing=[r['observed_bearing_error_rad'] for r in policy];command=[r['yaw_command_proxy_port_minus_starboard'] for r in policy];turn=[r['resulting_heading_change_rad'] for r in policy];nz=[i for i,x in enumerate(bearing) if abs(x)>.05 and abs(command[i])>.01]
 report={'schema_version':1,'artifact_kind':'task-5-action-reward-policy-audit','training_performed':False,'action_mapping_reference':'[+1,-1] produces positive yaw; yaw command proxy is port-starboard','reward_observation_waypoint_alignment':{'episodes':3,'steps':len(waypoint),'advancement_steps':len(advanced),'max_abs_progress_component_error':max(map(abs,progress_errors)),'max_returned_observation_target_error_m':max(target_errors),'advancement_rows':advanced},'trained_policy':{'checkpoint_training_seed':7322,'episodes':10,'steps':len(policy),'correlation_observed_bearing_vs_commanded_yaw':corr(bearing,command),'correlation_commanded_yaw_vs_resulting_heading_change':corr(command,turn),'sign_agreement_bearing_and_command_fraction':sum((bearing[i]>0)==(command[i]>0) for i in nz)/len(nz),'samples_used_for_sign':len(nz),'median_abs_observed_bearing_rad':statistics.median(map(abs,bearing)),'median_abs_yaw_command_proxy':statistics.median(map(abs,command)),'rows':policy}}
 OUT.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('trained_policy',)},indent=2));print(json.dumps({k:v for k,v in report['trained_policy'].items() if k!='rows'},indent=2))
if __name__=='__main__':main()
