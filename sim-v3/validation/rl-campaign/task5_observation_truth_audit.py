#!/usr/bin/env python3
"""Evaluation-only audit of the 15-field Task 5 observation against truth."""
from __future__ import annotations

import json, math, statistics, sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_task5_conditions import Task5ConditionEnv  # noqa: E402

OUT=ROOT/'artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k/observation-ground-truth-audit.json'
SEEDS=(30000,30017,30029,30046)

def wrap(x): return (x+math.pi)%(2*math.pi)-math.pi
def stats(xs):
 xs=[abs(float(x)) for x in xs]
 return {'samples':len(xs),'median_abs_error':statistics.median(xs),'max_abs_error':max(xs),'p95_abs_error':float(np.quantile(xs,.95))}
def nearest(history,t): return min(history,key=lambda x:abs(x['time_s']-t))
def sign_bad(a,b,tol=.05): return abs(a)>tol and abs(b)>tol and (a>0)!=(b>0)

def los_episode(seed):
 e=Task5ConditionEnv(ROOT,fixed_reset_seed=seed,final_leg_curriculum=True,condition='default-noise')
 rows=[]; history=[]
 try:
  e.reset(); history.append(e.last_truth); a,b=e.route[1],e.route[2];dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy
  for control_step in range(e.max_control_steps):
   t=e.last_truth;p=t['position_ned_m'];yaw=t['attitude_rad'][2];u=t['velocity_body_mps'][0];r=t['angular_rate_body_rad_s'][2]
   cross=(-dy*(p[0]-a[0])+dx*(p[1]-a[1]))/math.sqrt(den);los_heading=math.atan2(dy,dx)-math.atan2(cross,4.0)
   surge=max(-150,min(150,100*(1.5-u)));yw=max(-100,min(100,70*wrap(los_heading-yaw)-35*r));cmd={'active_sensors':['imu','gps'],'actuators':{'desiredWrench':[surge,0,0,0,0,yw]}}
   result=None
   for _ in range(e.physics_steps_per_action):
    result=e.bridge.step([cmd]);history.append(e.bridge.ground_truth())
   e.last_observation=result['observations'][0];e.last_truth=history[-1]
   act=e.bridge.checkpoint()['checkpoints'][0]['payload']['actuatorState']['lastEffectorCommands'];e.prev_action=np.asarray([act['port']['thrust']/70,act['starboard']['thrust']/70])
   obs=e._obs(); sensors=e.last_observation.get('sensors',{}); imu=sensors.get('imu');gps=sensors.get('gps')
   row={'seed':seed,'control_step':control_step+1,'time_s':e.last_truth['time_s'],'observation':obs.tolist(),'current_truth':e.last_truth}
   if imu and imu.get('valid') and imu.get('payload'):
    it=nearest(history,float(imu['timestampS']));ip=imu['payload'];row['imu']={'timestamp_s':imu['timestampS'],'sample_truth':it,'payload':ip}
   if gps and gps.get('valid') and gps.get('payload'):
    gt=nearest(history,float(gps['timestampS']));gp=gps['payload'];row['gps']={'timestamp_s':gps['timestampS'],'sample_truth':gt,'payload':gp}
   rows.append(row)
   if math.hypot(b[0]-e.last_truth['position_ned_m'][0],b[1]-e.last_truth['position_ned_m'][1])<=e.final_radius_m:break
  return rows
 finally:e.close()

def paired_mask_check():
 d=Task5ConditionEnv(ROOT,fixed_reset_seed=30000,final_leg_curriculum=True,condition='default-noise');c=Task5ConditionEnv(ROOT,fixed_reset_seed=30000,final_leg_curriculum=True,condition='control')
 diffs=[]; yaw_default=[]; yaw_control=[]; wrap_errors=[]; wrap_naive=[]; wrap_payload_errors=[]
 try:
  od,_=d.reset();oc,_=c.reset()
  for i in range(100):
   action=np.asarray([.35+.1*math.sin(i/13),.30-.1*math.sin(i/17)],np.float32)
   od,_,td,xd,_=d.step(action);oc,_,tc,xc,_=c.step(action)
   diffs.extend(np.abs(np.delete(od,6)-np.delete(oc,6)).tolist());yaw_default.append(float(od[6]));yaw_control.append(float(oc[6]))
   if abs(d.last_truth['attitude_rad'][2])>2.8:
    wrap_errors.append(wrap(float(od[6])-d.last_truth['attitude_rad'][2]))
    wrap_naive.append(float(od[6])-d.last_truth['attitude_rad'][2])
    payload=d.last_observation['sensors']['imu']['payload']['orientation_rad'][2]
    wrap_payload_errors.append(wrap(float(od[6])-float(payload)))
   if td or xd or tc or xc:break
  return {'steps':len(yaw_control),'max_abs_difference_excluding_yaw':max(diffs),'control_yaw_unique':sorted(set(yaw_control)),'default_yaw_range':[min(yaw_default),max(yaw_default)],
          'wraparound_abs_truth_yaw_gt_2p8':{'samples':len(wrap_errors),'circular_error_vs_current_truth_including_latency':stats(wrap_errors),'observation_vs_imu_payload_circular_error':stats(wrap_payload_errors),'naive_difference_max_abs':max(map(abs,wrap_naive)) if wrap_naive else None}}
 finally:d.close();c.close()

def main():
 rows=[r for seed in SEEDS for r in los_episode(seed)];errors={k:[] for k in ('yaw_current','yaw_sample','goal_n_current','goal_e_current','goal_n_sample','goal_e_sample','accel_sample','rate_sample','bearing')};signs={k:0 for k in ('goal_n','goal_e')};wrap_samples=0;valid={'imu':0,'gps':0}
 for r in rows:
  o=r['observation'];ct=r['current_truth'];target=next(Task5ConditionEnv(ROOT,fixed_reset_seed=r['seed'],final_leg_curriculum=True,condition='default-noise')._randomization(r['seed'])[2].__iter__(),None) if False else None
  # The final target is recoverable from the observed GPS vector plus its payload position.
  if 'imu' in r:
   valid['imu']+=1;it=r['imu']['sample_truth'];ip=r['imu']['payload'];errors['yaw_current'].append(wrap(o[6]-ct['attitude_rad'][2]));errors['yaw_sample'].append(wrap(o[6]-it['attitude_rad'][2]));wrap_samples+=int(abs(it['attitude_rad'][2])>2.8)
   errors['accel_sample'].extend(np.asarray(o[0:3])-np.asarray(it['acceleration_body_mps2']));errors['rate_sample'].extend(np.asarray(o[3:6])-np.asarray(it['angular_rate_body_rad_s']))
  if 'gps' in r:
   valid['gps']+=1;gt=r['gps']['sample_truth'];gp=r['gps']['payload'];target_n=gp['position_ned_m'][0]+o[7];target_e=gp['position_ned_m'][1]+o[8]
   cur=[target_n-ct['position_ned_m'][0],target_e-ct['position_ned_m'][1]];sam=[target_n-gt['position_ned_m'][0],target_e-gt['position_ned_m'][1]]
   errors['goal_n_current'].append(o[7]-cur[0]);errors['goal_e_current'].append(o[8]-cur[1]);errors['goal_n_sample'].append(o[7]-sam[0]);errors['goal_e_sample'].append(o[8]-sam[1])
   signs['goal_n']+=int(sign_bad(o[7],cur[0]));signs['goal_e']+=int(sign_bad(o[8],cur[1]))
   sensor_bearing=wrap(math.atan2(o[8],o[7])-o[6]);truth_bearing=wrap(math.atan2(cur[1],cur[0])-ct['attitude_rad'][2]);errors['bearing'].append(wrap(sensor_bearing-truth_bearing))
 report={'schema_version':1,'artifact_kind':'task-5-observation-ground-truth-trajectory-audit','training_performed':False,'contract_sha256':Task5ConditionEnv.EXPECTED_CONTRACT_SHA256,'driver':'LOS-PID-v2 desired-wrench controller','seeds':list(SEEDS),'control_steps_logged':len(rows),'valid_samples':valid,'wraparound_samples_abs_yaw_gt_2p8':wrap_samples,'absolute_error':{k:stats(v) for k,v in errors.items()},'current_truth_sign_disagreements':signs,'paired_control_default_check':paired_mask_check(),'bearing_definition':'wrap(atan2(goal_east, goal_north) - yaw); compared with the same direct-to-goal bearing from current privileged truth. LOS-PID steering itself uses a cross-track lookahead heading and is intentionally not identical to direct bearing.','interpretation_notes':['Sample-time comparisons isolate sensor noise/bias from configured sensor latency and sample-and-hold.','Current-time comparisons include the intended latency and low-rate GPS hold.'],'rows':rows}
 OUT.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:report[k] for k in ('control_steps_logged','valid_samples','wraparound_samples_abs_yaw_gt_2p8','absolute_error','current_truth_sign_disagreements','paired_control_default_check')},indent=2))
if __name__=='__main__':main()
