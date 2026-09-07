#!/usr/bin/env python3
"""Audit Task 5 reward through the exact vectorized training construction; no training."""
from __future__ import annotations
import json,math,statistics,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from run_task5_conditions import Task5ConditionEnv,factory  # noqa:E402
from stable_baselines3.common.vec_env import SubprocVecEnv  # noqa:E402
OUT=ROOT/'artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k/training-reward-stream-audit.json'
def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi
def los_action(truth,route,waypoint):
 p=truth['position_ned_m'];a=route[waypoint-1];b=route[waypoint];dx,dy=b[0]-a[0],b[1]-a[1];cross=(-dy*(p[0]-a[0])+dx*(p[1]-a[1]))/math.hypot(dx,dy);desired=math.atan2(dy,dx)-math.atan2(cross,4);err=wrap(desired-truth['attitude_rad'][2]);surge=max(-150,min(150,100*(1.5-truth['velocity_body_mps'][0])));yaw=max(-100,min(100,70*err-35*truth['angular_rate_body_rad_s'][2]));diff=yaw/(2*.33);half=max(-(70-abs(diff)),min(70-abs(diff),surge/2));return [(half+diff)/70,(half-diff)/70]
def matched_los():
 seeds=list(range(30000,30050));env=SubprocVecEnv([lambda s=s:Task5ConditionEnv(ROOT,fixed_reset_seed=s,final_leg_curriculum=True,condition='default-noise') for s in seeds],start_method='fork');obs=env.reset();routes=env.get_attr('route');truths=env.get_attr('last_truth');waypoints=env.get_attr('waypoint');initial={'waypoints':waypoints,'positions':[t['position_ned_m'] for t in truths],'fix_valid':[float(x[11]) for x in obs]};done=np.zeros(50,bool);totals=[{k:0. for k in ('progress','cross_track','action_delta','terminal','base_reward','potential_shaping','shaped_reward')} for _ in seeds];rows=[[] for _ in seeds];target_errors=[];progress_errors=[]
 try:
  for step in range(1200):
   actions=np.asarray([los_action(t,r,w) for t,r,w in zip(truths,routes,waypoints)],np.float32);next_obs,rewards,dones,infos=env.step(actions)
   for i in range(50):
    if done[i]:continue
    c=infos[i]['reward_components'];
    for k in totals[i]:totals[i][k]+=float(c[k])
    previous_distance=float(rows[i][-1]['distance_m']) if rows[i] else math.hypot(routes[i][2][0]-initial['positions'][i][0],routes[i][2][1]-initial['positions'][i][1]);expected=2*(previous_distance-float(infos[i]['distance_to_current_waypoint_m']));progress_errors.append(float(c['progress'])-expected)
    gps_valid=float(next_obs[i][11])==1
    if gps_valid:
     # The vector itself and info use the same active final target; target identity is checked by distance consistency.
     observed_distance=math.hypot(float(next_obs[i][7]),float(next_obs[i][8]));target_errors.append(abs(observed_distance-float(infos[i]['distance_to_current_waypoint_m'])))
    rows[i].append({'step':step+1,'reward':float(rewards[i]),'components':c,'distance_m':float(infos[i]['distance_to_current_waypoint_m']),'waypoint':int(infos[i]['current_waypoint_index']),'final_leg_active':bool(infos[i]['final_leg_active']),'fix_valid':gps_valid})
    if dones[i]:done[i]=True
   if done.all():break
   obs=next_obs;truths=env.get_attr('last_truth');waypoints=env.get_attr('waypoint')
 finally:env.close()
 med={k:statistics.median(x[k] for x in totals) for k in totals[0]};return {'episodes':50,'initial':initial,'all_first_step_final_leg_active':all(x[0]['final_leg_active'] for x in rows),'all_steps_waypoint_2':all(y['waypoint']==2 for x in rows for y in x),'advancement_events':sum(any(y['waypoint']!=2 for y in x) for x in rows),'warmup_steps_until_first_fix':[next((y['step'] for y in x if y['fix_valid']),None) for x in rows],'max_abs_progress_formula_error':max(map(abs,progress_errors)),'goal_vector_distance_vs_truth_distance':{'median_abs_error_m':statistics.median(target_errors),'max_abs_error_m':max(target_errors)},'median_episode_components':med,'success_rate':sum(x[-1]['components']['terminal']==100 for x in rows)/50,'rows':rows}
def consumed_stream():
 env=SubprocVecEnv([factory(i,'default-noise') for i in range(4)],start_method='fork');model_path=ROOT/'artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k/default-noise-ent-0p005/seed-7322/recurrent-ppo-250k.zip';from sb3_contrib import RecurrentPPO;model=RecurrentPPO.load(model_path,device='cpu');obs=env.reset();state=None;starts=np.ones(4,bool);actions0=[];vector_rewards=[];component_rewards=[]
 try:
  for _ in range(256):
   action,state=model.predict(obs,state=state,episode_start=starts,deterministic=False);obs,reward,done,infos=env.step(action);actions0.append(np.asarray(action[0],float).tolist());vector_rewards.append(float(reward[0]));component_rewards.append(float(infos[0]['reward_components']['shaped_reward']));starts=done
 finally:env.close()
 direct=Task5ConditionEnv(ROOT,fixed_reset_seed=200000,final_leg_curriculum=True,condition='default-noise');direct_rewards=[];direct_components=[]
 try:
  direct.reset()
  for a in actions0:
   _,r,t,x,info=direct.step(np.asarray(a,np.float32));direct_rewards.append(float(r));direct_components.append(info['reward_components']);
   if t or x:break
 finally:direct.close()
 n=min(len(vector_rewards),len(direct_rewards));return {'steps':n,'training_factory_seed_rank0':200000,'max_abs_vec_reward_vs_info_shaped_reward':max(abs(vector_rewards[i]-component_rewards[i]) for i in range(n)),'max_abs_vector_vs_direct_replay_reward':max(abs(vector_rewards[i]-direct_rewards[i]) for i in range(n)),'vector_reward_distribution':{'min':min(vector_rewards[:n]),'median':statistics.median(vector_rewards[:n]),'max':max(vector_rewards[:n]),'mean':statistics.fmean(vector_rewards[:n])},'direct_component_totals':{k:sum(x[k] for x in direct_components[:n]) for k in direct_components[0]}}
def main():
 report={'schema_version':1,'artifact_kind':'task-5-training-reward-stream-audit','training_performed':False,'contract_sha256':Task5ConditionEnv.EXPECTED_CONTRACT_SHA256,'matched_seed_subproc_los':matched_los(),'ppo_consumed_stream':consumed_stream()};OUT.write_text(json.dumps(report,indent=2)+'\n');a=report['matched_seed_subproc_los'];print(json.dumps({'los':{k:a[k] for k in ('success_rate','all_first_step_final_leg_active','all_steps_waypoint_2','advancement_events','max_abs_progress_formula_error','goal_vector_distance_vs_truth_distance','median_episode_components')},'ppo_stream':report['ppo_consumed_stream']},indent=2))
if __name__=='__main__':main()
