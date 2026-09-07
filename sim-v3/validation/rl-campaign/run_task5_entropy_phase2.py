#!/usr/bin/env python3
"""Authorized Task 5 Phase 2, topology-native action condition only."""
from __future__ import annotations
import json,statistics,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from bcod_sim import CommonWaypointEnv  # noqa:E402
from run_task5_conditions import Task5ConditionEnv,atomic_json  # noqa:E402
from stable_baselines3.common.callbacks import BaseCallback  # noqa:E402
OUT=ROOT/'artifacts/rl-campaign/surveyor/task-5-default-noise-ent-0p005-phase2/seed-7319'
SOURCE=ROOT/'artifacts/rl-campaign/surveyor/task-5-default-noise-ent-0p005-phase1-1p5m/seed-7319/phase1-1500000.zip'
CONTRACT=json.loads((ROOT/'artifacts/rl-campaign/surveyor/task-contract-frozen.json').read_text());TASK=next(x for x in CONTRACT['tasks'] if x['task_id']=='common-waypoint-transit-v1')
def factory(rank):return lambda:Task5ConditionEnv(ROOT,base_seed=200_000+rank*1_000_000,final_leg_curriculum=False,condition='default-noise')
class Checkpoints(BaseCallback):
 def __init__(self):super().__init__();self.next=250_000
 def _on_step(self):
  if self.num_timesteps>=self.next:self.model.save(OUT/f'phase2-{self.next}');atomic_json(OUT/'training-state.json',{'status':'phase-2-running','phase_2_timesteps':self.next,'actual_model_timesteps':self.num_timesteps,'phase_2_started':True});self.next+=250_000
  return True
def evaluate(model):
 rows=[]
 for seed in range(10000,10050):
  env=Task5ConditionEnv(ROOT,fixed_reset_seed=seed,final_leg_curriculum=False,condition='default-noise')
  try:
   obs,_=env.reset();state=None;start=np.ones(1,bool);shaped=0.;components={k:0. for k in ('progress','cross_track','action_delta','terminal','base_reward','potential_shaping')}
   while True:
    action,state=model.predict(obs,state=state,episode_start=start,deterministic=True);obs,reward,terminated,truncated,info=env.step(action);shaped+=float(reward)
    for k in components:components[k]+=float(info['reward_components'][k])
    start=np.asarray([terminated or truncated],bool)
    if terminated or truncated:break
   rows.append({'seed':seed,'success':bool(info['success']),'termination_reason':info['termination_reason'],'shaped_return':shaped,'base_return':components['base_reward'],'components':components,'waypoints_reached':info['waypoints_reached']})
  finally:env.close()
 return {'episodes':50,'success_rate':sum(x['success'] for x in rows)/50,'median_base_return':statistics.median(x['base_return'] for x in rows),'median_shaped_return':statistics.median(x['shaped_return'] for x in rows),'median_component_contributions':{k:statistics.median(x['components'][k] for x in rows) for k in rows[0]['components']},'rows':rows}
def main():
 from sb3_contrib import RecurrentPPO
 from stable_baselines3.common.vec_env import SubprocVecEnv
 if not SOURCE.exists():raise FileNotFoundError(SOURCE)
 if CommonWaypointEnv.EXPECTED_CONTRACT_SHA256!=CONTRACT['content_sha256']:raise RuntimeError('Contract hash mismatch before Phase 2')
 OUT.mkdir(parents=True,exist_ok=True);env=SubprocVecEnv([factory(i) for i in range(16)],start_method='fork');model=RecurrentPPO.load(SOURCE,env=env,device='cpu');started=time.time();atomic_json(OUT/'training-state.json',{'status':'phase-2-running','phase_2_timesteps':0,'phase_2_started':True,'source_checkpoint':str(SOURCE.relative_to(ROOT)),'contract_sha256':CONTRACT['content_sha256'],'action_condition':'topology-native normalized actuators'})
 try:model.learn(total_timesteps=3_000_000,reset_num_timesteps=True,callback=Checkpoints(),progress_bar=False);model.save(OUT/'recurrent-ppo-final')
 finally:env.close()
 result=evaluate(model);rr=TASK['learnability']['absolute_return_threshold'];sr=TASK['learnability']['absolute_success_rate_threshold'];normalized=(result['median_base_return']-rr['untrained_median'])/(rr['reference_median']-rr['untrained_median']);passed=result['median_base_return']>=rr['value'] and normalized>=.5 and result['success_rate']>=sr['value'];report={'schema_version':1,'artifact_kind':'task-5-default-noise-entropy-phase2-topology-native','contract_sha256':CONTRACT['content_sha256'],'action_condition':'topology-native normalized actuators','must_not_be_pooled_with':'common body wrench plus frozen allocator','training':{'phase_1_source':str(SOURCE.relative_to(ROOT)),'phase_2_steps':3_000_000,'seed':7319,'algorithm':'RecurrentPPO','policy':'MlpLstmPolicy','ent_coef':.005,'condition':'default-noise','final_leg_curriculum':False,'wall_clock_s':time.time()-started},'evaluation':result|{'normalized_base_return':normalized},'acceptance':{'median_base_return_min':rr['value'],'normalized_base_return_min':.5,'success_rate_min':sr['value'],'all_must_pass':True},'gate_passed':passed};atomic_json(OUT/'report.json',report);atomic_json(OUT/'training-state.json',{'status':'completed','phase_2_started':True,'phase_2_timesteps':3_000_000,'gate_passed':passed,'report':str((OUT/'report.json').relative_to(ROOT))});print(json.dumps({'success_rate':result['success_rate'],'median_base_return':result['median_base_return'],'median_shaped_return':result['median_shaped_return'],'normalized_base_return':normalized,'gate_passed':passed},indent=2))
if __name__=='__main__':main()
