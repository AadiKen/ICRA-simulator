from __future__ import annotations
import hashlib,json,statistics,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv

CONTRACT_PATH=ROOT/"artifacts/rl-campaign/surveyor/task-contract-frozen.json"
PROTOCOL_PATH=ROOT/"artifacts/rl-campaign/surveyor/p3-v5-curriculum-training-protocol.json"
OUT=ROOT/"artifacts/rl-campaign/surveyor/p3-v6-rate-shaped-local"
CONTRACT=json.loads(CONTRACT_PATH.read_text())
PROTOCOL=json.loads(PROTOCOL_PATH.read_text())
assert CONTRACT["content_sha256"]==PROTOCOL["task_contract_content_sha256"]
if not PROTOCOL["execution_authorized"]:raise RuntimeError("Long run disabled: set execution_authorized only when explicitly starting P3-v5.")
COMMON=next(x for x in CONTRACT["tasks"] if x["task_id"]=="common-waypoint-transit-v1")
OUT.mkdir(parents=True,exist_ok=True)

def atomic(path,value):
 tmp=Path(str(path)+".tmp");tmp.write_text(json.dumps(value,indent=2)+"\n");tmp.replace(path)
def factory(rank,curriculum):return lambda:CommonWaypointEnv(ROOT,base_seed=200_000+rank*1_000_000,final_leg_curriculum=curriculum)
def evaluate(model,curriculum,first,count):
 rows=[]
 for seed in range(first,first+count):
  env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed,final_leg_curriculum=curriculum);obs,_=env.reset();base_total=0.;shaped_total=0.;final_leg=False
  for step in range(2400):
   action,_=model.predict(obs,deterministic=True);obs,reward,terminated,truncated,info=env.step(action);shaped_total+=float(reward);base_total+=float(info["reward_components"]["base_reward"]);final_leg=final_leg or bool(info["final_leg_active"])
   if terminated or truncated:break
  rows.append({"seed":seed,"base_return":base_total,"shaped_return":shaped_total,"success":bool(info["success"]),"final_leg_active":final_leg,"episode_length":step+1});env.close()
 return {"success_rate":sum(x["success"] for x in rows)/count,"median_base_return":float(statistics.median(x["base_return"] for x in rows)),"median_shaped_return":float(statistics.median(x["shaped_return"] for x in rows)),"raw":rows}
def linear_learning_rate(progress_remaining):return 3e-4*progress_remaining
class Phase2Progress(BaseCallback):
 def __init__(self,phase1_steps):super().__init__();self.phase1_steps=phase1_steps;self.next_checkpoint=250_000
 def _on_step(self):
  if self.num_timesteps>=self.next_checkpoint:
   self.model.save(OUT/f"phase2-{self.next_checkpoint}");atomic(OUT/"training-state.json",{"status":"phase-2-running","timesteps":self.next_checkpoint,"phase_1_steps":self.phase1_steps});self.next_checkpoint+=250_000
  return True

common_kwargs=dict(seed=7319,n_steps=512,batch_size=512,n_epochs=10,learning_rate=3e-4,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=0,policy_kwargs={"net_arch":[128,128]},verbose=1,device="cpu")
phase1_env=SubprocVecEnv([factory(i,True) for i in range(16)],start_method="fork")
model=PPO("MlpPolicy",phase1_env,**common_kwargs);phase1=[];consecutive=0;started=time.time()
atomic(OUT/"training-state.json",{"status":"phase-1-running","timesteps":0,"contract":CONTRACT["content_sha256"]})
for steps in range(250_000,1_500_001,250_000):
 model.learn(total_timesteps=250_000,reset_num_timesteps=False,progress_bar=False);model.save(OUT/f"phase1-{steps}");result=evaluate(model,True,30000,50);phase1.append({"checkpoint_steps":steps,"evaluation":result});consecutive=consecutive+1 if result["success_rate"]>=.9 else 0;atomic(OUT/"training-state.json",{"status":"phase-1-running","timesteps":steps,"success_rate":result["success_rate"],"consecutive_passing_checkpoints":consecutive})
 if consecutive>=2:break
phase1_env.close();atomic(OUT/"phase-1-report.json",{"schema_version":1,"converged":consecutive>=2,"curve":phase1})
if consecutive<2:
 atomic(OUT/"training-state.json",{"status":"phase-1-failed","timesteps":phase1[-1]["checkpoint_steps"],"full_episode_training_started":False});raise SystemExit(2)

phase2_env=SubprocVecEnv([factory(i,False) for i in range(16)],start_method="fork");model.set_env(phase2_env);model.lr_schedule=linear_learning_rate
model.learn(total_timesteps=3_000_000,reset_num_timesteps=True,callback=Phase2Progress(phase1[-1]["checkpoint_steps"]),progress_bar=False)
model.save(OUT/"ppo-final");phase2_env.close();gate=evaluate(model,False,10000,50)
return_rule=COMMON["learnability"]["absolute_return_threshold"];success_rule=COMMON["learnability"]["absolute_success_rate_threshold"];median=gate["median_base_return"];normalized=(median-return_rule["untrained_median"])/(return_rule["reference_median"]-return_rule["untrained_median"]);passed=median>=return_rule["value"] and normalized>=.5 and gate["success_rate"]>=success_rule["value"]
report={"schema_version":1,"artifact_kind":"surveyor-p3-v6-rate-shaped-curriculum-gate-3","task_contract_content_sha256":CONTRACT["content_sha256"],"training":{"phase_1_steps":phase1[-1]["checkpoint_steps"],"phase_2_steps":3_000_000,"seed":7319,"wall_clock_s":time.time()-started},"evaluation":{**gate,"normalized_base_return":normalized},"acceptance":{"median_base_return_min":return_rule["value"],"normalized_base_return_min":.5,"success_rate_min":success_rule["value"],"both_must_pass":True},"gate_passed":passed,"no_fifth_revision_without_diagnosis":True};atomic(OUT/"report.json",report);atomic(OUT/"training-state.json",{"status":"completed","gate_passed":passed,"report":"artifacts/rl-campaign/surveyor/p3-v6-rate-shaped-local/report.json"});raise SystemExit(0 if passed else 2)
