from __future__ import annotations
import hashlib,json,statistics,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv
PROTOCOL_PATH=ROOT/"artifacts/rl-campaign/surveyor/p3-v3-training-protocol.json";CONTRACT_PATH=ROOT/"artifacts/rl-campaign/surveyor/task-contract-frozen.json";APPROVAL_PATH=ROOT/"artifacts/rl-campaign/surveyor/contract-approval.json";PROTOCOL=json.loads(PROTOCOL_PATH.read_text());TASK=json.loads(CONTRACT_PATH.read_text());APPROVAL=json.loads(APPROVAL_PATH.read_text());assert TASK["content_sha256"]==PROTOCOL["task_contract_content_sha256"]==APPROVAL["content_sha256"];assert APPROVAL["status"]=="FROZEN_APPROVED";COMMON=next(task for task in TASK["tasks"] if task["task_id"]=="common-waypoint-transit-v1");OUT=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local";OUT.mkdir(parents=True,exist_ok=True)
def atomic(path,value):tmp=Path(str(path)+".tmp");tmp.write_text(json.dumps(value,indent=2)+"\n");tmp.replace(path)
class Progress(BaseCallback):
 def __init__(self):super().__init__();self.started=time.time();self.next_checkpoint=250_000
 def _on_step(self):
  if self.num_timesteps>=self.next_checkpoint:
   path=OUT/f"ppo-{self.num_timesteps}";self.model.save(path);atomic(OUT/"training-state.json",{"status":"running","timesteps":self.num_timesteps,"checkpoint":str(path)+".zip","wall_clock_s":time.time()-self.started,"updated_at":time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())});self.next_checkpoint+=250_000
  return True
def factory(rank):return lambda:CommonWaypointEnv(ROOT,base_seed=200_000+rank*1_000_000)
def linear_learning_rate(progress_remaining):return 3e-4*progress_remaining
atomic(OUT/"training-state.json",{"status":"starting","timesteps":0,"task_contract_content_sha256":TASK["content_sha256"],"protocol_sha256":hashlib.sha256(PROTOCOL_PATH.read_bytes()).hexdigest(),"stability_fix":"linear learning-rate decay 3e-4 to 0"});env=SubprocVecEnv([factory(i) for i in range(16)],start_method="fork");model=PPO("MlpPolicy",env,seed=7319,n_steps=512,batch_size=512,n_epochs=10,learning_rate=linear_learning_rate,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=0,policy_kwargs={"net_arch":[128,128]},verbose=1,device="cpu");started=time.time();model.learn(total_timesteps=3_000_000,callback=Progress(),progress_bar=False);model.save(OUT/"ppo-final");env.close();returns=[];successes=[];raw=[]
for seed in range(10000,10050):
 env1=CommonWaypointEnv(ROOT,fixed_reset_seed=seed);obs,_=env1.reset();total=0
 for step in range(2400):
  action,_=model.predict(obs,deterministic=True);obs,reward,terminated,truncated,info=env1.step(action);total+=reward
  if terminated or truncated:break
 raw.append({"seed":seed,"return":total,"success":bool(info.get("success",False)),"final_leg_active":bool(info.get("final_leg_active",False)),"episode_length":step+1,"termination_reason":info.get("termination_reason")});returns.append(total);successes.append(bool(info.get("success",False)));env1.close()
return_rule=COMMON["learnability"]["absolute_return_threshold"];success_rule=COMMON["learnability"]["absolute_success_rate_threshold"];median=float(statistics.median(returns));untrained=return_rule["untrained_median"];reference=return_rule["reference_median"];normalized=(median-untrained)/(reference-untrained);success_rate=sum(successes)/len(successes);final_leg_activation_rate=sum(bool(row["final_leg_active"]) for row in raw)/len(raw);passed=median>=return_rule["value"] and normalized>=.5 and success_rate>=success_rule["value"];report={"schema_version":1,"artifact_kind":"surveyor-p3-v4-time-aware-local-learnability","host_class":"local","paper_measurement_eligible":False,"task_definition_content_sha256":TASK["content_sha256"],"protocol_sha256":hashlib.sha256(PROTOCOL_PATH.read_bytes()).hexdigest(),"training":{"timesteps":3_000_000,"seed":7319,"wall_clock_s":time.time()-started,"model":"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/ppo-final.zip","stability_fix":"linear learning-rate decay from 3e-4 to 0"},"evaluation":{"median_return":median,"mean_return":float(np.mean(returns)),"standard_deviation":float(np.std(returns,ddof=1)),"min":min(returns),"max":max(returns),"normalized_score":normalized,"success_rate":success_rate,"final_leg_activation_rate":final_leg_activation_rate,"raw":raw},"acceptance":{"absolute_return_threshold":return_rule["value"],"normalized_return_threshold":.5,"absolute_success_rate_threshold":success_rule["value"],"terminal_definition":success_rule["terminal_definition"],"both_must_pass":True},"gate_passed":passed,"immutability":"Outcome does not authorize further tuning or a third task revision."};atomic(OUT/"report.json",report);atomic(OUT/"training-state.json",{"status":"completed","timesteps":3_000_000,"gate_passed":passed,"report":"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local/report.json"});print(json.dumps({"median_return":median,"normalized_score":normalized,"success_rate":success_rate,"final_leg_activation_rate":final_leg_activation_rate,"gate_passed":passed},indent=2));raise SystemExit(0 if passed else 2)
