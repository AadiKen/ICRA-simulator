from __future__ import annotations
import hashlib,json,statistics,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
BASE=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-local";OUT=ROOT/"artifacts/rl-campaign/surveyor/p3-v4-time-aware-checkpoint-curve.json"
checkpoints=[250000,500000,750000,1000000,1250000,1500000,1750000,2000000,2250000,2500000,2750000,3000000]
def factory(seed):return lambda:CommonWaypointEnv(ROOT,fixed_reset_seed=seed)
def evaluate(path):
 model=PPO.load(path,device="cpu");rows=[]
 for first in range(10000,10050,10):
  seeds=list(range(first,first+10));env=SubprocVecEnv([factory(s) for s in seeds],start_method="fork");obs=env.reset();totals=np.zeros(10);active=np.ones(10,dtype=bool);final=np.zeros(10,dtype=bool);success=np.zeros(10,dtype=bool);length=np.zeros(10,dtype=int)
  for step in range(2400):
   actions,_=model.predict(obs,deterministic=True);obs,rewards,dones,infos=env.step(actions);totals[active]+=rewards[active];length[active]+=1
   for i in range(10):
    if active[i]:final[i]=final[i] or bool(infos[i].get("final_leg_active",False))
    if active[i] and dones[i]:success[i]=bool(infos[i].get("success",False));active[i]=False
   if not active.any():break
  for i,seed in enumerate(seeds):rows.append({"seed":seed,"return":float(totals[i]),"success":bool(success[i]),"final_leg_active":bool(final[i]),"episode_length":int(length[i])})
  env.close()
 returns=[r["return"] for r in rows];return {"median_return":float(statistics.median(returns)),"mean_return":float(statistics.mean(returns)),"success_rate":sum(r["success"] for r in rows)/50,"final_leg_activation_rate":sum(r["final_leg_active"] for r in rows)/50,"raw":rows}
curve=[]
for step in checkpoints:
 path=BASE/f"ppo-{step}.zip";started=time.time();result=evaluate(path);curve.append({"checkpoint_timesteps":step,"checkpoint_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"evaluation":result,"wall_clock_s":time.time()-started});print(json.dumps({"checkpoint":step,**{k:v for k,v in result.items() if k!="raw"}}),flush=True)
artifact={"schema_version":1,"artifact_kind":"surveyor-p3-v4-time-aware-checkpoint-learning-curve","status":"DIAGNOSTIC_ONLY_NO_FURTHER_TUNING","task_contract_content_sha256":"cc2c35cafee9eceb31cbb7e76522426cbabbcc78ef4176ba03a69bbdf420a1fb","evaluation":{"seeds":[10000,10049],"episodes":50,"deterministic":True},"curve":curve,"decision":{"training_extended":False,"observation_changed_by_authorized_revision":True,"contract_changed_by_authorized_revision":True,"third_revision_authorized":False}}
tmp=Path(str(OUT)+".tmp");tmp.write_text(json.dumps(artifact,indent=2)+"\n");tmp.replace(OUT)
