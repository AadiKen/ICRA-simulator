from __future__ import annotations
import hashlib,json,statistics,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv
from stable_baselines3 import PPO
MODEL=ROOT/"artifacts/rl-campaign/p3-local/ppo-final.zip";OLD=json.loads((ROOT/"artifacts/rl-campaign/p3-local/report.json").read_text());ANCHOR=json.loads((ROOT/"artifacts/rl-campaign/p2.5-corrected-corridor.json").read_text());model=PPO.load(MODEL,device="cpu");rows=[]
for seed in range(10000,10050):
 env=CommonWaypointEnv(ROOT,fixed_reset_seed=seed);obs,_=env.reset();total=0.
 for step in range(2400):
  action,_=model.predict(obs,deterministic=True);obs,reward,terminated,truncated,info=env.step(action);total+=reward
  if terminated or truncated:break
 rows.append({"seed":seed,"return":total,"success":bool(info["success"]),"episode_length":step+1,"termination_reason":info["termination_reason"],"waypoints_reached":info["waypoints_reached"]});env.close()
median=statistics.median(x["return"] for x in rows);untrained=ANCHOR["return_reference"]["untrained_median"];reference=ANCHOR["return_reference"]["reference_median"];report={"schema_version":1,"artifact_kind":"p3-checkpoint-corridor-advancement-rescore","gate_result":False,"diagnostic_only":True,"new_training_performed":False,"model_sha256":hashlib.sha256(MODEL.read_bytes()).hexdigest(),"original":{"median_return":OLD["evaluation"]["median_return"],"success_rate":OLD["evaluation"]["success_rate"]},"corridor_corrected":{"median_return":median,"mean_return":float(np.mean([x["return"] for x in rows])),"success_rate":sum(x["success"] for x in rows)/50,"normalized_return_score":(median-untrained)/(reference-untrained),"return_threshold":ANCHOR["return_reference"]["absolute_threshold"],"raw":rows},"anchor":{"untrained_median":untrained,"reference_policy":ANCHOR["return_reference"]["policy"],"reference_median":reference},"warning":"This rescore is diagnostic, not Gate 3. Fresh training requires a separately approved re-hashed contract."};path=ROOT/"artifacts/rl-campaign/p3-local/corridor-advancement-rescore.json";tmp=Path(str(path)+".tmp");tmp.write_text(json.dumps(report,indent=2)+"\n");tmp.replace(path);print(json.dumps({"original":report["original"],"corridor_corrected":{k:v for k,v in report["corridor_corrected"].items() if k!="raw"}},indent=2))
