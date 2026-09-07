"""Bounded diagnostic-only recurrent PPO run with privileged yaw observation."""
from __future__ import annotations

import json
from pathlib import Path
import statistics
import sys

import gymnasium as gym
import numpy as np
from sb3_contrib import RecurrentPPO
from stable_baselines3.common.vec_env import SubprocVecEnv

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv  # noqa: E402

OUT=ROOT/"artifacts/rl-campaign/surveyor/p3-v7-heading-oracle-diagnostic"
STEPS=250_000


class HeadingOracleObservation(gym.ObservationWrapper):
    """Append privileged ground-truth yaw; never eligible as a contract policy."""
    def __init__(self,env):
        super().__init__(env); self.observation_space=gym.spaces.Box(-np.inf,np.inf,(17,),np.float32)
    def observation(self,observation):
        yaw=float(self.env.unwrapped.last_truth["attitude_rad"][2])
        return np.concatenate((np.asarray(observation,dtype=np.float32),np.asarray([yaw],dtype=np.float32)))


def make(seed=None,rank=0):
    return HeadingOracleObservation(CommonWaypointEnv(ROOT,base_seed=200_000+rank*1_000_000,
                                                        fixed_reset_seed=seed,final_leg_curriculum=True))


def factory(rank): return lambda:make(rank=rank)


def evaluate(model):
    rows=[]
    for seed in range(30000,30050):
        env=make(seed=seed); obs,_=env.reset(); state=None; episode_start=np.ones((1,),dtype=bool); total=0.
        try:
            while True:
                action,state=model.predict(obs,state=state,episode_start=episode_start,deterministic=True)
                obs,reward,terminated,truncated,info=env.step(action);total+=float(reward)
                episode_start=np.asarray([terminated or truncated],dtype=bool)
                if terminated or truncated:break
        finally:env.close()
        rows.append({"seed":seed,"success":bool(info["success"]),"return":total,"termination_reason":info["termination_reason"]})
    return {"episodes":50,"success_rate":sum(r["success"] for r in rows)/50,
            "median_return":float(statistics.median(r["return"] for r in rows)),"rows":rows}


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    env=SubprocVecEnv([factory(i) for i in range(16)],start_method="fork")
    model=RecurrentPPO("MlpLstmPolicy",env,seed=7319,n_steps=512,batch_size=512,n_epochs=10,
        learning_rate=3e-4,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=0,
        policy_kwargs={"net_arch":[128,128],"lstm_hidden_size":128,"enable_critic_lstm":True},
        verbose=1,device="cpu")
    model.learn(total_timesteps=STEPS,progress_bar=False);model.save(OUT/f"phase1-{STEPS}");env.close()
    result=evaluate(model)
    report={"schema_version":1,"artifact_kind":"p3-v7-heading-oracle-ablation",
        "status":"COMPLETE_DIAGNOSTIC_ONLY","contract_compliant":False,"training_budget_steps":STEPS,
        "observation_change":{"base_fields":16,"diagnostic_fields":17,"appended_field":"privileged ground-truth yaw_rad"},
        "control":{"algorithm":"RecurrentPPO","policy":"MlpLstmPolicy","all_other_hyperparameters_match_v7":True},
        "reference":{"classical_success_rate":.68,"v7_recurrent_success_rate_at_250k":.06,"v7_recurrent_peak_success_rate":.22},
        "evaluation":result,"interpretation_rule":"A material jump toward 0.68 supports heading availability as the bottleneck; continued underperformance rejects heading availability as the primary bottleneck.",
        "prohibitions":["Not contract compliant","Not figure eligible","Does not authorize Phase 2, budget extension, or another architecture revision"]}
    (OUT/"report.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({"output":str(OUT/"report.json"),"success_rate":result["success_rate"],"median_return":result["median_return"]},indent=2))


if __name__=="__main__":main()
