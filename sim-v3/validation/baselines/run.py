from __future__ import annotations
import hashlib,json,pathlib,sys
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
sys.path.insert(0,str(pathlib.Path(__file__).parents[2]/"packages/python-client"))
from bcod_sim import BCODGymEnv,EnvConfig,MPCPolicy,PIDPolicy

ROOT=pathlib.Path(__file__).parents[2]
SEEDS=[41,42,43]

class RandomInitialState(gym.Wrapper):
    def reset(self,*,seed=None,options=None):
        observation,info=self.env.reset(seed=seed,options=options)
        state=self.np_random.uniform(-1,1,12);state[2:5]=0;state[8:11]=0
        return self.env.reset(seed=seed,options={"initial_state":state})

def make_env(seed:int=2027):
    return RandomInitialState(BCODGymEnv(EnvConfig(timestep_s=.05,duration_s=4,seed=seed,mass_kg=1,linear_damping=.5,plant="planar3")))

def evaluate(policy,seed:int)->dict:
    env=make_env(seed);obs,_=env.reset(seed=seed);reward=0.0
    for _ in range(80):
        action=policy.action(obs) if hasattr(policy,"action") else policy.predict(obs,deterministic=True)[0]
        obs,r,_,truncated,_=env.step(action);reward+=float(r)
        if truncated:break
    return{"seed":seed,"steps":env.unwrapped._steps,"return":reward,"terminal_error":float(np.linalg.norm(obs[:6])),"finite":bool(np.isfinite(obs).all())}

def actor_export(model:PPO)->dict:
    policy=model.policy
    layers=[module for module in policy.mlp_extractor.policy_net if isinstance(module,torch.nn.Linear)]+[policy.action_net]
    return{"schema_version":1,"activation":"tanh","observation_order":["north_error","east_error","down_error","roll_error","pitch_error","yaw_error","u","v","w","p","q","r"],"action_order":["surge","sway","heave","roll","pitch","yaw"],"layers":[{"weight":layer.weight.detach().cpu().tolist(),"bias":layer.bias.detach().cpu().tolist()} for layer in layers],"deterministic_action":"clip(action_net(policy_net(observation)),-1,1)"}

def run(output:pathlib.Path):
    output.mkdir(parents=True,exist_ok=True);(output/"checkpoints").mkdir(exist_ok=True)
    seed=2027;torch.use_deterministic_algorithms(True);torch.manual_seed(seed);np.random.seed(seed)
    model=PPO("MlpPolicy",make_env(seed),seed=seed,n_steps=128,batch_size=64,n_epochs=4,learning_rate=3e-4,gamma=.98,policy_kwargs={"net_arch":[32,32]},verbose=0,device="cpu")
    model.learn(total_timesteps=16384,progress_bar=False);model.save(output/"checkpoints/ppo-cpu")
    actor=actor_export(model);actor_raw=json.dumps(actor,sort_keys=True,separators=(",",":")).encode();(output/"checkpoints/ppo-actor.json").write_text(json.dumps(actor,indent=2)+"\n")
    policies={"pid":PIDPolicy(),"mpc":MPCPolicy(),"ppo":model};results={name:[evaluate(policy,s) for s in SEEDS] for name,policy in policies.items()}
    artifact={"schema_version":2,"artifact_kind":"offline-control-baselines","status":"passed-real-ppo-software-baselines-not-physical-validation","backend":"cpu","training":{"algorithm":"PPO","implementation":"stable-baselines3","implementation_version":__import__("stable_baselines3").__version__,"algorithmic_novelty_claim":False,"seed":seed,"total_timesteps":16384,"actor_sha256":hashlib.sha256(actor_raw).hexdigest(),"checkpoint":"checkpoints/ppo-cpu.zip","actor_export":"checkpoints/ppo-actor.json"},"policies":results,"claim_limit":"PID, MPC, and PPO are conventional software baselines. Training uses the Gym contract's transport-neutral stabilization task; production USV-Bench scoring is a separate artifact and is not physical validation.","external_backends":{"cuda":"blocked-external-infrastructure","mps":"blocked-external-infrastructure"}}
    (output/"report.json").write_text(json.dumps(artifact,indent=2)+"\n");return artifact

if __name__=="__main__":print(json.dumps(run(pathlib.Path(sys.argv[1] if len(sys.argv)>1 else ROOT/"artifacts/baselines")),indent=2))
