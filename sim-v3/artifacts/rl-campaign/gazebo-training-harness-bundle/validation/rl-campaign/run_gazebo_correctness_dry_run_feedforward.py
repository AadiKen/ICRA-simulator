#!/usr/bin/env python3
"""Local AMD64-emulated Gazebo correctness run. Never a throughput benchmark."""
from __future__ import annotations
import argparse,hashlib,json,platform,sys,time
from multiprocessing.connection import wait as wait_connections
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"));sys.path.insert(0,str(ROOT/"validation/rl-campaign"))
from bcod_sim import GazeboGymEnv
from train_portable_ppo_feedforward import EPISODE_COLUMNS,assert_execution_authorized,episode_metric_row
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env.subproc_vec_env import _stack_obs

OUT=ROOT/"artifacts/rl-campaign/gazebo-correctness-dry-run-feedforward"
RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]

def atomic(path,value):
 path.parent.mkdir(parents=True,exist_ok=True);tmp=Path(str(path)+".tmp");tmp.write_text(json.dumps(value,indent=2)+"\n");tmp.replace(path)
def announce(message):print(f"[correctness-only] {message}",flush=True)
def make(seed):return GazeboGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,base_seed=seed,disturbance_mode="zero")
def factory(rank):return lambda:make(41000+rank*1000)

class TerminalProgress(BaseCallback):
 def __init__(self,total_timesteps,report_every_percent=5):
  super().__init__();self.total_timesteps=total_timesteps;self.increment=max(1,total_timesteps*report_every_percent//100);self.next_report=self.increment;self.started=0.
 def _on_training_start(self):
  self.started=time.monotonic();announce(f"policy update started: 0/{self.total_timesteps} steps (0%)")
 def _on_step(self):
  completed=min(self.num_timesteps,self.total_timesteps)
  if completed>=self.next_report or completed>=self.total_timesteps:
   elapsed=max(time.monotonic()-self.started,1e-9);rate=completed/elapsed;remaining=max(0,self.total_timesteps-completed);eta=remaining/rate if rate else float("inf")
   announce(f"policy progress: {completed}/{self.total_timesteps} steps ({100*completed/self.total_timesteps:.0f}%), {rate:.2f} steps/s, ETA {eta/60:.1f} min")
   while self.next_report<=completed:self.next_report+=self.increment
  return True

class FailFastSubprocVecEnv(SubprocVecEnv):
 """Subprocess vector env that surfaces a dead Gazebo worker promptly."""
 worker_timeout_s=120.
 def step_wait(self):
  deadline=time.monotonic()+self.worker_timeout_s;pending={remote:i for i,remote in enumerate(self.remotes)};results=[None]*len(self.remotes)
  while pending:
   dead=[(i,p.exitcode) for i,p in enumerate(self.processes) if p.exitcode is not None]
   if dead:raise RuntimeError(f"Gazebo environment worker exited during step: {dead}")
   remaining=deadline-time.monotonic()
   if remaining<=0:raise TimeoutError(f"Gazebo environment step exceeded {self.worker_timeout_s:.0f}s; worker exit codes: {[p.exitcode for p in self.processes]}")
   for remote in wait_connections(list(pending),timeout=min(1.,remaining)):
    index=pending.pop(remote)
    try:results[index]=remote.recv()
    except EOFError as error:raise RuntimeError(f"Gazebo environment worker {index} closed its result pipe") from error
  self.waiting=False
  obs,rews,dones,infos,self.reset_infos=zip(*results,strict=True)
  return _stack_obs(obs,self.observation_space),np.stack(rews),np.stack(dones),infos

def termination_exercises():
 env=make(40000);results={}
 try:
  env.reset();status=env.bridge._request({"op":"diagnostic_status"});payload={"active_sensors":["imu","gps"],"actuators":{"effectors":{"port":{"command":1.0},"starboard":{"command":1.0}}}}
  for kind in ("grounding","object_collision","instability","allocation_failure","precedence"):
   env.bridge._request({"op":"diagnostic_termination_override","kind":kind});response=None
   for ticks in range(1,25):
    response=env.bridge._request({"op":"step","action":payload})
    if response["terminated"]:break
   results[kind]={"terminated":bool(response and response["terminated"]),"stop_reason":None if response is None else response.get("info",{}).get("stop_reason"),"physics_ticks":ticks}
  return status,results
 finally:env.close()

def main(argv=None):
 args=argparse.ArgumentParser(description=__doc__);args.add_argument("--timesteps",type=int,default=10_000,help="PPO interaction steps; use 256 for a short debugging pass (default: 10000)");args=args.parse_args(argv)
 if args.timesteps<64:raise SystemExit("--timesteps must be at least 64 (one complete two-environment rollout)")
 assert_execution_authorized()
 requested_timesteps=args.timesteps
 OUT.mkdir(parents=True,exist_ok=True);started=time.perf_counter();announce("starting local AMD64-emulated validation; timing is not representative of cluster throughput")
 announce("exercising the four terminations and precedence");status,terminations=termination_exercises();announce(f"termination checks complete: {terminations}")
 announce("constructing and resetting 2 Gazebo environments")
 env=FailFastSubprocVecEnv([factory(0),factory(1)],start_method="fork");initial=env.reset();construct={"parallel_environments":2,"selection_basis":"small local correctness run only; not cluster performance","reset_shape":list(initial.shape),"finite":bool(np.isfinite(initial).all())};announce(f"environment reset complete: observation shape {tuple(initial.shape)}, finite={construct['finite']}")
 model=PPO("MlpPolicy",env,seed=7319,n_steps=32,batch_size=64,n_epochs=1,learning_rate=3e-4,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=.005,policy_kwargs={"net_arch":[64,64]},verbose=0,device="cpu")
 before=[p.detach().clone() for p in model.policy.parameters()];train_started=time.perf_counter();model.learn(total_timesteps=requested_timesteps,callback=TerminalProgress(requested_timesteps),progress_bar=False);train_wall=time.perf_counter()-train_started
 updates_applied=any(not np.array_equal(a.cpu().numpy(),b.detach().cpu().numpy()) for a,b in zip(before,model.policy.parameters()))
 announce(f"policy update complete in {train_wall/60:.1f} min; parameters changed={updates_applied}")
 checkpoint=OUT/"feedforward-ppo-correctness-only";model.save(checkpoint);env.close();checkpoint=checkpoint.with_suffix(".zip");announce(f"checkpoint saved: {checkpoint.relative_to(ROOT)}")

 announce("starting feedforward memoryless evaluation")
 loaded=PPO.load(checkpoint,device="cpu");evaluation_env=make(50000);eval_started=time.perf_counter();total=0.;prediction_calls=[]
 try:
  obs,_=evaluation_env.reset();evaluation_env.bridge._request({"op":"diagnostic_termination_override","kind":"allocation_failure"})
  for controls in range(1,30):
   action,_=loaded.predict(obs,deterministic=True);prediction_calls.append({"control":controls,"deterministic":True});obs,reward,terminated,truncated,info=evaluation_env.step(action);total+=float(reward)
   if terminated or truncated:break
  row=episode_metric_row(seed=50000,env=evaluation_env,info=info,total_return=total,wall_clock_s=time.perf_counter()-eval_started,policy_id="PPO-correctness-only",algorithm="PPO")
 finally:evaluation_env.close()
 announce(f"evaluation complete after {controls} controls; termination={info.get('termination_reason')}")
 rows_path=OUT/"episode-rows.json";atomic(rows_path,{"schema_version":1,"columns":list(EPISODE_COLUMNS),"rows":[row]});announce("15-field episode row written")
 report={"schema_version":1,"artifact_kind":"gazebo-local-correctness-dry-run","status":"PASS" if all(x["terminated"] for x in terminations.values()) and updates_applied and tuple(row)==EPISODE_COLUMNS else "FAIL","classification":"correctness-only validation; not a Gate D throughput measurement","run_scope":"Task 4 10k acceptance" if requested_timesteps==10_000 else "short debugging pass; does not satisfy Task 4 10k acceptance","training_authorized":False,"scientific_training_result":False,"runtime":{"image":"icra27-gazebo-harmonic:harmonic-8.15.0","image_id":"174e8baad590","host_architecture":platform.machine(),"container_architecture":"linux/amd64","execution":"AMD64-emulated on Apple Silicon","timing_label":"AMD64-emulated, not representative of cluster throughput"},"still_water_running_instance":status["environment"],"construction":construct,"rollout_and_update":{"requested_timesteps":requested_timesteps,"actual_timesteps":loaded.num_timesteps,"updates_applied":updates_applied,"wall_clock_s":train_wall,"timing_label":"AMD64-emulated, not representative of cluster throughput"},"checkpoint":{"path":str(checkpoint.relative_to(ROOT)),"exists":checkpoint.exists(),"sha256":hashlib.sha256(checkpoint.read_bytes()).hexdigest()},"evaluation":{"controls":controls,"memoryless_predict":True,"trace":prediction_calls,"episode_row_path":str(rows_path.relative_to(ROOT)),"episode_row_columns":list(row),"host_class":row["host_class"]},"termination_exercises":terminations,"precedence_expected":["instability","grounding","object_collision","allocation_failure"],"gate_d":{"throughput_measurement":False,"parallel_environments_selected_for_cluster":False,"budget_decision_made":False,"cluster_access_still_blocked":True},"total_wall_clock_s":time.perf_counter()-started}
 atomic(OUT/"report.json",report);announce(f"FINAL STATUS: {report['status']}; report written to {OUT.relative_to(ROOT)}/report.json");print(json.dumps({k:v for k,v in report.items() if k not in ("evaluation",)},indent=2),flush=True);return 0 if report["status"]=="PASS" else 1
if __name__=="__main__":raise SystemExit(main())
