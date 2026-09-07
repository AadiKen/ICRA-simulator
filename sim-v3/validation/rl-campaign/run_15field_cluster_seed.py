#!/usr/bin/env python3
"""One frozen 1.5M 15-field RecurrentPPO seed, intended for SLURM array use."""
import json,os,statistics,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).parent))
from bcod_sim import CommonWaypointEnv
from run_task5_conditions import evaluate,factory,atomic_json
from train_portable_ppo import detect_host_class
CONTRACT='2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36';BASE=ROOT/'artifacts/rl-campaign/surveyor/15-field-cluster-rerun';FIELDS=['imu.linear_accel_x','imu.linear_accel_y','imu.linear_accel_z','imu.angular_rate_x','imu.angular_rate_y','imu.angular_rate_z','imu.orientation_yaw_rad','gps.relative_goal_north_m','gps.relative_goal_east_m','gps.fix_valid','previous_action.effector_0','previous_action.effector_1','previous_action.steer_0','previous_action.steer_1','normalized_time_remaining']
def main():
 if not os.environ.get('SLURM_JOB_ID'):raise RuntimeError('Cluster rerun must execute under sbatch')
 host_class=detect_host_class()
 if host_class!='cluster':raise RuntimeError(f'Cluster rerun detected host_class={host_class!r}')
 if CommonWaypointEnv.EXPECTED_CONTRACT_SHA256!=CONTRACT:raise RuntimeError('Unexpected contract')
 from sb3_contrib import RecurrentPPO
 from stable_baselines3.common.vec_env import SubprocVecEnv
 decision=json.loads((BASE/'cluster-runtime-decision.json').read_text());n_env=int(decision['parallel_environments']);index=int(os.environ.get('SLURM_ARRAY_TASK_ID','0'));seeds=[7319,7320,7321];seed=seeds[index];out=BASE/f'seed-{seed}';out.mkdir(parents=True,exist_ok=True)
 if n_env!=16:raise RuntimeError(f'Frozen CodeNimbus configuration requires 16 environments, found {n_env}')
 env=SubprocVecEnv([factory(i,'default-noise') for i in range(n_env)],start_method='fork');model=RecurrentPPO('MlpLstmPolicy',env,seed=seed,n_steps=512,batch_size=512,n_epochs=10,learning_rate=3e-4,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=.005,policy_kwargs={'net_arch':[128,128],'lstm_hidden_size':128,'enable_critic_lstm':True},verbose=1,device='cuda');curve=[];started=time.time()
 try:
  for checkpoint in range(250_000,1_500_001,250_000):
   model.learn(total_timesteps=250_000,reset_num_timesteps=False,progress_bar=False);model.save(out/f'phase1-{checkpoint}');result=evaluate(model,'default-noise')
   for row in result['rows']:row.update({'host_class':host_class,'observation_field_count':15,'contract_sha256':CONTRACT})
   curve.append({'checkpoint_steps':checkpoint,'actual_model_timesteps':model.num_timesteps,'evaluation':result});atomic_json(out/'report.json',{'schema_version':1,'artifact_kind':'15-field-cluster-recurrent-ppo-seed','status':'running','host_class':host_class,'contract_sha256':CONTRACT,'observation_schema':FIELDS,'training':{'seed':seed,'algorithm':'RecurrentPPO','policy':'MlpLstmPolicy','steps':1_500_000,'parallel_environments':n_env,'n_steps':512,'batch_size':512,'n_epochs':10,'learning_rate':3e-4,'gamma':.99,'gae_lambda':.95,'clip_range':.2,'ent_coef':.005,'curriculum':'final-leg-isolation','device':'cuda'},'curve':curve})
  report=json.loads((out/'report.json').read_text());report.update({'status':'complete','wall_clock_s':time.time()-started});atomic_json(out/'report.json',report)
 finally:env.close()
if __name__=='__main__':main()
