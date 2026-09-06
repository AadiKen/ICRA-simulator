#!/usr/bin/env python3
"""Allocated-node discovery and bounded env-count throughput selection."""
import json,os,platform,subprocess,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).parent))
from bcod_sim import CommonWaypointEnv
from run_task5_conditions import factory

OUT=ROOT/'artifacts/rl-campaign/surveyor/15-field-cluster-rerun';OUT.mkdir(parents=True,exist_ok=True)
def command(*args):
 try:return subprocess.run(args,text=True,capture_output=True,check=True).stdout.strip()
 except Exception:return None
def bench(count):
 from stable_baselines3.common.vec_env import SubprocVecEnv
 env=SubprocVecEnv([factory(i,'default-noise') for i in range(count)],start_method='fork')
 started=time.perf_counter()
 try:
  env.reset()
  for _ in range(100):env.step(np.zeros((count,2),np.float32))
  elapsed=time.perf_counter()-started
 finally:env.close()
 return {'parallel_environments':count,'control_transitions':count*100,'wall_clock_s':elapsed,'transitions_per_s':count*100/elapsed}
def main():
 if not os.environ.get('SLURM_JOB_ID'):raise RuntimeError('Preflight must execute inside an sbatch allocation')
 import sb3_contrib
 cores=int(command('nproc') or os.cpu_count() or 1);candidates=sorted(set([16,min(32,cores),min(64,cores)]));measurements=[bench(x) for x in candidates]
 # Require at least 10% throughput improvement before increasing process count.
 baseline=next(x for x in measurements if x['parallel_environments']==16);eligible=[x for x in measurements if x['transitions_per_s']>=baseline['transitions_per_s']*1.10];selected=max(eligible or [baseline],key=lambda x:x['transitions_per_s'])['parallel_environments']
 decision={'schema_version':1,'artifact_kind':'15-field-cluster-runtime-decision','status':'COMPLETE','host_class':'cluster','contract_sha256':CommonWaypointEnv.EXPECTED_CONTRACT_SHA256,'allocated_node':{'hostname':platform.node(),'logical_cpu_count':cores,'cpu_model':command('bash','-lc',"lscpu | sed -n 's/^Model name:[[:space:]]*//p' | head -1"),'gpu_model':command('bash','-lc',"nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | paste -sd ',' -") or None,'slurm':{k:os.environ.get(k) for k in ('SLURM_JOB_ID','SLURM_JOB_NODELIST','SLURM_JOB_CPUS_PER_NODE','SLURM_CPUS_ON_NODE','SLURM_MEM_PER_NODE','SLURM_JOB_PARTITION','SLURM_JOB_ACCOUNT')}},'python':sys.version,'sb3_contrib_version':getattr(sb3_contrib,'__version__','unknown'),'throughput_measurements':measurements,'parallel_environments':selected,'n_steps':512,'batch_size':512,'batch_size_decision':'Retained at 512: it is a frozen hyperparameter and exactly divides 512*n_env for every candidate. Raising n_env changes the number of minibatches per rollout but does not create a partial minibatch.','selection_rule':'Increase above 16 only for at least 10% measured transition-throughput improvement on the allocated node.'}
 (OUT/'cluster-runtime-decision.json').write_text(json.dumps(decision,indent=2)+'\n');print(json.dumps(decision,indent=2))
if __name__=='__main__':main()
