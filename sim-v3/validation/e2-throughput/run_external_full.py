"""Fixed-horizon native instance sweep for HoloOcean, Stonefish, and Gazebo."""
from __future__ import annotations

import argparse, concurrent.futures, csv, json, os, platform, resource, subprocess, sys, time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/"packages/python-client"),str(ROOT/"stonefish/python"),str(ROOT/"validation/rl-campaign")]
from bcod_sim import GazeboGymEnv,HoloOceanVehicleAEnv
from stonefish_gym_env import StonefishGymEnv

def make(backend:str,rank:int):
    if backend=="holoocean": return HoloOceanVehicleAEnv(ROOT,base_seed=70000+rank,disturbance_mode="zero",wind_mode="off")
    if backend=="stonefish": return StonefishGymEnv(ROOT,executable=Path("/mnt/shared/gpfs/home/aadik3/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge"),data_dir=Path("/mnt/shared/gpfs/home/aadik3/stonefish-src/Tests/Data"),library_dirs=(Path("/mnt/shared/gpfs/home/aadik3/stonefish-install/lib"),Path("/mnt/shared/gpfs/home/aadik3/stonefish-deps/lib")),physics_threads=1,sensor_noise=False,base_seed=70000+rank,disturbance_mode="zero")
    runtime=[sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]
    return GazeboGymEnv(ROOT,runtime,allow_unconformant_diagnostic=True,base_seed=70000+rank,disturbance_mode="zero")

def worker(backend:str,rank:int,steps:int,warmups:int,repeats:int,barrier) -> dict:
    env=make(backend,rank); action=np.zeros(2,np.float32)
    try:
        completed=0; unexpected=[]
        for phase_count,timed in ((warmups,False),(repeats,True)):
            if timed: barrier.wait(); started=time.perf_counter()
            for rollout in range(phase_count):
                env.reset(seed=70000+rank*100+rollout)
                if hasattr(env,"timeout_steps"): env.timeout_steps=2**31-1
                for step in range(steps):
                    _,_,terminated,truncated,info=env.step(action); completed += int(timed)
                    if terminated or truncated:
                        unexpected.append({"rollout":rollout,"step":step,"reason":info.get("termination_reason")}); break
            if timed: finished=time.perf_counter()
        return {"rank":rank,"completed_steps":completed,"started":started,"finished":finished,"unexpected_terminations":unexpected,"max_rss_kib":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}
    finally: env.close()

def gpu_memory() -> int|None:
    p=subprocess.run(["nvidia-smi","--query-compute-apps=used_memory","--format=csv,noheader,nounits"],text=True,capture_output=True)
    if p.returncode: return None
    vals=[int(x.strip()) for x in p.stdout.splitlines() if x.strip().isdigit()]; return sum(vals)*1024*1024

def cell(backend,count,steps,warmups,repeats):
    import multiprocessing as mp
    manager=mp.Manager(); barrier=manager.Barrier(count)
    with concurrent.futures.ProcessPoolExecutor(max_workers=count) as pool:
        futures=[pool.submit(worker,backend,i,steps,warmups,repeats,barrier) for i in range(count)]
        rows=[f.result() for f in futures]
    wall=max(r["finished"] for r in rows)-min(r["started"] for r in rows); completed=sum(r["completed_steps"] for r in rows)
    return {"device":"native","instance_count":count,"parallelism_kind":"process_instances","rollout_steps":steps,"warmup_rollouts":warmups,"timed_rollouts":repeats,"aggregate_control_steps":completed,"timed_wall_s":wall,"steps_per_second":completed/wall,"ms_per_step":1000*wall/(steps*repeats),"ms_per_environment_step":1000*wall/completed,"memory_used_bytes":sum(r["max_rss_kib"] for r in rows)*1024,"gpu_process_memory_bytes":gpu_memory(),"unexpected_terminations":[x for r in rows for x in r["unexpected_terminations"]]}

def cmd(*args): return subprocess.run(args,text=True,capture_output=True,check=False).stdout.strip()
def main():
    p=argparse.ArgumentParser();p.add_argument("--backend",choices=["holoocean","stonefish","gazebo-harmonic"],required=True);p.add_argument("--counts",type=int,nargs="+",required=True);p.add_argument("--output",type=Path,required=True);p.add_argument("--rollout-steps",type=int,default=300);p.add_argument("--warmups",type=int,default=3);p.add_argument("--repeats",type=int,default=10);a=p.parse_args()
    rows=[];previous=None
    for count in a.counts:
        row=cell(a.backend,count,a.rollout_steps,a.warmups,a.repeats);row["growth_fraction"]=None if previous is None else row["steps_per_second"]/previous-1;rows.append(row);print(json.dumps(row),flush=True)
        if previous is not None and row["growth_fraction"]<.05: row["stop_reason"]="throughput_growth_below_5_percent";break
        previous=row["steps_per_second"]
    scalar=rows[0]; report={"schema_version":1,"artifact_kind":"e2-real-throughput-sweep","status":"COMPLETE","simulator":a.backend,"scope":{"vehicle":"vehicle-a-otter","fixed_horizon":True},"protocol":{"rollout_steps":a.rollout_steps,"warmup_rollouts":a.warmups,"timed_rollouts":a.repeats,"success_termination_expected":False},"hardware":{"hostname":platform.node(),"cpu_model":cmd("bash","-lc","lscpu | sed -n 's/^Model name:[[:space:]]*//p' | head -1"),"ram_bytes":os.sysconf("SC_PAGE_SIZE")*os.sysconf("SC_PHYS_PAGES"),"gpu_model":cmd("nvidia-smi","--query-gpu=name","--format=csv,noheader"),"slurm_job_id":os.environ.get("SLURM_JOB_ID")},"scalar_reference":{"instance_count":1,"steps_per_second":scalar["steps_per_second"],"ms_per_step":scalar["ms_per_step"]},"rows":rows,"headline":{"peak_steps_per_second":max(r["steps_per_second"] for r in rows),"peak_speedup_over_scalar":max(r["steps_per_second"] for r in rows)/scalar["steps_per_second"]},"retraction_root_cause":"NOT_FOUND; this measurement does not resolve the provenance failure in the retracted bcod-sim report."};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n")
    with a.output.with_suffix(".csv").open("w",newline="") as f:
        fields=["device","instance_count","steps_per_second","ms_per_step","ms_per_environment_step","memory_used_bytes","gpu_process_memory_bytes","growth_fraction","stop_reason"];w=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore");w.writeheader();w.writerows(rows)
if __name__=="__main__":main()
