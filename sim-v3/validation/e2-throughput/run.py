"""Short E2 pilot: measure a few steps and explicitly extrapolate the full campaign."""
from __future__ import annotations

import argparse, csv, json, os, platform, subprocess, sys, time
from pathlib import Path
import torch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'backends/tensor'))
from bcod_tensor import VehicleAPlanar3Config,VehicleAPlanar3TensorBackend

def hardware():
    cpu=platform.processor() or platform.machine()
    try:
        if sys.platform=='darwin': cpu=subprocess.check_output(['sysctl','-n','machdep.cpu.brand_string'],text=True).strip()
    except (OSError,subprocess.SubprocessError): pass
    try: ram=os.sysconf('SC_PAGE_SIZE')*os.sysconf('SC_PHYS_PAGES')
    except (AttributeError,ValueError,OSError): ram=None
    return {'hostname':platform.node(),'os':platform.platform(),'cpu':cpu,'ram_bytes':ram,'torch':torch.__version__,'cuda':torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,'mps_available':torch.backends.mps.is_available()}

def devices():
    found=['cpu']
    if torch.cuda.is_available(): found.append('cuda')
    if torch.backends.mps.is_available(): found.append('mps')
    return found

def synchronize(device):
    if device=='cuda': torch.cuda.synchronize()
    elif device=='mps': torch.mps.synchronize()

def measure(device,batch,sample_steps,warmup_steps,rollout_steps,full_rollouts):
    numerics='float32' if device=='mps' else 'float64'
    backend=VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=batch,device=device,numerics_mode=numerics))
    command=torch.tensor([[.4,.3]],dtype=backend.dtype,device=backend.device).repeat(batch,1)
    for _ in range(warmup_steps): backend.step(command)
    synchronize(device); started=time.perf_counter()
    for _ in range(sample_steps): backend.step(command)
    synchronize(device); elapsed=time.perf_counter()-started
    aggregate=batch*sample_steps
    return {'device':device,'batch_size':batch,'numerics_mode':numerics,'measurement_class':'heuristic-pilot-extrapolation','sample_vector_steps':sample_steps,'sample_aggregate_environment_steps':aggregate,'measured_wall_clock_s':elapsed,'steps_per_second':aggregate/elapsed,'ms_per_step':1000*elapsed/sample_steps,'ms_per_environment_step':1000*elapsed/aggregate,'bridge_overhead_ms':None,'bridge_overhead_status':'deferred: bridge is not isolated by the tensor backend','memory_used_bytes':None,'memory_status':'deferred: portable allocator accounting is not implemented','estimated_wall_clock_per_rollout_s':elapsed*rollout_steps/sample_steps,'estimated_full_protocol_wall_clock_s':elapsed*rollout_steps*full_rollouts/sample_steps,'estimate_assumption':'linear steady-state scaling from the timed vector steps; excludes initialization and later-run thermal/allocator effects'}

def main():
    p=argparse.ArgumentParser();p.add_argument('--batch-sizes',type=int,nargs='+',default=[1,2,4]);p.add_argument('--sample-steps',type=int,default=16);p.add_argument('--warmup-steps',type=int,default=4);p.add_argument('--rollout-steps',type=int,default=300);p.add_argument('--full-rollouts',type=int,default=13);p.add_argument('--output',type=Path,default=ROOT/'artifacts/e2-throughput/heuristic-pilot.json');a=p.parse_args()
    if a.sample_steps<2 or a.warmup_steps<0 or a.rollout_steps<1 or a.full_rollouts<1 or any(x<1 for x in a.batch_sizes): p.error('counts must be positive (warmup may be zero)')
    rows=[measure(d,b,a.sample_steps,a.warmup_steps,a.rollout_steps,a.full_rollouts) for d in devices() for b in a.batch_sizes]
    scalar=json.loads(subprocess.check_output(['node',str(Path(__file__).with_name('scalar_probe.mjs')),str(a.sample_steps),str(a.warmup_steps)],cwd=ROOT,text=True))
    for row in rows: row['speedup_over_scalar']=row['steps_per_second']/scalar['steps_per_second']
    report={'schema_version':1,'artifact_kind':'e2-throughput-heuristic-pilot','status':'PILOT_ESTIMATES_NOT_PUBLICATION_MEASUREMENTS','scope':{'vehicle':'vehicle-a-otter','plant':'planar3','fixed_length_rollout_steps':a.rollout_steps,'sample_vector_steps':a.sample_steps,'warmup_vector_steps':a.warmup_steps,'full_protocol_rollouts_per_cell':a.full_rollouts},'hardware':hardware(),'scalar_reference':{**scalar,'implementation':'Node Vehicle A planar3 production reference','measurement_class':'heuristic-pilot'},'rows':rows,'deferred':['Full powers-of-two sweep through saturation','At least 3 full-rollout warmups and 10 timed full rollouts per cell','Isolated Node-to-Python bridge timing','Portable device memory measurement','E4 four-arm training/evaluation campaign','E5 grounded-vs-synthetic training/evaluation campaign'],'interpretation':'All total-duration values are linear extrapolations from a short step sample. Re-run the full protocol before publication.'}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    csv_path=a.output.with_suffix('.csv')
    with csv_path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps({'output':str(a.output),'csv':str(csv_path),'status':report['status'],'cells':len(rows)},indent=2))

if __name__=='__main__': main()
