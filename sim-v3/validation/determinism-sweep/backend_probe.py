import hashlib,json,pathlib,subprocess,sys
import torch
ROOT=pathlib.Path(__file__).parents[2];sys.path.insert(0,str(ROOT/"backends/tensor"))
from bcod_tensor import VehicleAPlanar3Config,VehicleAPlanar3TensorBackend
steps=40;dt=.05
reference=json.loads(subprocess.check_output(["node","--experimental-strip-types","validation/tensor/node_reference.mjs",str(steps),str(dt)],cwd=ROOT,text=True))["trace"]
backend=VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=1,timestep_s=dt));actual=[]
for item in reference:
 command=torch.tensor([item["command"]],dtype=torch.float64);state=backend.step(command)
 actual.append({"state":state[0].tolist(),"acceleration":backend.body_acceleration[0].tolist(),"thruster_state":backend.thruster_state[0].tolist(),"energy_j":backend.energy_j[0].item(),"power_w":backend.power_w[0].item(),"step":backend.step_counters[0].item()})
expected=[{k:item[k] for k in ("state","acceleration","thruster_state","energy_j","power_w","step")} for item in reference]
canonical=lambda x:json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=False).encode();sha=lambda x:hashlib.sha256(canonical(x)).hexdigest()
first=None;maximum=0.0
for i,(a,b) in enumerate(zip(expected,actual)):
 values=[]
 for field in ("state","acceleration","thruster_state"):values.extend(abs(float(x)-float(y)) for x,y in zip(a[field],b[field]))
 values.extend([abs(float(a["energy_j"])-float(b["energy_j"])),abs(float(a["power_w"])-float(b["power_w"])),abs(float(a["step"])-float(b["step"]))]);m=max(values)
 if m and first is None:first=i
 maximum=max(maximum,m)
print(json.dumps({"reference_sha256":sha(expected),"candidate_sha256":sha(actual),"bit_identical":sha(expected)==sha(actual),"first_divergence_step":first,"divergence_magnitude":maximum}))
