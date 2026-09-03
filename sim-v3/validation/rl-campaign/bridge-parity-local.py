from __future__ import annotations
import hashlib,json,platform,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"));sys.path.insert(0,str(ROOT/"backends/tensor"))
from bcod_sim import PersistentNodeBridge
def canonical(x):return json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=False).encode()
def sha(x):return hashlib.sha256(canonical(x)).hexdigest()
def config(seed,plant,dt=.05):return{"schema_version":1,"experiment":{"name":f"bridge-{plant}-{seed}","seed":seed,"timestep_s":dt,"duration_s":2},"backend":{"type":"node"},"vehicle":{"preset":"searobotics-surveyor-m1.8" if plant=="planar3" else "vehicle-c-azimuth","plant":plant},"mission":{"type":"hold"}}
def action(plant,step,steps):
 phase=step/steps
 return {"portCommand":.2+.3*phase,"starboardCommand":.4-.2*phase} if plant=="planar3" else {"surgeForce":20+10*phase,"yawMoment":2-4*phase}
def first_diff(a,b):
 for i,(x,y) in enumerate(zip(a,b)):
  if canonical(x)!=canonical(y):return i
 return None if len(a)==len(b) else min(len(a),len(b))
def bridged(plant,steps=20,dt=.05,batch=1):
 with PersistentNodeBridge(ROOT) as bridge:
  bridge.reset([config(73+i,plant,dt) for i in range(batch)]);trace=[];start=time.perf_counter()
  for step in range(steps):
   bridge.step([action(plant,step,steps) for _ in range(batch)]);trace.append(bridge.ground_truth(0))
  elapsed=time.perf_counter()-start
 return trace,{"batch":batch,"steps":steps*batch,"wall_clock_s":elapsed,"aggregate_steps_per_s":steps*batch/elapsed,"host_class":"local","paper_measurement_eligible":False}
cells=[]
for plant in ("planar3","coupled6"):
 direct_args=["node","--experimental-strip-types","validation/rl-campaign/direct-trace.ts",plant,"20","0.05"]+(["searobotics-surveyor-m1.8"] if plant=="planar3" else []);direct=json.loads(subprocess.check_output(direct_args,cwd=ROOT,text=True))["trace"];candidate,timing=bridged(plant);cells.append({"leg":"persistent-bridge-vs-direct-node","plant":plant,"vehicle":"searobotics-surveyor-m1.8" if plant=="planar3" else "vehicle-c-azimuth","reference_sha256":sha(direct),"candidate_sha256":sha(candidate),"bit_identical":direct==candidate,"first_divergence_step":first_diff(direct,candidate),"timing":timing})
normal,_=bridged("planar3");perturbed,_=bridged("planar3",dt=.051);negative={"leg":"negative-control-timestep-perturbation","parameter":{"timestep_s":{"reference":.05,"perturbed":.051}},"detected":normal!=perturbed,"first_divergence_step":first_diff(normal,perturbed)}
if not negative["detected"]:raise RuntimeError("Negative control was not detected; parity result is invalid")
for batch in (1,8,64):_,timing=bridged("planar3",steps=100,batch=batch);cells.append({"leg":"throughput-budget-projection-only","plant":"planar3","timing":timing})
probe=json.loads(subprocess.check_output([str(ROOT/".venv/bin/python"),"validation/determinism-sweep/backend_probe.py"],cwd=ROOT,text=True));cells.append({"leg":"node-vs-tensor-cpu-float64","plant":"planar3",**probe,"classification":"expected machine-epsilon non-associativity" if not probe["bit_identical"] and probe["divergence_magnitude"]<=sys.float_info.epsilon else "pass" if probe["bit_identical"] else "fail"})
try:
 import torch;mps_available=torch.backends.mps.is_available()
except Exception:mps_available=False
report={"schema_version":1,"artifact_kind":"bridge-parity-local","host_class":"local","paper_measurement_eligible":False,"architecture_decision":"persistent-batched-node-service","negative_control":negative,"cells":cells,"mps":{"requested":True,"available":mps_available,"configured_numerics_mode":"float32","configuration_cpu_smoke_test":"covered by backends/tensor/tests/test_backend.py","status":"blocked-local-runtime" if not mps_available else "available-not-executed-by-this-harness","reason":"torch.backends.mps.is_available() returned false" if not mps_available else "MPS runtime is available"},"tensor_scope":{"planar3":"Vehicle A only","coupled6":"not implemented; cannot claim tensor parity"},"platform":{"python":platform.python_version(),"machine":platform.machine(),"system":platform.system()}}
out=ROOT/"artifacts/rl-campaign/bridge-parity-local.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps({"output":str(out),"negative_control":negative,"parity":[(x.get("plant"),x.get("bit_identical")) for x in cells if x["leg"]=="persistent-bridge-vs-direct-node"],"mps":report["mps"]},indent=2))
