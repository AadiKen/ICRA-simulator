import hashlib,json,math,subprocess,sys
from pathlib import Path
import torch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"backends/tensor"));from bcod_tensor import VehicleAPlanar3Config,VehicleAPlanar3TensorBackend
protocol_path=ROOT/"artifacts/rl-campaign/long-horizon-parity-protocol-v1.1.json";protocol=json.loads(protocol_path.read_text());log_steps=set(protocol["reporting"]["log_steps"]);tol=protocol["acceptance"]
def canonical(x):return json.dumps(x,sort_keys=True,separators=(",",":"),allow_nan=False).encode()
def run(case,tau=.25):
 ref=json.loads(subprocess.check_output(["node","validation/rl-campaign/node-tensor-long-reference.mjs",json.dumps(case,separators=(",",":"))],cwd=ROOT,text=True));b=VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=1,timestep_s=.05,thruster_tau_s=tau));b.pose_ned_yaw[0]=torch.tensor(case["state"][:3],dtype=torch.float64);b.body_velocity[0]=torch.tensor(case["state"][3:],dtype=torch.float64);current=torch.tensor([case["current_ned_mps"]],dtype=torch.float64);cand=[]
 for k in range(2400):
  cmd=torch.tensor([[max(-1,min(1,.55*math.sin(.013*k)+.25*math.cos(.031*k))),max(-1,min(1,.50*math.sin(.017*k+.4)-.20*math.cos(.029*k)))]],dtype=torch.float64);state=b.step(cmd,current);cand.append({"state":state[0].tolist(),"acceleration":b.body_acceleration[0].tolist(),"thruster_state":b.thruster_state[0].tolist(),"energy_j":b.energy_j[0].item()})
 return ref,cand
def compare(ref,cand):
 maxima={k:{"value":0.,"step":None} for k in ["position_m","yaw_rad","yaw_circular_diagnostic_rad","linear_velocity_m_s","yaw_rate_rad_s","acceleration","thruster_force_n","energy_j"]};first=None;samples=[]
 for i,(a,b) in enumerate(zip(ref,cand),1):
  yaw_delta=a["state"][2]-b["state"][2];groups={"position_m":[abs(a["state"][j]-b["state"][j]) for j in (0,1)],"yaw_rad":[abs(yaw_delta)],"yaw_circular_diagnostic_rad":[abs(math.atan2(math.sin(yaw_delta),math.cos(yaw_delta)))],"linear_velocity_m_s":[abs(a["state"][j]-b["state"][j]) for j in (3,4)],"yaw_rate_rad_s":[abs(a["state"][5]-b["state"][5])],"acceleration":[abs(x-y) for x,y in zip(a["acceleration"],b["acceleration"])],"thruster_force_n":[abs(x-y) for x,y in zip(a["thruster_state"],b["thruster_state"])],"energy_j":[abs(a["energy_j"]-b["energy_j"])]};stepmax=max(max(x) for k,x in groups.items() if k!="yaw_circular_diagnostic_rad");physical_stepmax=max(max(x) for k,x in groups.items() if k!="yaw_rad");first=i if first is None and stepmax>0 else first
  for key,values in groups.items():
   value=max(values)
   if value>maxima[key]["value"]:maxima[key]={"value":value,"step":i}
  if i in log_steps:samples.append({"step":i,"raw_max_abs":stepmax,"circular_yaw_physical_max_abs":physical_stepmax})
 limits={"position_m":tol["position_m_max_abs"],"yaw_circular_diagnostic_rad":tol["yaw_circular_rad_max_abs"],"linear_velocity_m_s":tol["linear_velocity_m_s_max_abs"],"yaw_rate_rad_s":tol["yaw_rate_rad_s_max_abs"],"acceleration":tol["acceleration_max_abs"],"thruster_force_n":tol["thruster_force_n_max_abs"],"energy_j":tol["energy_j_max_abs"]};passed=all(maxima[k]["value"]<=limits[k] for k in limits)
 return{"passed":passed,"first_nonzero_step":first,"maxima":maxima,"log_samples":samples,"reference_sha256":hashlib.sha256(canonical(ref)).hexdigest(),"candidate_sha256":hashlib.sha256(canonical(cand)).hexdigest()}
cells=[]
for case in protocol["initial_conditions"]:
 ref,cand=run(case);cells.append({"plant":"planar3","initial_condition":case["id"],**compare(ref,cand)})
ref,perturbed=run(protocol["initial_conditions"][0],.251);negative=compare(ref,perturbed);detected=not negative["passed"]
if not detected:raise RuntimeError("Predeclared negative control did not fail")
report={"schema_version":1,"protocol_version":"1.1.0","protocol_sha256":hashlib.sha256(protocol_path.read_bytes()).hexdigest(),"retained_v1_failure":"artifacts/rl-campaign/long-horizon-parity.json","rollout_steps":2400,"negative_control":{"detected":detected,"parameter":"thruster_tau_s","reference":.25,"perturbed":.251,"result":negative},"cells":cells,"summary":{"planar3_passed":all(x["passed"] for x in cells),"planar3_cases":len(cells),"coupled6_status":"not-run-tensor-backend-not-implemented","gate_passed":False,"reason":"A required plant is missing even if every amended planar3 cell passes."}};out=ROOT/"artifacts/rl-campaign/long-horizon-parity-v1.1.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report["summary"],indent=2))
