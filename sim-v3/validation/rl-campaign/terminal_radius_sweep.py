from __future__ import annotations
import hashlib,json,math,os,statistics,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim.node_bridge import PersistentNodeBridge
from bcod_sim.common_task_env import Mulberry32
P25=json.loads((ROOT/"artifacts/rl-campaign/p2.5-corrected-corridor.json").read_text());PROTOCOL=json.loads((ROOT/"artifacts/rl-campaign/p2.5-corrected-corridor-protocol.json").read_text());RADII=[2,3,4,5,6,8,10,12,15];DT=.05;H120=2400;H240=4800;SEED_FIRST=int(os.environ.get("RADIUS_SWEEP_SEED_FIRST","10000"));SEED_COUNT=int(os.environ.get("RADIUS_SWEEP_SEED_COUNT","50"));OUTPUT=ROOT/os.environ.get("RADIUS_SWEEP_OUTPUT","artifacts/rl-campaign/p3-local/terminal-radius-sweep.json")
def clamp(x,a,b):return min(b,max(a,x))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))
def episode(seed):
 r=Mulberry32(seed);u=lambda a,b:a+(b-a)*r.next();angle=math.radians(u(-20,20));start=[10000+u(-1,1),10000+u(-1,1)];rot=lambda p:[start[0]+p[0]*math.cos(angle)-p[1]*math.sin(angle),start[1]+p[0]*math.sin(angle)+p[1]*math.cos(angle)];speed=u(0,1);d=u(0,2*math.pi);wind=u(0,8);wd=u(0,2*math.pi);return start,math.radians(u(-10,10)),[rot(p) for p in ([20,0],[35,15],[50,5])],[speed*math.cos(d),speed*math.sin(d),0],[wind*math.cos(wd),wind*math.sin(wd),0]
rng=Mulberry32(7319);rand=lambda:2*rng.next()-1;W1=[[rand() for _ in range(9)]for _ in range(16)];b1=[rand()for _ in range(16)];W2=[[rand()for _ in range(16)]for _ in range(2)];b2=[rand()for _ in range(2)]
gains={"LOS-PID-v2":P25["tuning"]["los_pid"]["selected"]["gains"],"LOS-SPEEDCAP-v2":P25["tuning"]["los_mpc"]["selected"]["gains"],"frozen-untrained-policy-v1":{"lookahead":8,"kp":100,"kd":35,"speed":1.5}}
def geom(r,t):
 a=r["route"][r["wp"]-1]if r["wp"]else r["start"];b=r["route"][r["wp"]];dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy);cn,ce=dx/length,dy/length;n,e=t["position_ned_m"][:2];cross=-ce*(n-a[0])+cn*(e-a[1]);along=(n-a[0])*cn+(e-a[1])*ce;return math.hypot(b[0]-n,b[1]-e),cross,math.atan2(ce,cn),along>=length and abs(cross)<=PROTOCOL["advancement"]["plane_crossing_lateral_corridor_m"]
def action(r,t):
 distance,cross,heading,_=geom(r,t);g=gains[r["policy"]];yaw=t["attitude_rad"][2];u,v=t["velocity_body_mps"][:2];rate=t["angular_rate_body_rad_s"][2];err=wrap(heading-math.atan2(cross,g["lookahead"])-yaw)
 if r["policy"]=="frozen-untrained-policy-v1":
  obs=[clamp(distance/100,0,1),math.sin(err),math.cos(err),u/3,v/3,rate,r["prev"][0]/150,r["prev"][1]/100,1];h=[math.tanh(sum(w*x for w,x in zip(row,obs))+bias)for row,bias in zip(W1,b1)];return[150*math.tanh(sum(w*x for w,x in zip(W2[0],h))+b2[0]),100*math.tanh(sum(w*x for w,x in zip(W2[1],h))+b2[1])]
 speed=g["speed"]
 if r["policy"]=="LOS-SPEEDCAP-v2":speed=min(speed,math.sqrt(max(0,.8*distance)),speed*max(.25,math.cos(min(math.pi/2,abs(err)))))
 return[clamp(100*(speed-u),-150,150),clamp(g["kp"]*err-g["kd"]*rate,-100,100)]
specs=[];configs=[];runs=[]
for policy in ["frozen-untrained-policy-v1","LOS-PID-v2","LOS-SPEEDCAP-v2"]:
 for seed in range(SEED_FIRST,SEED_FIRST+SEED_COUNT):
  start,heading,route,current,wind=episode(seed);specs.append((policy,seed));configs.append({"schema_version":1,"experiment":{"name":f"sweep-{policy}-{seed}","seed":seed,"timestep_s":.05,"duration_s":240},"backend":{"type":"node"},"vehicle":{"preset":"vehicle-a-otter","plant":"planar3"},"environment":{"current_mps":current,"wind_mps":wind},"initial_state":{"position_ned_m":[*start,0],"attitude_rad":[0,0,heading]},"mission":{"type":"rl-common-waypoint-v1","waypoints":[{"north_m":19000,"east_m":19000}]},"sensors":[]});runs.append({"policy":policy,"seed":seed,"start":start,"route":route,"wp":0,"prev":[0.,0.],"final_step":None,"min120":float("inf"),"min240":float("inf"),"through120":{z:False for z in RADII},"through240":{z:False for z in RADII},"fullrun":{z:0 for z in RADII},"full120":{z:False for z in RADII},"full240":{z:False for z in RADII}})
with PersistentNodeBridge(ROOT)as bridge:
 bridge.reset(configs)
 for step in range(H240):
  truths=bridge.ground_truth_all();pre=[action(r,t)for r,t in zip(runs,truths)];bridge.step([{"actuators":{"desiredWrench":[a[0],0,0,0,0,a[1]]}}for a in pre]);truths=bridge.ground_truth_all()
  for r,t in zip(runs,truths):
   distance,cross,heading,passed=geom(r,t);r["prev"]=action(r,t)
   if r["wp"]<2 and(distance<=PROTOCOL["advancement"]["intermediate_radius_m"]or passed):r["wp"]+=1
   if r["wp"]==2:
    if r["final_step"]is None:r["final_step"]=step+1
    r["min240"]=min(r["min240"],distance)
    if step<H120:r["min120"]=min(r["min120"],distance)
    speed=math.hypot(*t["velocity_body_mps"][:2])
    for z in RADII:
     inside=distance<=z;r["fullrun"][z]=r["fullrun"][z]+1 if inside and speed<=.5 else 0
     if step<H120:
      if inside:r["through120"][z]=True
      if r["fullrun"][z]>=40:r["full120"][z]=True
     if inside:r["through240"][z]=True
     if r["fullrun"][z]>=40:r["full240"][z]=True
raw=[{"policy":r["policy"],"seed":r["seed"],"final_leg_time_s":None if r["final_step"]is None else r["final_step"]*DT,"closest_final_m_120":None if math.isinf(r["min120"]) else r["min120"],"closest_final_m_240":None if math.isinf(r["min240"]) else r["min240"],"pass_through_120":r["through120"],"full_120":r["full120"],"pass_through_240":r["through240"],"full_240":r["full240"]}for r in runs]
def rate(policy,key,z):return sum(x[key][z]for x in raw if x["policy"]==policy)/SEED_COUNT
cells=[]
for horizon in (120,240):
 for variant in ("pass_through","full"):
  key=f"{variant}_{horizon}"
  for z in RADII:
   u,p,m=rate("frozen-untrained-policy-v1",key,z),rate("LOS-PID-v2",key,z),rate("LOS-SPEEDCAP-v2",key,z);best=max(p,m);cells.append({"timeout_s":horizon,"variant":variant,"radius_m":z,"rates":{"untrained":u,"los_pid":p,"los_mpc":m,"best_classical":best},"normalized_success_gap":None if best==u else(best-u)/(best-u),"best_classical_in_40_70":.4<=best<=.7})
def qs(values):
 v=sorted(values);q=lambda p:v[int((len(v)-1)*p)];return{"count":len(v),"min":min(v),"q1":q(.25),"median":q(.5),"q3":q(.75),"q95":q(.95),"max":max(v)}
closest=[]
for policy in ("LOS-PID-v2","LOS-SPEEDCAP-v2"):
 r=[x for x in raw if x["policy"]==policy];closest.append({"policy":policy,"closest_final_m":{"timeout_120":qs([x["closest_final_m_120"]for x in r if x["closest_final_m_120"]is not None]),"timeout_240":qs([x["closest_final_m_240"]for x in r if x["closest_final_m_240"]is not None])}})
report={"schema_version":1,"artifact_kind":"terminal-radius-sweep","status":"complete-halt-before-contract-revision","provenance":{"new_training_performed":False,"new_rollouts_performed":False,"method":"Deterministic reconstruction of retained corridor-corrected trajectories; prior artifacts lacked per-step terminal traces."},"seed_set":{"first":SEED_FIRST,"count":SEED_COUNT},"source_corridor_p2_5_sha256":hashlib.sha256((ROOT/"artifacts/rl-campaign/p2.5-corrected-corridor.json").read_bytes()).hexdigest(),"radii_m":RADII,"raw":raw,"cells":cells,"closest_approach":closest,"selection_rule":"Smallest radius where best classical is 40-70%; prefer 120 s pass-through; do not select above 10 m.","decision":{"contract_revision_applied":False,"human_approval_required":True}}
path=OUTPUT;tmp=Path(str(path)+".tmp");tmp.write_text(json.dumps(report,indent=2)+"\n");tmp.replace(path);print(json.dumps({"seed_set":report["seed_set"],"qualifying":[x for x in cells if x["best_classical_in_40_70"]],"closest":closest},indent=2))
