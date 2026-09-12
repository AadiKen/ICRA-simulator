from __future__ import annotations
import argparse,importlib,json,math
from pathlib import Path
from typing import Any,Callable
import numpy as np
from .adapters.base import PolicyAdapter
from .adapters.determinism_gate import load_replay,run_determinism_gate
from .contracts import TaskContract,load_contracts
from .judge.plant import AnalyticJudge,FossenParameters,PlantState
from .metrics import EpisodeMetrics,aggregate
from .preflight import verify_preflight
ARMS=("bcod-sim","holoocean","stonefish","gazebo")
def _load_factory(spec:str)->Callable[[],Any]:
 module,sep,name=spec.partition(":")
 if not sep:raise ValueError("factory must be module:callable")
 return getattr(importlib.import_module(module),name)
def _sample(value,rng):return float(rng.uniform(*value)) if isinstance(value,list) else float(value)
def _cross_track(point,a,b):
 p=np.asarray(point);a=np.asarray(a);d=np.asarray(b)-a;q=np.clip(np.dot(p-a,d)/np.dot(d,d),0,1);return float(np.linalg.norm(p-(a+q*d)))
def run_episode(judge:AnalyticJudge,adapter:PolicyAdapter,policy:Any,contract:TaskContract,seed:int)->dict[str,Any]:
 d=contract.document;rng=np.random.default_rng(seed);route=d["route"]["waypoints_local_m"];termination={**d["termination"],**d.get("difficulty",{})};vp=d["vehicle_parameters"]
 params=FossenParameters(max_thrust_n=adapter.force_ceiling_n,actuator_tau_s=adapter.actuator_tau_s).perturbed(_sample(vp["mass_scale"],rng),_sample(vp["drag_scale"],rng),_sample(vp["thrust_scale"],rng));judge.reset(PlantState(north_m=route[0][0],east_m=route[0][1]),params)
 disturbance=d["disturbance"];current_speed=_sample(disturbance.get("current_mps",0),rng) if not isinstance(disturbance.get("current_mps"),str) else 0.;ca=float(rng.uniform(-math.pi,math.pi));wind_speed=_sample(disturbance.get("wind_mps",0),rng);wa=float(rng.uniform(-math.pi,math.pi));current=(current_speed*math.cos(ca),current_speed*math.sin(ca));wind=(wind_speed*math.cos(wa),wind_speed*math.sin(wa))
 goals=route[1:];goal_index=0;previous=np.zeros(2);cross=[];energy=0.;violations=0;dt=judge.dt_s;control_steps=max(1,round(.1/dt));cap=float(termination["time_cap_s"]);max_steps=math.ceil(cap/dt);obstacles=d.get("obstacles",[]);degraded=bool(d.get("sensing",{}).get("degraded"));success=False
 for step in range(max_steps):
  if step%control_steps==0:
   obs=judge.observation(goals[goal_index],previous,1-step/max_steps,fix_valid=0. if degraded and rng.random()<.1 else 1.)
   if degraded:obs[:7]+=rng.normal(0,.02,7);obs[7:9]+=rng.normal(0,1.2,2)
   previous=adapter.policy_step(policy,obs)
  state=judge.step(previous,current,wind);energy+=dt*(abs(state.port_thrust_n)+abs(state.starboard_thrust_n));a=route[goal_index];b=goals[goal_index];cross.append(_cross_track((state.north_m,state.east_m),a,b))
  for obstacle in obstacles:
   margin=math.hypot(state.north_m-obstacle["north_m"],state.east_m-obstacle["east_m"])-obstacle["radius_m"]
   if margin<obstacle.get("safety_margin_m",0):violations+=1
  if math.hypot(state.north_m-b[0],state.east_m-b[1])<=float(termination["success_radius_m"]):
   if goal_index==len(goals)-1:success=energy<=float(termination["energy_cap_ns"]);break
   goal_index+=1
 return {"success":success,"cross_track_error_m":float(np.mean(cross)),"completion_time_s":judge.state.time_s,"propulsion_cost_ns":energy,"safety_violations":violations}
def run_suite(*,contracts:list[TaskContract],adapters:dict[str,PolicyAdapter],policies:dict[str,Any],training:dict[str,dict[str,float]],preflight:dict[str,Any],judge_validation:dict[str,Any],bootstrap_samples:int=10_000,preflight_authorization:dict[str,Any]|None=None)->dict[str,Any]:
 if preflight_authorization is None:raise RuntimeError("A passing content-hashed preflight authorization artifact is required")
 verify_preflight(preflight_authorization)
 unfrozen=[c.condition_id for c in contracts if c.document.get("status")!="frozen"]
 if unfrozen:raise RuntimeError("Calibration must freeze task contracts before evaluation: "+", ".join(unfrozen))
 if not judge_validation.get("passed"):raise RuntimeError("Judge validation gate has not passed")
 steps={int(training[arm]["steps"]) for arm in ARMS}
 if len(steps)!=1:raise RuntimeError("Common-suite sample-efficiency evaluation requires exactly matched training steps")
 failed=[arm for arm in ARMS if not preflight.get(arm,{}).get("passed")]
 if failed:raise RuntimeError("Adapter determinism gate blocks arms: "+", ".join(failed))
 gaps={row["arm"]:float(row["domain_gap"]) for row in judge_validation["arms"]};rows=[];skipped=[]
 for arm in ARMS:
  adapter=adapters[arm]
  for contract in contracts:
   if contract.requires_sensors and not adapter.consumes_sensors:skipped.append({"arm":arm,"condition":contract.condition_id,"reason":"purely kinematic policy"});continue
   for seed in contract.seeds:
    result=run_episode(AnalyticJudge(),adapter,policies[arm],contract,seed);rows.append(EpisodeMetrics(arm,contract.condition_id,contract.content_sha256,seed,bool(result["success"]),float(result["cross_track_error_m"]),float(result["completion_time_s"]),float(result["propulsion_cost_ns"]),int(result["safety_violations"]),int(training[arm]["steps"]),float(training[arm]["wall_clock_s"]),gaps[arm]))
 return {"schema_version":2,"artifact_kind":"cross-simulator-analytic-judge-suite","claim_scope":"transfer performance to a validated-but-imperfect analytic plant under matched conditions","arms":list(ARMS),"judge_validation":judge_validation,"preflight":preflight,"skipped":skipped,"episodes":[r.to_dict() for r in rows],"aggregates":aggregate(rows,bootstrap_samples=bootstrap_samples)}
def main():
 p=argparse.ArgumentParser();p.add_argument("--config",type=Path,required=True);p.add_argument("--output",type=Path,required=True);p.add_argument("--bootstrap-samples",type=int,default=10_000);a=p.parse_args();cfg=json.loads(a.config.read_text());authorization=json.loads(Path(cfg["preflight_authorization"]).read_text());verify_preflight(authorization);contracts=load_contracts(Path(cfg.get("contract_dir",Path(__file__).with_name("contracts"))));adapters={arm:_load_factory(cfg["adapter_factories"][arm])() for arm in ARMS};policies={arm:adapters[arm].load_policy(cfg["checkpoints"][arm]) for arm in ARMS};preflight={}
 for arm in ARMS:
  result=run_determinism_gate(adapters[arm],load_replay(Path(cfg["native_replays"][arm])));preflight[arm]=result.to_dict();print(f"adapter-preflight {arm}: {'PASS' if result.passed else 'FAIL'} ({result.notes}; determinism only, not fidelity)")
 report=run_suite(contracts=contracts,adapters=adapters,policies=policies,training=cfg["training"],preflight=preflight,judge_validation=json.loads(Path(cfg["judge_validation"]).read_text()),bootstrap_samples=a.bootstrap_samples,preflight_authorization=authorization);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n")
if __name__=="__main__":main()
