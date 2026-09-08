import {readFileSync} from "node:fs";
import {SeededRandom} from "../../packages/core/src/random.ts";
import {allocatePlanarAzimuthMinimumNorm} from "../../packages/core/src/azimuth-allocation.js";
import {createVehicleCWorkingController,VEHICLE_C_WORKING_CONTROLLER} from "../../backends/node/src/vehicle-c-sliding-mode-controller.ts";
import {buildVehicleCProductionConfiguration} from "../../backends/node/src/vehicle-c-production.ts";

export const DT=.05,CONTROL_DT=.1,HOLD=Math.round(CONTROL_DT/DT),MAX_STEPS=2400,SEED=30013,TERMINAL_RADIUS=2;
export const PHYSICS_MISMATCH="Gazebo Vehicle C has no Coriolis coupling or added-mass dynamics and excludes pod-hull, pod-pod, and angle-dependent thrust interaction. The unchanged controller still applies bcod-sim-tuned C(v) compensation; this is a navigation-outcome comparison against a simpler model, not a matched-physics test.";
const contract=JSON.parse(readFileSync(new URL("../../artifacts/rl-campaign/surveyor/task-contract-frozen.json",import.meta.url),"utf8")).tasks.find((x:any)=>x.task_id==="common-waypoint-transit-v1");

export function scenario(seed=SEED){
  const r=new SeededRandom(seed),u=(a:number,b:number)=>a+(b-a)*r.next(),rr=contract.reset_randomization,angle=u(...rr.route_rotation_deg)*Math.PI/180,start:[number,number]=[10000+u(...rr.start_position_offset_m),10000+u(...rr.start_position_offset_m)],rot=([n,e]:number[]):[number,number]=>[start[0]+n*Math.cos(angle)-e*Math.sin(angle),start[1]+n*Math.sin(angle)+e*Math.cos(angle)],cs=u(...rr.current_speed_m_s),cd=u(0,2*Math.PI),ws=u(...rr.wind_speed_m_s),wd=u(0,2*Math.PI);
  return{seed,start_ned_m:start,initial_heading_rad:u(...rr.start_heading_deg)*Math.PI/180,route_ned_m:rr.route_relative_m.map(rot),current_ned_mps:[cs*Math.cos(cd),cs*Math.sin(cd),0] as [number,number,number],wind_ned_mps:[ws*Math.cos(wd),ws*Math.sin(wd),0] as [number,number,number]};
}

export function geometry(run:{waypoint:number;scenario:ReturnType<typeof scenario>},state:any){
  const a=run.waypoint?run.scenario.route_ned_m[run.waypoint-1]:run.scenario.start_ned_m,b=run.scenario.route_ned_m[run.waypoint],dn=b[0]-a[0],de=b[1]-a[1],length=Math.hypot(dn,de),cn=dn/length,ce=de/length,n=state.position_ned_m[0],e=state.position_ned_m[1],along=(n-a[0])*cn+(e-a[1])*ce,cross=-ce*(n-a[0])+cn*(e-a[1]);
  return{a,b,distance:Math.hypot(b[0]-n,b[1]-e),heading:Math.atan2(de,dn),passed:along>=length&&Math.abs(cross)<=15.361124064575238};
}

export function makeControl(){
  const controller=createVehicleCWorkingController(CONTROL_DT),production=buildVehicleCProductionConfiguration(),placements=production.parameters.effectors.map((x:any)=>({id:x.id,pos:x.pos,maxThrust:x.maxForwardThrust,maxReverseThrust:x.maxReverseThrust}));
  return{controller,command(state:any,target:any,current:[number,number,number]){const out=controller.action(state,target,current),allocation=allocatePlanarAzimuthMinimumNorm([out.desired_wrench[0],out.desired_wrench[1],out.desired_wrench[5]],placements,{regularization:0,angleContinuityWeight:0});return{controller:out,allocation,port:allocation.pods.find((x:any)=>x.id.includes("port")),starboard:allocation.pods.find((x:any)=>x.id.includes("starboard"))};}};
}

export const controllerMetadata={...VEHICLE_C_WORKING_CONTROLLER,control_interval_s:CONTROL_DT,physics_timestep_s:DT};
