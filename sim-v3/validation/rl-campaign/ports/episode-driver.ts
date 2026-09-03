/**
 * Contract-side episode preparation and trace assembly shared by the two
 * external runners.  Deliberately contains no process handling: Gazebo uses
 * captureGazeboLog's lifecycle and VRX is launched by its checked-in launch
 * file.  This prevents a second, subtly different, simulator lifecycle.
 */
import {readFileSync,writeFileSync,mkdirSync} from "node:fs";
import {dirname} from "node:path";
import {fixedActionTrace} from "./frozen-conformance-trace.ts";
import {TaskTraceBridge,type OdomSample,type TaskReset} from "./task-trace-bridge.ts";
import {assertTraceV2,type TraceV2} from "./trace-schema-v2.ts";
import {GazeboThrusterAdapter,VrxWamvThrusterAdapter} from "./simulator-actuator-adapters.ts";

export const PHYSICS_DT=.05, CONTROL_DT=.1, FULL_STEPS=2400;
type Simulator="Gazebo Harmonic"|"VRX";
const unit=(seed:number)=>{let x=seed>>>0;x^=x<<13;x^=x>>>17;x^=x<<5;return (x>>>0)/0x1_0000_0000};
export function frozenReset(seed:number):TaskReset {
  const a=unit(seed),b=unit(seed+1),c=unit(seed+2),d=unit(seed+3),e=unit(seed+4);
  const angle=(-20+40*a)*Math.PI/180, n=10000+(-1+2*b), east=10000+(-1+2*c), ca=Math.cos(angle),sa=Math.sin(angle);
  const route=[[20,0],[35,15],[50,5]].map(([x,y])=>[n+x*ca-y*sa,east+x*sa+y*ca] as [number,number]);
  return {seed,initial_state:[n,east,(-10+20*d)*Math.PI/180,0,0,0],route_ned_m:route,disturbance:{wind_speed_m_s:8*e,wind_direction_deg:360*unit(seed+5),current_speed_m_s:unit(seed+6),current_direction_deg:360*unit(seed+7)}};
}
export function prepareEpisode(simulator:Simulator,seed:number,steps=FULL_STEPS,actionScale=1) {
  const actions=fixedActionTrace(steps); const adapter=simulator==="Gazebo Harmonic"?new GazeboThrusterAdapter():new VrxWamvThrusterAdapter();
  // Apply lag once per physics step; each control action is held for two steps.
  const transport=Array.from({length:steps},(_,step)=>{
    const command=actions[Math.floor(step/2)].command.map((value,index)=>index<2?actionScale*value:value) as [number,number,number,number];
    return adapter.apply(command,PHYSICS_DT);
  });
  return {reset:frozenReset(seed),actions,transport,actionScale,inertia_provenance:"geometry-derived bootstrap craft mass and geometry"};
}
/** Assemble a schema trace from simulator true-odometry rows.  `odom` must be
 * already frame-normalized by the simulator adapter; it is never pose-diffed. */
export function assembleTrace(simulator:Simulator,seed:number,odom:OdomSample[],steps=FULL_STEPS):TraceV2 {
  if(odom.length!==steps)throw new Error(`Expected ${steps} true-odometry samples; received ${odom.length}.`);
  const episode=prepareEpisode(simulator,seed,steps),bridge=new TaskTraceBridge(simulator,episode.reset); bridge.resetBridge();
  const samples=odom.map((sample,step)=>{bridge.apply(episode.actions[Math.floor(step/2)].command,PHYSICS_DT);return bridge.sample(step,{...sample,time_s:step*PHYSICS_DT});});
  const trace=bridge.trace(episode.actions,samples);assertTraceV2(trace);return trace;
}
export function writeTrace(path:string,trace:TraceV2){assertTraceV2(trace);mkdirSync(dirname(path),{recursive:true});writeFileSync(path,JSON.stringify(trace,null,2)+"\n");}
/** Convert the Jazzy exporter JSONL.  The exporter supplies odometry velocity,
 * IMU and GPS; no state field is reconstructed from pose differences. */
export function assembleVrxExporterTrace(seed:number,input:string,out:string,steps=FULL_STEPS){
 const rows=readFileSync(input,"utf8").trim().split(/\n+/).filter(Boolean).map(x=>JSON.parse(x));
 const odom:OdomSample[]=rows.slice(0,steps).map((r:any,i:number)=>({time_s:i*PHYSICS_DT,N_m:r.state[0],E_m:r.state[1],yaw_rad:r.state[2],u_mps:r.state[3],v_mps:r.state[4],r_rad_s:r.state[5],imu_linear_accel_body:r.imu?.slice(0,3),imu_angular_rate_body:r.imu?.slice(3,6),gps_fix_valid:r.gps_fix_valid}));
 const trace=assembleTrace("VRX",seed,odom,steps);writeTrace(out,trace);return trace;
}
