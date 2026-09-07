import {mkdir,writeFile} from "node:fs/promises";
import {resolveExperiment} from "../../packages/experiment-schema/src/index.ts";
import {HeadlessMarineSimulation} from "../../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "../../backends/node/src/legacy-production-engine.ts";

const DT=.05,STEPS=300,ORIGIN=[10000,10000] as const,ANGLES=[-.35,-.25,-.15,-.08,0,.08,.15,.25,.35];
const root="artifacts/gazebo/vehicle-b/rudder-sweep";
const id=(angle:number)=>`${angle<0?"m":"p"}${Math.abs(angle).toFixed(2).replace(".","p")}`;

await mkdir(`${root}/schedules`,{recursive:true});await mkdir(`${root}/node`,{recursive:true});
for(const target of ANGLES){
 const config=resolveExperiment({schema_version:1,experiment:{name:`vehicle-b-rudder-sweep-${id(target)}`,seed:7319,timestep_s:DT,duration_s:STEPS*DT},backend:{type:"node"},vehicle:{preset:"vehicle-b-rudder",plant:"coupled6"},environment:{current_mps:[0,0,0],wind_mps:[0,0,0]},initial_state:{position_ned_m:[...ORIGIN,0],attitude_rad:[0,0,0]},mission:{type:"rl-common-waypoint-v1",waypoints:[{north_m:19000,east_m:19000}]}} as any);
 const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());sim.reset(config);const rows=[],samples=[];
 for(let step=0;step<STEPS;step++){
  const result:any=sim.step({actuators:{propeller_rps:15,rudder_rad:target}}),truth:any=sim.getGroundTruth(),diagnostics=result.info.vehicle_diagnostics;
  if(result.terminated||(result.truncated&&step<STEPS-1))throw new Error(`${id(target)} stopped at ${step}: ${String(result.info.stop_reason)}`);
  rows.push({step,time_s:step*DT,target_rudder_rad:target,applied_rudder_rad:diagnostics.applied_command.rudder_rad,applied_thrust_n:diagnostics.force_components.propeller[0],surge_speed_mps:truth.velocity_body_mps[0],yaw_rate_ned_rad_s:truth.angular_rate_body_rad_s[2],rudder_yaw_moment_nm:diagnostics.force_components.rudder[2]});
  samples.push({step,time_s:step*DT,thrust_n:diagnostics.force_components.propeller[0],steer_rad:diagnostics.applied_command.rudder_rad});
 }
 sim.dispose();
 await writeFile(`${root}/node/${id(target)}.json`,JSON.stringify({schema_version:1,target_rudder_rad:target,rows},null,2)+"\n");
 await writeFile(`${root}/schedules/${id(target)}.json`,JSON.stringify({schema_version:1,vehicle:"vehicle-b-rudder",case:`rudder-sweep-${id(target)}`,seed:7319,dt_s:DT,samples},null,2)+"\n");
}
