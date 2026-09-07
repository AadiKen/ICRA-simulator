import {createHash} from "node:crypto";
import {mkdir,writeFile} from "node:fs/promises";
import {resolve} from "node:path";
import {resolveExperiment} from "../../packages/experiment-schema/src/index.ts";
import {HeadlessMarineSimulation} from "../../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "../../backends/node/src/legacy-production-engine.ts";

const DT=.05,STEPS=120,OUT=resolve("artifacts/gazebo/vehicle-b"),ORIGIN_NED=[50,50] as const;
const cases={
  "straight-step":(step:number)=>({propeller_rps:15,rudder_rad:0}),
  "reverse-step":(step:number)=>({propeller_rps:-15,rudder_rad:0}),
  "positive-rudder-step":(step:number)=>({propeller_rps:15,rudder_rad:.2}),
  "negative-rudder-step":(step:number)=>({propeller_rps:15,rudder_rad:-.2}),
  "chirp-step":(step:number)=>step<40?{propeller_rps:15,rudder_rad:0}:step<80?{propeller_rps:18,rudder_rad:.2}:{propeller_rps:10,rudder_rad:-.15},
} as const;
const sha=(value:string)=>createHash("sha256").update(value).digest("hex");

async function main(){
  await mkdir(`${OUT}/node-reference`,{recursive:true});await mkdir(`${OUT}/schedules`,{recursive:true});
  for(const [id,commandAt] of Object.entries(cases)){
    const config=resolveExperiment({schema_version:1,experiment:{name:`gazebo-b-${id}`,seed:7319,timestep_s:DT,duration_s:STEPS*DT},backend:{type:"node"},vehicle:{preset:"vehicle-b-rudder",plant:"coupled6"},environment:{current_mps:[0,0,0],wind_mps:[0,0,0]},initial_state:{position_ned_m:[...ORIGIN_NED,0],attitude_rad:[0,0,0]},mission:{type:"production-integration"}} as any);
    const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());sim.reset(config);const rows=[],samples=[];
    for(let step=0;step<STEPS;step++){
      const command=commandAt(step),result:any=sim.step({actuators:command}),truth:any=sim.getGroundTruth(),diagnostics=result.info.vehicle_diagnostics;
      if(result.terminated||(result.truncated&&step<STEPS-1))throw new Error(`${id} stopped at step ${step}: ${String(result.info.stop_reason)}`);
      const propeller=diagnostics.force_components.propeller as number[];
      rows.push({step,time_s:step*DT,state:{N_m:truth.position_ned_m[0]-ORIGIN_NED[0],E_m:truth.position_ned_m[1]-ORIGIN_NED[1],yaw_rad:truth.attitude_rad[2],u_mps:truth.velocity_body_mps[0],v_mps:truth.velocity_body_mps[1],r_rad_s:truth.angular_rate_body_rad_s[2]},command,applied_command:diagnostics.applied_command,applied_propeller_thrust_n:propeller[0]});
      samples.push({step,time_s:step*DT,thrust_n:propeller[0],steer_rad:diagnostics.applied_command.rudder_rad});
    }
    sim.dispose();
    const schedule={schema_version:1,vehicle:"vehicle-b-rudder",case:id,seed:7319,dt_s:DT,samples};
    const reference={schema_version:1,artifact_kind:"bcod-sim-open-loop-reference",vehicle:"vehicle-b-rudder",case:id,seed:7319,dt_s:DT,rows,trace_sha256:sha(JSON.stringify(rows)),claim_limit:"Node-side reference for cross-implementation consistency only; Vehicle B validation status is unchanged."};
    await writeFile(`${OUT}/schedules/${id}.json`,JSON.stringify(schedule,null,2)+"\n");
    await writeFile(`${OUT}/node-reference/${id}.json`,JSON.stringify(reference,null,2)+"\n");
  }
}
await main();
