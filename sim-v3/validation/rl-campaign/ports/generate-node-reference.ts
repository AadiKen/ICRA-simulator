import {resolveExperiment} from "../../../packages/experiment-schema/src/index.ts";
import {HeadlessMarineSimulation} from "../../../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "../../../backends/node/src/legacy-production-engine.ts";
import {assembleTrace,frozenReset,writeTrace,FULL_STEPS,PHYSICS_DT} from "./episode-driver.ts";
import {commandAt} from "./frozen-conformance-trace.ts";
import type {OdomSample} from "./task-trace-bridge.ts";

export function generateNodeReference(seed:number,out:string,steps=FULL_STEPS,scales={wind:1,current:1,action:1}){
 const reset=frozenReset(seed),wind=reset.disturbance.wind_direction_deg*Math.PI/180,current=reset.disturbance.current_direction_deg*Math.PI/180;
 const config=resolveExperiment({schema_version:1,experiment:{name:`gate-7-reference-${seed}`,seed,timestep_s:PHYSICS_DT,duration_s:steps*PHYSICS_DT+.1},backend:{type:"node"},vehicle:{preset:"searobotics-surveyor-m1.8",plant:"planar3"},environment:{wind_mps:[scales.wind*reset.disturbance.wind_speed_m_s*Math.cos(wind),scales.wind*reset.disturbance.wind_speed_m_s*Math.sin(wind),0],current_mps:[scales.current*reset.disturbance.current_speed_m_s*Math.cos(current),scales.current*reset.disturbance.current_speed_m_s*Math.sin(current),0]},initial_state:{position_ned_m:[reset.initial_state[0],reset.initial_state[1],0],attitude_rad:[0,0,reset.initial_state[2]]},mission:{type:"rl-common-waypoint-v1",waypoints:reset.route_ned_m.map(([north_m,east_m])=>({north_m,east_m}))},sensors:[]} as any);
 const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());sim.reset(config);const odom:OdomSample[]=[];
 // The reference receives the same normalized twin-thruster command every
 // physics step.  Its internal actuator state remains the Node source of truth.
 for(let step=0;step<steps;step++){const a=commandAt(Math.floor(step/2)*2);sim.step({actuators:{effectors:{port:{command:scales.action*a[0]},starboard:{command:scales.action*a[1]}}}});const g:any=sim.getGroundTruth();odom.push({time_s:step*PHYSICS_DT,N_m:g.position_ned_m[0],E_m:g.position_ned_m[1],yaw_rad:g.attitude_rad[2],u_mps:g.velocity_body_mps[0],v_mps:g.velocity_body_mps[1],r_rad_s:g.angular_rate_body_rad_s[2],imu_linear_accel_body:g.acceleration_body_mps2,imu_angular_rate_body:g.angular_rate_body_rad_s,gps_fix_valid:1});}
 sim.dispose();const trace=assembleTrace("Gazebo Harmonic",seed,odom,steps);writeTrace(out,trace);return trace;
}
if(process.argv[1]?.endsWith("generate-node-reference.ts")){const [seed,out]=process.argv.slice(2);if(!seed||!out)throw new Error("usage: generate-node-reference.ts seed output.json");generateNodeReference(Number(seed),out);}
