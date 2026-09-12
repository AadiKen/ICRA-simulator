import {mkdir,writeFile} from "node:fs/promises";
import {resolveExperiment} from "../../packages/experiment-schema/src/index.ts";
import {HeadlessMarineSimulation} from "../../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "../../backends/node/src/legacy-production-engine.ts";

const DT=Number(process.argv[2]??.01),THRUST_FRACTION=Number(process.argv[3]??.7),DURATION_S=6,STEPS=Math.round(DURATION_S/DT),ORIGIN=[10000,10000] as const;
if(![.01,.05].includes(DT))throw Error("Diagnostic dt must be 0.01 or 0.05 s");
if(!(THRUST_FRACTION>0&&THRUST_FRACTION<=1))throw Error("Thrust fraction must be in (0, 1]");
const ROOT=`artifacts/gazebo/vehicle-c/timestep-diagnostic/dt-${DT.toFixed(2)}-thrust-${THRUST_FRACTION.toFixed(2)}`,TARGET=THRUST_FRACTION*1340.506075016061;
const cfg=resolveExperiment({schema_version:1,experiment:{name:`vehicle-c-straight-ahead-dt-${DT}`,seed:7319,timestep_s:DT,duration_s:DURATION_S},backend:{type:"node"},vehicle:{preset:"vehicle-c-azimuth",plant:"coupled6"},environment:{current_mps:[0,0,0],wind_mps:[0,0,0]},initial_state:{position_ned_m:[...ORIGIN,0],attitude_rad:[0,0,0]},mission:{type:"rl-common-waypoint-v1",waypoints:[{north_m:19000,east_m:19000}]}} as any);
const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());sim.reset(cfg);const rows=[],samples=[];
for(let step=0;step<STEPS;step++){const desired={thrust:TARGET,azimuth:0},result:any=sim.step({actuators:{effectors:{"azimuth-port":desired,"azimuth-starboard":desired}}}),truth:any=sim.getGroundTruth(),d=result.info.vehicle_diagnostics,port=d.effectors.find((x:any)=>x.id==="azimuth-port"),starboard=d.effectors.find((x:any)=>x.id==="azimuth-starboard");rows.push({step,time_s:(step+1)*DT,north_m:truth.position_ned_m[0]-ORIGIN[0],east_m:truth.position_ned_m[1]-ORIGIN[1],heading_rad:truth.attitude_rad[2],u_mps:truth.velocity_body_mps[0],v_mps:truth.velocity_body_mps[1],r_rad_s:truth.angular_rate_body_rad_s[2]});samples.push({step,time_s:step*DT,port_commanded_thrust_n:TARGET,port_commanded_azimuth_rad:0,port_thrust_n:port.thrust,port_azimuth_rad:port.azimuth,starboard_commanded_thrust_n:TARGET,starboard_commanded_azimuth_rad:0,starboard_thrust_n:starboard.thrust,starboard_azimuth_rad:starboard.azimuth});}sim.dispose();
await mkdir(ROOT,{recursive:true});await writeFile(`${ROOT}/bcod-sim.json`,JSON.stringify({schema_version:1,dt_s:DT,rows},null,2)+"\n");await writeFile(`${ROOT}/schedule.json`,JSON.stringify({schema_version:1,dt_s:DT,samples},null,2)+"\n");
