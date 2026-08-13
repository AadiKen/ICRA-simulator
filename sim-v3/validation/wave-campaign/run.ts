import {createHash} from "node:crypto";
import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {dirname,resolve} from "node:path";
import {fileURLToPath} from "node:url";
import {CoupledSixPlant} from "../../core/coupledSixPlant.js";
import {RigidBodyState} from "../../core/rigidBodyState.js";
import {WaveExcitation} from "../../packages/core/src/wave-forces.js";
import {interpolateWaveExcitation} from "../../packages/core/src/hydrodynamics.ts";
import {buildVehicleBProductionConfiguration} from "../../backends/node/src/vehicle-b-production.ts";
import {buildVehicleCProductionConfiguration} from "../../backends/node/src/vehicle-c-production.ts";

const sha=(value:string|Buffer)=>createHash("sha256").update(value).digest("hex");
const rms=(values:number[])=>Math.sqrt(values.reduce((sum,value)=>sum+value*value,0)/values.length);

function runVehicle(id:"vehicle-b-rudder"|"vehicle-c-azimuth",sea:"head"|"beam"){
  const production=id==="vehicle-b-rudder"?buildVehicleBProductionConfiguration():buildVehicleCProductionConfiguration();
  const source=JSON.parse(readFileSync(resolve(production.definition.potential_flow!.artifact_path),"utf8"));
  const heading=sea==="head"?0:-Math.PI/2;
  production.parameters.hydrodynamics.wave_excitation=interpolateWaveExcitation(source.wave_excitation,production.parameters.hydrodynamics.evaluation_frequency_rad_s,heading);
  const plant=new CoupledSixPlant(production.parameters,[new WaveExcitation()],"rk4"),state=RigidBodyState.fromEuler({N:0,E:0,D:0},0,0,0);
  const dt=.01,steps=6000,amplitude=.1,trace=[] as any[];
  for(let step=0;step<steps;step++){
    plant.step(state,{waveAmplitudeM:amplitude,wavePhaseRad:0},{},dt,step*dt);
    const e=state.eulerAngles,row={step,time_s:(step+1)*dt,north_m:state.position.N,east_m:state.position.E,down_m:state.position.D,roll_rad:e.roll,pitch_rad:e.pitch,yaw_rad:e.yaw,u_mps:state.velocity.u,v_mps:state.velocity.v,w_mps:state.velocity.w,p_rad_s:state.angularRate.p,q_rad_s:state.angularRate.q,r_rad_s:state.angularRate.r};
    if(Object.values(row).some((value)=>typeof value==="number"&&!Number.isFinite(value)))throw new Error(`${id} regular-wave response became non-finite at step ${step}`);
    trace.push(row);
  }
  const steady=trace.slice(2000),field=(name:string)=>steady.map((row)=>row[name]);
  return{vehicle_id:id,sea,heading_rad:heading,validation_status:production.definition.validation.status,plant:"coupled6",out_of_plane_dofs_free:true,coefficient_source:production.definition.potential_flow?.artifact_path,coefficient_status:production.hydrodynamics.status,episode:{duration_s:steps*dt,dt_s:dt,steps,wave_amplitude_m:amplitude,wave_period_s:2*Math.PI/production.parameters.hydrodynamics.wave_excitation.omega_rad_s,evaluation_frequency_rad_s:production.parameters.hydrodynamics.evaluation_frequency_rad_s,selection_method:production.parameters.hydrodynamics.selection_method,approximation:production.parameters.hydrodynamics.approximation},metrics:{steady_state_rms:{heave_m:rms(field("down_m")),roll_rad:rms(field("roll_rad")),pitch_rad:rms(field("pitch_rad"))},max_abs:{heave_m:Math.max(...field("down_m").map(Math.abs)),roll_rad:Math.max(...field("roll_rad").map(Math.abs)),pitch_rad:Math.max(...field("pitch_rad").map(Math.abs))},finite:true},trace_rows:trace.length,trace_checksum_sha256:sha(JSON.stringify(trace)),trace_retention:"checksum-and-metrics-only"};
}

export function runCampaign(){const vehicles=(["vehicle-b-rudder","vehicle-c-azimuth"] as const).flatMap((id)=>[runVehicle(id,"head"),runVehicle(id,"beam")]);return{schema_version:1,artifact_kind:"parametric-hull-regular-wave-response",status:"software-campaign-passed-physical-validation-blocked",is_physical_validation_evidence:false,vehicles,claim_limit:"Demonstrates deterministic bounded head- and beam-wave execution through production coupled6 with parametric Capytaine coefficients and the zero-speed viscous lower bound. It is not agreement with measured wave-response data.",limitations:["Constant coefficients omit Cummins radiation memory.","Encounter frequency is fixed for the episode.","Representative hull geometry and viscous damping are unvalidated design estimates.","No measured RAO or regular-wave trajectory is available."]};}

if(process.argv[1]===fileURLToPath(import.meta.url)){const output=resolve(process.argv[2]??"artifacts/wave-response/parametric-coupled6.json"),artifact=runCampaign();mkdirSync(dirname(output),{recursive:true});writeFileSync(output,`${JSON.stringify(artifact,null,2)}\n`);console.log(JSON.stringify({output,status:artifact.status,vehicles:artifact.vehicles.map(({vehicle_id,metrics,trace_checksum_sha256})=>({vehicle_id,metrics,trace_checksum_sha256}))},null,2));}
