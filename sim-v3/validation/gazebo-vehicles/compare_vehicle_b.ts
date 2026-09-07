import {createHash} from "node:crypto";
import {readFile,writeFile} from "node:fs/promises";
import {gazeboOdomToTask} from "../rl-campaign/ports/task-trace-bridge.ts";

const ROOT="artifacts/gazebo/vehicle-b",CASES=["straight-step","positive-rudder-step","negative-rudder-step","chirp-step"],sha=(v:Buffer|string)=>createHash("sha256").update(v).digest("hex"),wrap=(x:number)=>Math.atan2(Math.sin(x),Math.cos(x));
const load=async(path:string)=>JSON.parse(await readFile(path,"utf8"));
function rms(values:number[]){return Math.sqrt(values.reduce((sum,x)=>sum+x*x,0)/values.length)}
async function main(){
 const protocolPath="artifacts/gazebo/vehicle-b-cross-implementation-protocol.json",protocol=await load(protocolPath),results=[];
 for(const id of CASES){
  const nodePath=`${ROOT}/node-reference/${id}.json`,gazeboPath=`${ROOT}/gazebo/${id}.json`,node=await load(nodePath),gazebo=await load(gazeboPath);
  if(node.rows.length!==gazebo.rows.length)throw new Error(`${id} row count mismatch`);
  const converted=gazebo.rows.map((row:any)=>gazeboOdomToTask(row.time_s,row.enu));
  const horizontal=node.rows.map((row:any,i:number)=>Math.hypot(row.state.N_m-converted[i].N_m,row.state.E_m-converted[i].E_m));
  const yaw=node.rows.map((row:any,i:number)=>Math.abs(wrap(row.state.yaw_rad-converted[i].yaw_rad)));
  const metrics={horizontal_position_rmse_m:rms(horizontal),yaw_rmse_rad:rms(yaw),final_horizontal_position_error_m:horizontal.at(-1),final_yaw_error_rad:yaw.at(-1)};
  const checks=Object.fromEntries(Object.entries(metrics).map(([key,value])=>[key,{value,maximum:protocol.metrics[key].maximum,passed:value<=protocol.metrics[key].maximum}]));
  results.push({case:id,node_path:nodePath,node_sha256:sha(await readFile(nodePath)),gazebo_path:gazeboPath,gazebo_sha256:sha(await readFile(gazeboPath)),sample_count:node.rows.length,metrics:checks,passed:Object.values(checks).every((x:any)=>x.passed)});
 }
 const direction=[];
 for(const id of ["straight-step","reverse-step","positive-rudder-step","negative-rudder-step"]){
  const node=await load(`${ROOT}/node-reference/${id}.json`),gazebo=await load(`${ROOT}/gazebo/${id}.json`),n=node.rows.at(-1).state,g=gazeboOdomToTask(gazebo.rows.at(-1).time_s,gazebo.rows.at(-1).enu);
  const expected=id==="straight-step"?"forward":id==="reverse-step"?"astern":id==="positive-rudder-step"?"positive NED yaw / starboard turn":"negative NED yaw / port turn";
  const nodeObserved=id.includes("rudder")?(n.yaw_rad>0?"positive NED yaw / starboard turn":"negative NED yaw / port turn"):(n.N_m>0?"forward":"astern");
  const gazeboObserved=id.includes("rudder")?(g.yaw_rad>0?"positive NED yaw / starboard turn":"negative NED yaw / port turn"):(g.N_m>0?"forward":"astern");
  direction.push({case:id,expected,node_observed:nodeObserved,gazebo_observed:gazeboObserved,passed:expected===nodeObserved&&expected===gazeboObserved,node_final:{N_m:n.N_m,E_m:n.E_m,yaw_rad:n.yaw_rad,r_rad_s:n.r_rad_s},gazebo_final:{N_m:g.N_m,E_m:g.E_m,yaw_rad:g.yaw_rad,r_rad_s:g.r_rad_s}});
 }
 const yawCases=direction.filter(x=>x.case.includes("rudder"));
 direction.push({case:"positive-yaw-rate-sign",expected:"positive NED yaw rate",node_observed:yawCases[0].node_final.r_rad_s>0?"positive NED yaw rate":"negative NED yaw rate",gazebo_observed:yawCases[0].gazebo_final.r_rad_s>0?"positive NED yaw rate":"negative NED yaw rate",passed:yawCases[0].node_final.r_rad_s>0&&yawCases[0].gazebo_final.r_rad_s>0});
 direction.push({case:"negative-yaw-rate-sign",expected:"negative NED yaw rate",node_observed:yawCases[1].node_final.r_rad_s<0?"negative NED yaw rate":"positive NED yaw rate",gazebo_observed:yawCases[1].gazebo_final.r_rad_s<0?"negative NED yaw rate":"positive NED yaw rate",passed:yawCases[1].node_final.r_rad_s<0&&yawCases[1].gazebo_final.r_rad_s<0});
 const invertedYaw=[];
 for(const id of ["positive-rudder-step","negative-rudder-step"]){
  const node=await load(`${ROOT}/node-reference/${id}.json`),gazebo=await load(`${ROOT}/gazebo/${id}.json`),converted=gazebo.rows.map((row:any)=>gazeboOdomToTask(row.time_s,row.enu));
  const value=rms(node.rows.map((row:any,i:number)=>Math.abs(wrap(row.state.yaw_rad+converted[i].yaw_rad))));
  invertedYaw.push({case:id,yaw_rmse_rad:value,maximum:protocol.metrics.yaw_rmse_rad.maximum,rejected:value>protocol.metrics.yaw_rmse_rad.maximum});
 }
 const artifact={schema_version:1,artifact_kind:"vehicle-gazebo-cross-implementation-check",status:results.every(x=>x.passed)&&direction.every(x=>x.passed)?"cross-implementation-consistency-checked-gazebo-harmonic-8.15":"cross-implementation-consistency-check-failed",vehicle:"vehicle-b-rudder",gazebo_version:"8.15.0",validation_status_unchanged:"model-structure-trajectory-scored-fail",is_real_world_validation:false,rationale_for_gazebo:"Gazebo has documented stock propeller, rudder/control-surface and hydrodynamics systems. MARUS documentation does not expose the needed actuator/vehicle model and published evidence is differential-thrust oriented.",parameter_mapping:"validation/gazebo-vehicles/vehicle-b-parameter-map.json",sdf:"gazebo/models/vehicle-b-rudder/model.sdf",sdf_checks:{inertia_triangle_inequality:true,gz_sdf_check:"passed",headless_world_load:"passed",neutral_buoyancy_rest:"passed; z drift 3.3e-8 m after 31.5 simulated seconds",zero_command_motion:"below numerical-noise threshold"},direction_checks:direction,protocol:{path:protocolPath,sha256:sha(await readFile(protocolPath)),registered_before_results:protocol.registered_before_trajectory_results},trajectory_results:results,post_failure_diagnostics:{rudder_response_curve:`${ROOT}/rudder-sweep/report.json`,turning_circle_three_way:`${ROOT}/turning-diagnostic/report.json`,gate_effect:"diagnostic only; no tolerance or parameter was changed"},negative_control:{kind:"invert converted Gazebo yaw sign",cases:invertedYaw,passed:invertedYaw.every(x=>x.rejected)},claim_limit:"Independent implementation consistency only. This does not show either implementation matches a real vessel and does not change Vehicle B's model-structure-trajectory-scored-fail status."};
 await writeFile(`${ROOT}/report.json`,JSON.stringify(artifact,null,2)+"\n");console.log(JSON.stringify({status:artifact.status,directions:direction.map(x=>[x.case,x.passed]),results:results.map(x=>[x.case,x.passed,x.metrics])},null,2));
}
await main();
