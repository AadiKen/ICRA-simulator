import {createHash} from "node:crypto";
import {mkdir,readFile,writeFile} from "node:fs/promises";
import {replayManeuverWithoutAddedMass} from "../goldenLogCompare.js";

const ROOT="artifacts/gazebo/vehicle-a",IDS=["constant-thrust","turning-circle","yaw-turn","zig-zag","coast-down","current-drift","impulse-hold"];
const sha=async(p:string)=>createHash("sha256").update(await readFile(p)).digest("hex");
const pct=(a:number[],p:number)=>[...a].sort((x,y)=>x-y)[Math.min(a.length-1,Math.ceil(p*a.length)-1)];
const stats=(a:number[])=>({mean:a.reduce((s,x)=>s+x,0)/a.length,p95:pct(a,.95),max:Math.max(...a),final:a.at(-1)});
const wrap=(x:number)=>Math.abs(Math.atan2(Math.sin(x),Math.cos(x)));
const effectiveMass=(rows:any[],forceN:number,windowS=.75)=>{let a00=0,a01=0,a11=0,b0=0,b1=0;for(const row of rows.filter((x:any)=>x.time_s<=windowS)){const t=row.time_s,t2=t*t,y=row.north_m;a00+=t*t;a01+=t*t2;a11+=t2*t2;b0+=t*y;b1+=t2*y;}const det=a00*a11-a01*a01,quadratic=(a00*b1-a01*b0)/det;return forceN/(2*quadratic);};
const scenarios=[];
for(const id of IDS){
  const manifest=JSON.parse(await readFile(`gazebo/generated/manifests/otter_${id}.json`,`utf8`));
  const raw=JSON.parse(await readFile(`${ROOT}/iteration-synchronized/${id}.json`,`utf8`));
  const node=replayManeuverWithoutAddedMass(id,{env:manifest.env}).slice(0,manifest.steps);
  if(node.length!==raw.rows.length)throw Error(`${id}: row mismatch ${node.length}/${raw.rows.length}`);
  if(raw.rows.some((r:any,i:number)=>r.iteration_delta!==1||r.sim_time_delta_s!==.05||Math.abs(r.time_s-(i+1)*.05)>1e-9))throw Error(`${id}: synchronization invariant failed`);
  if(raw.synchronization.blind_retries!==false||raw.synchronization.step_requests_per_target!==1)throw Error(`${id}: request policy invariant failed`);
  const bcod=node.map((x:any,i:number)=>({time_s:(i+1)*.05,north_m:x.N,east_m:x.E,heading_rad:x.yaw,command:raw.rows[i].command}));
  const gazebo=raw.rows.map((r:any)=>({time_s:r.time_s,gazebo_iteration:r.gazebo_iteration,gazebo_sim_time_s:r.gazebo_sim_time_s,north_m:r.north_m,east_m:r.east_m,heading_rad:r.heading_rad,command:r.command}));
  const errors=bcod.map((x:any,i:number)=>({time_s:x.time_s,horizontal_position_error_m:Math.hypot(x.north_m-gazebo[i].north_m,x.east_m-gazebo[i].east_m),heading_error_rad:wrap(x.heading_rad-gazebo[i].heading_rad)}));
  const nodePath=Math.hypot(node.at(-1).N,node.at(-1).E),gazeboPath=Math.hypot(raw.rows.at(-1).north_m,raw.rows.at(-1).east_m),maxPosition=Math.max(...errors.map((x:any)=>x.horizontal_position_error_m));
  scenarios.push({id,dt_s:.05,synchronization:raw.synchronization,actuation:{mode:"bodyWrench",requested_schedule:raw.rows.map((r:any)=>r.command)},bcod_sim:bcod,gazebo_harmonic:gazebo,error_time_series:errors,summary:{horizontal_position_error_m:stats(errors.map((x:any)=>x.horizontal_position_error_m)),heading_error_rad:stats(errors.map((x:any)=>x.heading_error_rad)),path_length_m:{bcod_sim:nodePath,gazebo_harmonic:gazeboPath},relative_position_error:maxPosition/Math.max(nodePath,gazeboPath,1e-12)}});
}
const runner="validation/gazebo-vehicles/run_gazebo_vehicle_a_open_loop.py",scheduleSource=`${ROOT}/open-loop-trajectory-comparison.json`;
const ramp=scenarios.find((x:any)=>x.id==="constant-thrust")!,nodeMass=effectiveMass(ramp.bcod_sim,60),gazeboMass=effectiveMass(ramp.gazebo_harmonic,60),massDifference=Math.abs(nodeMass-gazeboMass)/((nodeMass+gazeboMass)/2);
const artifact={schema_version:3,artifact_kind:"otter-reference-open-loop-fixed-input-trajectory-comparison",vehicle:"vehicle-a-otter-reference",plant_match:{mode:"rigid-mass-only",gazebo_added_mass:false,bcod_sim_added_mass:false,reason:"Native DART fluid_added_mass resets the stabilized Otter fixture.",effective_inertia_fit:{method:"least-squares x(t)=v0*t+0.5*a*t^2 over the first 0.75 s of the 60 N constant-thrust ramp; fitted mass=F/a",bcod_sim_kg:nodeMass,gazebo_harmonic_kg:gazeboMass,relative_difference:massDifference,accepted:massDifference<.03}},timing:{physics_timestep_s:.05,sample_interval_s:.05,physics_iterations_per_sample:1,initial_sample_convention:"The paused-world baseline is iteration initial_iteration at initial_sim_time_s; sample 0 is captured after one commanded step at elapsed t=0.05 s, matching Vehicle C's post-step convention.",iteration_delta_reason:"Vehicle A's Gazebo worlds use max_step_size=0.05 s, so one physics iteration equals the 0.05 s sample interval; Vehicle C uses a 0.01 s physics step and therefore requires five iterations.",full_resolution:true},scenarios,provenance:{runner:{path:runner,sha256:await sha(runner)},schedule_source:{path:scheduleSource,sha256:await sha(scheduleSource)},node_replay:{path:"validation/goldenLogCompare.js",sha256:await sha("validation/goldenLogCompare.js")},model:{path:"gazebo/generated/models/otter/model.sdf",sha256:await sha("gazebo/generated/models/otter/model.sdf")},controller:"none; fixed-input open-loop"},claim_limit:"Otter reference implementation comparison using the stabilized primitive-buoyancy rigid-mass-only fixture and direct body-wrench actuation; not validation of the paper's SR-Surveyor M1.8 Vehicle A, physical pod actuators, or coupled6 hydrostatics."};
await mkdir(ROOT,{recursive:true});await writeFile(`${ROOT}/open-loop-trajectory-comparison-iteration-synchronized.json`,JSON.stringify(artifact,null,2)+"\n");console.log(JSON.stringify(Object.fromEntries(scenarios.map((x:any)=>[x.id,x.summary])),null,2));
