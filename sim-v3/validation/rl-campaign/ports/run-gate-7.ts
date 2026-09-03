import {existsSync,mkdirSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {prepareEpisode,FULL_STEPS} from "./episode-driver.ts";

const root=resolve("artifacts/rl-campaign/gate-7"),seeds=Array.from({length:20},(_,i)=>20000+i);
const simulator=(name:"Gazebo Harmonic"|"VRX")=>{
 const prepared=seeds.map(seed=>prepareEpisode(name,seed,FULL_STEPS));
 const traces=seeds.filter(seed=>existsSync(resolve(root,name.replaceAll(" ","-").toLowerCase(),`${seed}.json`)));
 return {simulator:name,smoke:{requested_episodes:3,valid_traces:0,status:"not-run-live-driver-not-connected"},full:{seeds,valid_traces:traces,pass:false,status:"not-run-live-driver-not-connected"},per_axis_max_divergence:{N:null,E:null,yaw:null,u:null,v:null,r:null},seeds_exceeding_tolerance:seeds,divergence_growth_start:{N:null,E:null,yaw:null,u:null,v:null,r:null},inertia:{required:"geometry-derived bootstrap craft mass and geometry",used_for_every_run:false,prepared_episode_count:prepared.length,reason:"No simulator episode completed; this must not be represented as live use."}};
};
const report={schema_version:1,artifact_kind:"gate-7-external-simulator-conformance",status:"BLOCKED_STRUCTURAL_GAP",checkpoint:"Gate 7",generated_at:new Date().toISOString(),tolerance_abs:1e-4,node_reference:{status:"complete",seeds,trace_directory:resolve(root,"reference"),traces:seeds.map(seed=>resolve(root,"reference",`node-${seed}.json`))},reason:"VRX v3.0.1 launches the stock WAM-V, whose hull geometry, mass, inertia, buoyancy and thruster placement are not the geometry-derived bootstrap craft. Running it would violate the explicit requirement that corrected inertia be used for every run. A VRX vehicle-model port is required before command/topic orchestration can yield a valid Gate 7 measurement.",simulators:[simulator("Gazebo Harmonic"),simulator("VRX")],required_next_work:"Create and validate a VRX model/SDF port of the bootstrap craft, including its geometry-derived rigid-body inertia and compatible buoyancy/thruster model; then resume bounded episode orchestration. This is modeling work, not an ordinary wiring bug.",prohibition:"No retuning, re-derivation, or compensating patch was performed after this checkpoint."};
mkdirSync(root,{recursive:true});writeFileSync(resolve(root,"gate-7-report.json"),JSON.stringify(report,null,2)+"\n");console.log(JSON.stringify(report,null,2));
