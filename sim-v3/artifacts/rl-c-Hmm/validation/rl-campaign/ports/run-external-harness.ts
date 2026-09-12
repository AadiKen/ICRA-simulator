import {mkdirSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {spawnSync} from "node:child_process";

const root=resolve(".");const out=resolve(root,"artifacts/rl-campaign/external-harness");mkdirSync(out,{recursive:true});
const docker=(args:string[])=>spawnSync("docker",args,{encoding:"utf8"});
function runGazebo(){
 const model=resolve(out,"vehicle-a-task-model.sdf");
 const generate=spawnSync(process.execPath,["--input-type=module","-e","import('./gazebo/generateGazeboParity.js').then(async ({renderModelSdf})=>{const {bcodUsvCoefficients}=await import('./core/vehicles/coefficients.js');process.stdout.write(renderModelSdf(bcodUsvCoefficients,{perThrusterActuation:true,trueOdometry:true,odomHz:20,phaseASensors:true}))})"],{cwd:root,encoding:"utf8"});
 if(generate.status!==0) return {runtime:"Gazebo Harmonic",sdf_valid:false,trace:null,reason:generate.stderr||"SDF generation failed."};
 writeFileSync(model,generate.stdout);
 const check=docker(["run","--rm","-v",`${model}:/tmp/model.sdf:ro`,"--entrypoint","bash","icra27-gazebo-harmonic:harmonic-8.15.0","-lc","gz sdf -k /tmp/model.sdf"]);
 return {runtime:"Gazebo Harmonic",sdf_valid:check.status===0,stdout:check.stdout,stderr:check.stderr,trace:null,reason:"Launch/command/odometry trace wiring requires a generated task world and Transport process; validation retained separately."};
}
function runVrx(){
 const ports=resolve("validation/rl-campaign/ports");const check=docker(["run","--rm","-v",`${ports}:/ports:ro`,"--entrypoint","bash","leadcat/vrx:v3.0.1","-lc","source /opt/ros/jazzy/setup.bash && PYTHONPYCACHEPREFIX=/tmp/cache python3 -m py_compile /ports/vrx_trace_exporter.py && ros2 launch /ports/vrx_trace_exporter.launch.py --show-args"]);
 return {runtime:"VRX",exporter_launch_valid:check.status===0,stdout:check.stdout,stderr:check.stderr,trace:null,reason:"Episode world launch and command-trace driver remain required before a trace can be compared."};
}
const artifact={schema_version:1,artifact_kind:"external-simulator-harness",generated_at:new Date().toISOString(),gazebo:runGazebo(),vrx:runVrx(),next:"Run only when a reference trace and simulator-produced trace-schema-v2 JSON are present; conformance.ts computes separate yaw divergence."};writeFileSync(resolve(out,"harness-status.json"),JSON.stringify(artifact,null,2)+"\n");console.log(JSON.stringify(artifact,null,2));
