import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {assembleVrxExporterTrace} from "./episode-driver.ts";
import {generateNodeReference} from "./generate-node-reference.ts";
import {compareTraces} from "./conformance.ts";
import type {TraceV2} from "./trace-schema-v2.ts";

const seeds=Array.from({length:20},(_,i)=>20000+i);
const checkpoints=[{label:"30",index:600},{label:"60",index:1200},{label:"120_terminal",index:2399}];
const root=resolve("artifacts/rl-campaign/vrx-gate7-full");
const referenceRoot=resolve("artifacts/rl-campaign/gate-7/reference");
const tolerance={kinematic_abs:1e-4,environment_vector_relative:0.30,environment_direction_deg:15,component_sign_floor_m:0.5};
const read=(path:string)=>JSON.parse(readFileSync(path,"utf8")) as TraceV2;
const norm=(v:number[])=>Math.hypot(...v);

const episodes=seeds.map(seed=>{
  const onDir=resolve(root,"on",`seed-${seed}`),offDir=resolve(root,"off",`seed-${seed}`);
  const onPath=resolve(onDir,"trace.json"),offPath=resolve(offDir,"trace.json"),nodeOffPath=resolve(offDir,"node-trace.json");
  const vrxOn=assembleVrxExporterTrace(seed,resolve(onDir,"raw.jsonl"),onPath);
  const vrxOff=assembleVrxExporterTrace(seed,resolve(offDir,"raw.jsonl"),offPath);
  const nodeOn=read(resolve(referenceRoot,`node-${seed}.json`));
  const nodeOff=generateNodeReference(seed,nodeOffPath,2400,{wind:0,current:0,action:1});
  const kinematic=compareTraces(nodeOn,vrxOn);
  const warmupVelocity=([3,4,5] as const).map(axis=>{
    const values=vrxOn.samples.slice(10).map((sample,index)=>Math.abs(sample.state[axis]-nodeOn.samples[index+10].state[axis]));
    return {max_abs:Math.max(...values),mean_abs:values.reduce((sum,value)=>sum+value,0)/values.length};
  });
  const environment=checkpoints.map(({label,index})=>{
    const n=[nodeOn.samples[index].state[0]-nodeOff.samples[index].state[0],nodeOn.samples[index].state[1]-nodeOff.samples[index].state[1]];
    const v=[vrxOn.samples[index].state[0]-vrxOff.samples[index].state[0],vrxOn.samples[index].state[1]-vrxOff.samples[index].state[1]];
    const nn=norm(n),vn=norm(v),den=Math.max(nn,0.5),dot=n[0]*v[0]+n[1]*v[1];
    const angle=nn>0&&vn>0?Math.acos(Math.max(-1,Math.min(1,dot/(nn*vn))))*180/Math.PI:180;
    const sign_ok=n.every((x,i)=>Math.abs(x)<tolerance.component_sign_floor_m||Math.sign(x)===Math.sign(v[i]));
    const relative_vector_divergence=norm([v[0]-n[0],v[1]-n[1]])/den;
    return {checkpoint_s:label==="120_terminal"?119.95:Number(label),sample_index:index,node_environment_displacement_ne_m:n,vrx_environment_displacement_ne_m:v,node_magnitude_m:nn,vrx_magnitude_m:vn,magnitude_divergence:Math.abs(vn-nn)/den,relative_vector_divergence,direction_difference_deg:angle,component_sign_consistent:sign_ok,pass:relative_vector_divergence<=tolerance.environment_vector_relative&&angle<=tolerance.environment_direction_deg&&sign_ok};
  });
  return {seed,samples:vrxOn.samples.length,kinematic,warmupVelocity,kinematic_pass:Object.values(kinematic.per_state_max_abs).every(x=>x<=tolerance.kinematic_abs),environment,environment_pass:environment.every(x=>x.pass)};
});

const axes=["N","E","yaw","u","v","r"] as const;
const report={
  schema_version:1,
  artifact_kind:"vrx-gate-7-full-conformance",
  status:"COMPLETE",
  contract_content_sha256:"cc2c35cafee9eceb31cbb7e76522426cbabbcc78ef4176ba03a69bbdf420a1fb",
  held_out_seeds:seeds,
  synchronized_samples_per_episode:2400,
  tolerance,
  kinematic:{
    pass:episodes.every(x=>x.kinematic_pass),
    seeds_passing:episodes.filter(x=>x.kinematic_pass).map(x=>x.seed),
    seeds_failing:episodes.filter(x=>!x.kinematic_pass).map(x=>x.seed),
    per_axis_max_abs_warmup_excluded:{
      N:Math.max(...episodes.map(x=>x.kinematic.per_state_max_abs.N)),
      E:Math.max(...episodes.map(x=>x.kinematic.per_state_max_abs.E)),
      yaw:Math.max(...episodes.map(x=>x.kinematic.per_state_max_abs.yaw)),
      u:Math.max(...episodes.map(x=>x.warmupVelocity[0].max_abs)),
      v:Math.max(...episodes.map(x=>x.warmupVelocity[1].max_abs)),
      r:Math.max(...episodes.map(x=>x.warmupVelocity[2].max_abs))
    },
    per_axis_mean_abs_warmup_excluded:{
      u:episodes.reduce((sum,x)=>sum+x.warmupVelocity[0].mean_abs,0)/episodes.length,
      v:episodes.reduce((sum,x)=>sum+x.warmupVelocity[1].mean_abs,0)/episodes.length,
      r:episodes.reduce((sum,x)=>sum+x.warmupVelocity[2].mean_abs,0)/episodes.length
    },
    warmup_exclusion:{samples_excluded:10,duration_s:.5,basis:"Measured identical samples 0-9 publisher transient on all 20 seeds; sample 10 is the first ordinary velocity sample."},
    vrx_odometry_initialization_transient_separate:{classification:"known VRX startup reporting artifact; raw samples retained",maximum_abs_divergence:{u:Math.max(...episodes.map(x=>x.kinematic.per_state_max_abs.u)),v:Math.max(...episodes.map(x=>x.kinematic.per_state_max_abs.v)),r:Math.max(...episodes.map(x=>x.kinematic.per_state_max_abs.r))}},
    note:"Pass/fail remains false after warm-up exclusion; readable velocity summaries use samples 10-2399."
  },
  environment_response:{
    pass:episodes.every(x=>x.environment_pass),
    seeds_passing:episodes.filter(x=>x.environment_pass).map(x=>x.seed),
    seeds_failing:episodes.filter(x=>!x.environment_pass).map(x=>x.seed),
    maximum_relative_vector_divergence:Math.max(...episodes.flatMap(x=>x.environment.map(y=>y.relative_vector_divergence))),
    maximum_magnitude_divergence:Math.max(...episodes.flatMap(x=>x.environment.map(y=>y.magnitude_divergence))),
    maximum_direction_difference_deg:Math.max(...episodes.flatMap(x=>x.environment.map(y=>y.direction_difference_deg))),
    failed_checkpoints:episodes.flatMap(x=>x.environment.filter(y=>!y.pass).map(y=>({seed:x.seed,...y}))),
    definition:"Paired environment-on minus environment-off N/E displacement for each simulator, evaluated independently per seed at each preregistered checkpoint."
  },
  overall_gate_7_pass:episodes.every(x=>x.kinematic_pass)&&episodes.every(x=>x.environment_pass),
  episodes
};
mkdirSync(root,{recursive:true});writeFileSync(resolve(root,"gate-7-result.json"),JSON.stringify(report,null,2)+"\n");
console.log(JSON.stringify({kinematic:report.kinematic,environment_response:{...report.environment_response,failed_checkpoints:report.environment_response.failed_checkpoints.length},overall_gate_7_pass:report.overall_gate_7_pass},null,2));
