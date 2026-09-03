import {readFile,writeFile,mkdir} from "node:fs/promises";
import {resolve,join} from "node:path";
import {fileURLToPath} from "node:url";
import {scenarios,manifest as benchmarkManifest} from "../../benchmarks/usv-bench-36/manifest.ts";
import {VEHICLES} from "../../packages/vehicle-sdk/src/index.ts";
import {RunArtifactWriter} from "../../packages/metrics/src/run-artifacts.ts";
import {createReproducibilityManifest,hashConfig,sha256} from "../../packages/metrics/src/reproducibility-manifest.ts";
import {guardArtifactTree} from "../../packages/metrics/src/manifest-guard.ts";
import {run,runOne,type Actor,type Policy} from "./run.ts";

const root=resolve(new URL("../../",import.meta.url).pathname);
const output=resolve(process.argv[2]??join(root,"artifacts/usv-bench-36-complete-20260825"));
const ppoSeeds=(base:number)=>[base,base+10_000,base+20_000,base+30_000,base+40_000];
const mean=(values:number[])=>values.reduce((sum,value)=>sum+value,0)/values.length;
const std=(values:number[])=>{const center=mean(values);return Math.sqrt(mean(values.map(value=>(value-center)**2)));};
const csv=(rows:Record<string,unknown>[])=>{const keys=Object.keys(rows[0]??{});const cell=(value:unknown)=>{const text=typeof value==="string"?value:JSON.stringify(value);return /[",\n]/.test(text)?`"${text.replaceAll('"','""')}"`:text;};return `${keys.join(",")}\n${rows.map(row=>keys.map(key=>cell(row[key])).join(",")).join("\n")}\n`;};

type FailureCategory="collision"|"timeout"|"instability"|"allocation saturation"|"other";
function classify(row:ReturnType<typeof runOne>):{success:boolean;category:FailureCategory|null;step:number|null}{
  if(!row.finite)return{success:false,category:"instability",step:row.steps};
  if(row.mission_reached)return{success:true,category:null,step:null};
  return{success:false,category:"timeout",step:row.steps};
}

async function persist(policy:Policy,row:ReturnType<typeof runOne>){
  const runId=`${row.scenario_id}__${row.vehicle_id}__${policy}__seed-${row.seed}`;
  const config={benchmark:"USV-Bench-36",benchmark_version:benchmarkManifest.version,scenario_id:row.scenario_id,vehicle_id:row.vehicle_id,controller:policy,seed:row.seed,timestep_s:.05,duration_s:5,ppo_actor:policy==="ppo"?"artifacts/baselines/checkpoints/ppo-actor.json":null};
  const reproducibility=await createReproducibilityManifest({root,seed:row.seed,config,configSha256:hashConfig(config),producer:"benchmark"});
  const writer=new RunArtifactWriter(join(output,"runs"),runId),failure=classify(row),actorPath=join(root,"artifacts/baselines/checkpoints/ppo-actor.json"),inputs=policy==="ppo"?[{path:"artifacts/baselines/checkpoints/ppo-actor.json",checksum_sha256:sha256(await readFile(actorPath))}]:[];
  await writer.initialize({schema_version:1,run_id:runId,experiment_checksum:reproducibility.config_sha256,created_at:reproducibility.timestamp,software:{node:process.version,platform:process.platform,backend:"node-cpu"},seeds:[row.seed],inputs,validation_scope:{status:row.validation_status,claim:VEHICLES[row.vehicle_id].validation.claim,physical_validation:"No promotion: software policy evaluation only."},warnings:[...VEHICLES[row.vehicle_id].validation.limitations],reproducibility,backend_capabilities:{node:"passed"},sensor_retention:{mode:"summary",raw_plugins:[],max_bytes_per_run:1_000_000},failure_summary:{count:failure.success?0:1,categories:failure.category?[failure.category]:[]}},config,config);
  const result={...row,success:failure.success,failure_category:failure.category,failure_step:failure.step};
  await writeFile(join(writer.directory,"metrics.json"),`${JSON.stringify(result,null,2)}\n`);
  await writer.appendEvent({step:0,type:"RESET",scenario_id:row.scenario_id,vehicle_id:row.vehicle_id,controller:policy,seed:row.seed});
  if(!failure.success)await writer.appendEvent({step:failure.step,type:"FAILURE",category:failure.category,terminal_state:row.terminal_state});
  await writer.writeReplay({schema_version:1,seed:row.seed,steps:row.steps,terminal_state:row.terminal_state});
  await writer.finalize({success:failure.success,failure_reason:failure.category,completion_time_s:row.steps*.05,metrics:{progress_m:row.progress_m,energy:row.energy,mission_reached:row.mission_reached}});
  return result;
}

export async function runFullCampaign(){
  await mkdir(output,{recursive:true});
  const reference=JSON.parse(await readFile(join(root,"artifacts/baselines/usv-bench-policy-report.json"),"utf8")).rows.filter((row:any)=>row.policy==="pid"||row.policy==="mpc");
  const gate=run(["pid","mpc"]).rows.map(({terminal_state,...row})=>row);
  if(JSON.stringify(gate)!==JSON.stringify(reference))throw new Error("PID/MPC exact-match gate failed; PPO campaign was not started.");
  const actor=JSON.parse(await readFile(join(root,"artifacts/baselines/checkpoints/ppo-actor.json"),"utf8")) as Actor;
  const rows=[];
  for(const policy of ["pid","mpc"] as const)for(const scenario of scenarios)for(const vehicle of benchmarkManifest.vehicles)rows.push(await persist(policy,runOne(policy,scenario,vehicle,actor)));
  for(const scenario of scenarios)for(const vehicle of benchmarkManifest.vehicles)for(const seed of ppoSeeds(scenario.seed))rows.push(await persist("ppo",runOne("ppo",scenario,vehicle,actor,seed)));
  const ppoVariance=[];
  for(const scenario of scenarios)for(const vehicle of benchmarkManifest.vehicles){const selected=rows.filter(row=>row.policy==="ppo"&&row.scenario_id===scenario.id&&row.vehicle_id===vehicle),values=selected.map(row=>row.progress_m);ppoVariance.push({scenario_id:scenario.id,axis:scenario.axis,tier:scenario.severity.tier,vehicle_id:vehicle,metric:"progress_m",mean:mean(values),standard_deviation:std(values),min:Math.min(...values),max:Math.max(...values),raw_per_seed:selected.map(row=>({seed:row.seed,value:row.progress_m,manifest:`runs/${row.scenario_id}__${row.vehicle_id}__ppo__seed-${row.seed}/manifest.json`}))});}
  const heatmap=(["pid","mpc","ppo"] as const).flatMap(policy=>scenarios.flatMap(scenario=>benchmarkManifest.vehicles.map(vehicle=>{const selected=rows.filter(row=>row.policy===policy&&row.scenario_id===scenario.id&&row.vehicle_id===vehicle);return{scenario_id:scenario.id,axis:scenario.axis,tier:scenario.severity.tier,vehicle_id:vehicle,controller:policy,seeds:selected.length,progress_mean_m:mean(selected.map(row=>row.progress_m)),success_rate:mean(selected.map(row=>row.success?1:0)),energy_mean:mean(selected.map(row=>row.energy))};})));
  const failures=rows.filter(row=>!row.success).map(row=>({scenario_id:row.scenario_id,vehicle_id:row.vehicle_id,controller:row.policy,seed:row.seed,category:row.failure_category,step_of_failure:row.failure_step,terminal_state:row.terminal_state}));
  const counts=Object.fromEntries((["collision","timeout","instability","allocation saturation","other"] as const).map(category=>[category,failures.filter(row=>row.category===category).length])),otherRate=failures.length?counts.other/failures.length:0;
  const report={schema_version:1,artifact_kind:"usv-bench-36-complete-controller-campaign",status:otherRate>.05?"failed-other-taxonomy-exceeds-5-percent":"passed",exact_match_gate:{status:"passed",reference:"artifacts/baselines/usv-bench-policy-report.json",controllers:["pid","mpc"],rows:216},configuration:{scenarios:36,vehicles:3,controllers:["pid","mpc","ppo"],pid_mpc_seeds:1,ppo_seeds:5},executed_runs:rows.length,manifest_complete_runs:rows.length,failure_taxonomy:{counts,total:failures.length,other_rate:otherRate,other_exceeds_5_percent:otherRate>.05},files:{heatmap:"heatmap.csv",ppo_variance:"ppo-variance.json",ppo_variance_csv:"ppo-variance.csv",failures:"failure-taxonomy.json",rows:"rows.json"}};
  await Promise.all([writeFile(join(output,"report.json"),`${JSON.stringify(report,null,2)}\n`),writeFile(join(output,"rows.json"),`${JSON.stringify(rows,null,2)}\n`),writeFile(join(output,"heatmap.csv"),csv(heatmap)),writeFile(join(output,"ppo-variance.json"),`${JSON.stringify(ppoVariance,null,2)}\n`),writeFile(join(output,"ppo-variance.csv"),csv(ppoVariance.map(row=>({...row,raw_per_seed:row.raw_per_seed})))),writeFile(join(output,"failure-taxonomy.json"),`${JSON.stringify({counts,total:failures.length,other_rate:otherRate,rows:failures},null,2)}\n`)]);
  await guardArtifactTree(join(output,"runs"));
  return report;
}

if(process.argv[1]===fileURLToPath(import.meta.url)){const report=await runFullCampaign();console.log(JSON.stringify({output,...report},null,2));if(report.status!=="passed")process.exitCode=1;}
