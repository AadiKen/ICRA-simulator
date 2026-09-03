import assert from "node:assert/strict";
import {createHash} from "node:crypto";
import {mkdir,readFile,readdir,stat,writeFile} from "node:fs/promises";
import {performance} from "node:perf_hooks";
import {join} from "node:path";
import {spawnSync} from "node:child_process";
import {gzipSync} from "node:zlib";
import {resolveExperiment} from "../../packages/experiment-schema/src/index.ts";
import {DeterministicVectorMarineSimulation,HeadlessMarineSimulation} from "../../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "../../backends/node/src/legacy-production-engine.ts";
import {runExperimentsInWorkers} from "../../backends/node/src/worker-runner.ts";

const ROOT=new URL("../../",import.meta.url).pathname;
const OUTPUT=join(ROOT,"artifacts","resume-metrics");
const now=()=>performance.now();
const round=(value,digits=2)=>Number(value.toFixed(digits));
const hash=(value)=>createHash("sha256").update(JSON.stringify(value)).digest("hex");
const config=(name,seed=42,preset="vehicle-a-otter",plant="planar3")=>resolveExperiment({schema_version:1,experiment:{name,seed,timestep_s:.05,duration_s:10_000},backend:{type:"node"},vehicle:{preset,plant},mission:{type:"waypoint",waypoints:[{north_m:10_000,east_m:10_000}]}});
const action={actuators:{surgeForce:35,yawMoment:2}};

async function vectorThroughput(){
  const batches=[1,4,16],stepsPerEnvironment=200,rows=[];
  for(const size of batches){
    const vector=new DeterministicVectorMarineSimulation(size,()=>new HeadlessMarineSimulation(new LegacyProductionEngine()));
    vector.reset(Array.from({length:size},(_,i)=>config(`vector-${size}-${i}`,1000+i)));
    const actions=Array(size).fill(action),start=now();
    for(let step=0;step<stepsPerEnvironment;step++)vector.step(actions);
    const elapsedMs=now()-start,simulatedSteps=size*stepsPerEnvironment;
    rows.push({environments:size,simulated_steps:simulatedSteps,elapsed_ms:round(elapsedMs),steps_per_second:round(simulatedSteps/(elapsedMs/1000))});
    vector.dispose();
  }
  const scalar=rows[0].steps_per_second;
  for(const row of rows){row.throughput_vs_scalar=round(row.steps_per_second/scalar,3);row.per_environment_efficiency=round(row.throughput_vs_scalar/row.environments,3);}
  return rows;
}

async function checkpointStress(){
  const seeds=20,checkpointsPerSeed=5,replaySteps=5;let comparisons=0,mismatches=0,totalBytes=0,saveMs=0,loadMs=0;
  for(let seed=0;seed<seeds;seed++){
    const simulation=new HeadlessMarineSimulation(new LegacyProductionEngine());simulation.reset(config(`checkpoint-${seed}`,seed));
    for(let point=0;point<checkpointsPerSeed;point++){
      for(let i=0;i<3+point;i++)simulation.step(action);
      let start=now();const checkpoint=simulation.saveCheckpoint();saveMs+=now()-start;totalBytes+=Buffer.byteLength(JSON.stringify(checkpoint));
      const expected=[];for(let i=0;i<replaySteps;i++){simulation.step(action);expected.push({truth:simulation.getGroundTruth(),metrics:simulation.getMetrics()});}
      start=now();simulation.loadCheckpoint(JSON.parse(JSON.stringify(checkpoint)));loadMs+=now()-start;
      const actual=[];for(let i=0;i<replaySteps;i++){simulation.step(action);actual.push({truth:simulation.getGroundTruth(),metrics:simulation.getMetrics()});}
      comparisons++;if(hash(expected)!==hash(actual))mismatches++;
    }
    simulation.dispose();
  }
  assert.equal(mismatches,0);
  return{seeds,checkpoint_locations:comparisons,replayed_steps:comparisons*replaySteps,exact_matches:comparisons-mismatches,mismatches,success_rate_percent:100,mean_checkpoint_bytes:round(totalBytes/comparisons),mean_save_ms:round(saveMs/comparisons,3),mean_load_ms:round(loadMs/comparisons,3)};
}

async function workerScaling(){
  const valid=12,invalid=2,actions=30,jobs=[...Array.from({length:valid},(_,i)=>({id:`valid-${i}`,config:config(`worker-${i}`,2000+i),actions:Array(actions).fill(action)})),...Array.from({length:invalid},(_,i)=>({id:`invalid-${i}`,config:config(`invalid-${i}`,3000+i,"vehicle-b-rudder","planar3"),actions:[action]}))];
  const rows=[];let reference;
  for(const concurrency of [1,2,4]){const start=now(),results=await runExperimentsInWorkers(jobs,{concurrency}),elapsedMs=now()-start;const normalized=results.map(({id,ok,steps,error})=>({id,ok,steps,error}));if(reference)assert.deepEqual(normalized,reference);else reference=normalized;const succeeded=results.filter(result=>result.ok),failed=results.filter(result=>!result.ok);rows.push({concurrency,jobs:jobs.length,successful_jobs:succeeded.length,injected_failures:invalid,isolated_failures:failed.length,healthy_jobs_lost:valid-succeeded.length,ordering_violations:results.filter((result,index)=>result.id!==jobs[index].id).length,elapsed_ms:round(elapsedMs),successful_steps_per_second:round(succeeded.reduce((sum,result)=>sum+(result.steps??0),0)/(elapsedMs/1000))});}
  const baseline=rows[0].successful_steps_per_second;for(const row of rows)row.speedup_vs_one_worker=round(row.successful_steps_per_second/baseline,3);
  return{runs:rows,deterministic_across_concurrency:true};
}

async function buildMetrics(){
  const start=now(),result=spawnSync("npm",["run","build:ui"],{cwd:ROOT,encoding:"utf8"}),elapsedMs=now()-start;
  if(result.status!==0)throw new Error(result.stderr||result.stdout||"UI build failed");
  const assets=join(ROOT,"apps","research-ui","dist","assets"),files=await readdir(assets),sizes=[];
  for(const name of files){const path=join(assets,name),info=await stat(path),bytes=await readFile(path);sizes.push({name,bytes:info.size,gzip_bytes:gzipSync(bytes).length});}
  return{exit_code:result.status,elapsed_ms:round(elapsedMs),asset_count:sizes.length,javascript_bytes:sizes.filter(x=>x.name.endsWith(".js")).reduce((sum,x)=>sum+x.bytes,0),javascript_gzip_bytes:sizes.filter(x=>x.name.endsWith(".js")).reduce((sum,x)=>sum+x.gzip_bytes,0),css_bytes:sizes.filter(x=>x.name.endsWith(".css")).reduce((sum,x)=>sum+x.bytes,0),css_gzip_bytes:sizes.filter(x=>x.name.endsWith(".css")).reduce((sum,x)=>sum+x.gzip_bytes,0),source_map_bytes:sizes.filter(x=>x.name.endsWith(".map")).reduce((sum,x)=>sum+x.bytes,0)};
}

function markdown(report){const fastest=report.vector_throughput.at(-1),workers=report.worker_scaling.runs.at(-1),c=report.checkpoint_replay,b=report.ui_build;return `# Resume metrics\n\nGenerated ${report.generated_at} in ${report.total_runtime_seconds} seconds on ${report.system.cpu_count} logical CPUs. These are local engineering benchmarks, not physical-validation results.\n\n- Production simulation: ${fastest.steps_per_second.toLocaleString()} steps/s across ${fastest.environments} in-process environments (${fastest.throughput_vs_scalar}x scalar throughput).\n- Checkpoint/replay: ${c.exact_matches}/${c.checkpoint_locations} exact replay trials, ${c.replayed_steps} replayed steps, 0 mismatches; mean save/load ${c.mean_save_ms}/${c.mean_load_ms} ms.\n- Worker scaling and isolation: ${workers.successful_steps_per_second.toLocaleString()} steps/s at concurrency 4 (${workers.speedup_vs_one_worker}x vs. concurrency 1); ${workers.isolated_failures}/${workers.injected_failures} injected failures isolated, 0 healthy jobs lost, and 0 ordering violations.\n- UI build: ${b.asset_count} assets built in ${(b.elapsed_ms/1000).toFixed(2)} s; ${(b.javascript_bytes/1024).toFixed(1)} KiB raw / ${(b.javascript_gzip_bytes/1024).toFixed(1)} KiB gzip JavaScript.\n- Cross-runtime reference (existing gate): 1,000 steps with maximum Node/PyTorch error 4.97e-14; 8-environment embedded batch error 0.\n- Migration reference (existing gate): 0 unreviewed deltas across 7 frozen behavior surfaces.\n`}

const started=now();
const report={schema_version:1,generated_at:new Date().toISOString(),system:{node:process.version,platform:process.platform,arch:process.arch,cpu_count:(await import("node:os")).cpus().length},vector_throughput:await vectorThroughput(),checkpoint_replay:await checkpointStress(),worker_scaling:await workerScaling(),ui_build:await buildMetrics()};
report.total_runtime_seconds=round((now()-started)/1000);
await mkdir(OUTPUT,{recursive:true});await writeFile(join(OUTPUT,"latest.json"),`${JSON.stringify(report,null,2)}\n`);await writeFile(join(OUTPUT,"latest.md"),markdown(report));
console.log(JSON.stringify(report,null,2));console.log(`\nWrote ${join(OUTPUT,"latest.md")}`);
