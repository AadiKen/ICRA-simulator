import {createHash} from "node:crypto";
import {cp,mkdir,mkdtemp,readFile,rename,rm,stat,writeFile} from "node:fs/promises";
import {tmpdir} from "node:os";
import {basename,dirname,join,resolve} from "node:path";
import {spawn} from "node:child_process";
import {createReproducibilityManifest,outputChecksum} from "../../packages/metrics/src/reproducibility-manifest.ts";

type Run={id:string;seed:number;command:string[];inputs?:string[];checkpoint_interval_steps:number;env_count?:number};
type Campaign={schema_version:1;campaign_id:string;runs:Run[];execution?:{host_class:"local"|"cluster"|"synthetic";host_id:string;device:string;arch:string;numerics_mode:string;blas_backend:string};scheduler?:{backend:"local"|"slurm";partition?:string;wall_time?:string;gpu_type?:string}};
type State={schema_version:1;campaign_id:string;updated_at:string;runs:Record<string,{status:"pending"|"running"|"interrupted"|"failed"|"completed";attempts:number;last_checkpoint?:string;scratch_directory?:string;exit_code?:number;output_checksums?:Record<string,string>}>};
const safe=(id:string)=>{if(!/^[a-zA-Z0-9._-]+$/.test(id))throw new Error(`Unsafe run id: ${id}`);return id;};
const atomic=async(path:string,value:unknown)=>{await mkdir(dirname(path),{recursive:true});const temp=`${path}.tmp`;await writeFile(temp,`${JSON.stringify(value,null,2)}\n`);await rename(temp,path);};
const digest=async(path:string)=>createHash("sha256").update(await readFile(path)).digest("hex");

export async function emitSchedulerScript(campaignPath:string,campaign:Campaign,output:string):Promise<string>{
  const scheduler=campaign.scheduler??{backend:"local" as const},path=join(output,`${campaign.campaign_id}.${scheduler.backend}.sh`);
  const directives=scheduler.backend==="slurm"?["#!/usr/bin/env bash",`#SBATCH --job-name=${campaign.campaign_id}`,`#SBATCH --time=${scheduler.wall_time??"01:00:00"}`,...(scheduler.partition?[`#SBATCH --partition=${scheduler.partition}`]:[]),...(scheduler.gpu_type?[`#SBATCH --gres=gpu:${scheduler.gpu_type}:1`]:[])]:["#!/usr/bin/env bash"];
  const body=[...directives,"set -euo pipefail",`node --experimental-strip-types validation/rl-campaign/runner.ts ${JSON.stringify(resolve(campaignPath))} ${JSON.stringify(resolve(output))}`].join("\n")+"\n";await writeFile(path,body,{mode:0o755});return path;
}
export async function runCampaign(campaignPath:string,outputRoot:string):Promise<State>{
  const campaign=JSON.parse(await readFile(campaignPath,"utf8")) as Campaign;if(campaign.schema_version!==1||!campaign.runs?.length)throw new Error("Invalid campaign manifest");
  const output=resolve(outputRoot),statePath=join(output,"campaign-state.json");await mkdir(output,{recursive:true});
  let state:State={schema_version:1,campaign_id:campaign.campaign_id,updated_at:new Date().toISOString(),runs:{}};try{state=JSON.parse(await readFile(statePath,"utf8"));}catch{}
  await emitSchedulerScript(campaignPath,campaign,output);
  const lockPaths={npm:"package-lock.json",uv:"uv.lock",arm64:"validation/rl-campaign/locks/arm64-darwin.json",x86_64:"validation/rl-campaign/locks/x86_64-linux.json"},lockfileHashes=Object.fromEntries(await Promise.all(Object.entries(lockPaths).map(async([name,path])=>[name,await digest(resolve(path))])));
  for(const run of campaign.runs){safe(run.id);const previous=state.runs[run.id];if(previous?.status==="completed")continue;
    const finalDir=join(output,"runs",run.id),scratch=await mkdtemp(join(process.env.RL_CAMPAIGN_SCRATCH??tmpdir(),`bcod-${run.id}-`)),scratchOut=join(scratch,"output");await mkdir(scratchOut,{recursive:true});
    for(const input of run.inputs??[]){const source=resolve(input),target=join(scratch,"inputs",basename(source));await mkdir(dirname(target),{recursive:true});await cp(source,target,{recursive:true});}
    const checkpoint=join(finalDir,"checkpoint.json");try{await stat(checkpoint);await mkdir(scratchOut,{recursive:true});await cp(checkpoint,join(scratchOut,"checkpoint.json"));}catch{}
    state.runs[run.id]={status:"running",attempts:(previous?.attempts??0)+1,last_checkpoint:previous?.last_checkpoint,scratch_directory:scratchOut};state.updated_at=new Date().toISOString();await atomic(statePath,state);
    const execution=campaign.execution??{host_class:"local" as const,host_id:"local-apple-silicon",device:"cpu",arch:process.arch,numerics_mode:"float64",blas_backend:"Accelerate"};
    const reproducibility=await createReproducibilityManifest({root:resolve("."),seed:run.seed,config:run,producer:"benchmark",lockfileHashes,execution:{...execution,env_count:run.env_count??1}}),incrementalManifest={schema_version:1,run_id:run.id,campaign_id:campaign.campaign_id,status:"running",attempt:state.runs[run.id].attempts,reproducibility,output_checksums:{}};await atomic(join(finalDir,"run-manifest.json"),incrementalManifest);
    const [program,...args]=run.command,child=spawn(program,args,{cwd:resolve("."),stdio:"inherit",env:{...process.env,RL_RUN_DIR:scratchOut,RL_CHECKPOINT_PATH:join(scratchOut,"checkpoint.json"),RL_RESUME_FROM:join(scratchOut,"checkpoint.json"),RL_CHECKPOINT_INTERVAL_STEPS:String(run.checkpoint_interval_steps)}});
    const forward=()=>child.kill("SIGTERM");process.once("SIGTERM",forward);process.once("SIGINT",forward);const code=await new Promise<number|null>((ok)=>child.once("exit",ok));process.off("SIGTERM",forward);process.off("SIGINT",forward);
    await mkdir(dirname(finalDir),{recursive:true});await cp(scratchOut,finalDir,{recursive:true,force:true});await rm(scratch,{recursive:true,force:true});
    let last_checkpoint:string|undefined;try{last_checkpoint=await digest(checkpoint);}catch{}
    if(code!==0){state.runs[run.id]={status:code===143||code===null?"interrupted":"failed",attempts:state.runs[run.id].attempts,last_checkpoint,exit_code:code??143};state.updated_at=new Date().toISOString();await atomic(statePath,state);if(code!==143&&code!==null)throw new Error(`Run ${run.id} failed with ${code}`);return state;}
    const result=join(finalDir,"result.json");const output_checksums:Record<string,string>={};try{output_checksums["result.json"]=await digest(result);}catch{throw new Error(`Run ${run.id} exited successfully without result.json`);}
    reproducibility.output_checksum_sha256=outputChecksum(Object.entries(output_checksums).map(([path,checksum_sha256])=>({path,bytes:0,checksum_sha256})));await atomic(join(finalDir,"run-manifest.json"),{...incrementalManifest,status:"completed",reproducibility,output_checksums});state.runs[run.id]={status:"completed",attempts:state.runs[run.id].attempts,last_checkpoint,exit_code:0,output_checksums};state.updated_at=new Date().toISOString();await atomic(statePath,state);
  }return state;
}
if(import.meta.url===`file://${process.argv[1]}`){const [, ,campaign,output]=process.argv;if(!campaign||!output)throw new Error("usage: runner.ts CAMPAIGN.json OUTPUT_DIR");await runCampaign(campaign,output);}
