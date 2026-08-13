import {Worker} from "node:worker_threads";
import type {ResolvedExperimentV1} from "../../../packages/experiment-schema/src/index.ts";
import type {ProductionAction} from "./legacy-production-engine.ts";

export interface WorkerRunJob {id:string;config:ResolvedExperimentV1;actions:ProductionAction[]}
export interface WorkerRunResult {id:string;ok:boolean;steps?:number;ground_truth?:unknown;metrics?:Record<string,number>;final?:unknown;error?:string}

function runOne(job:WorkerRunJob):Promise<WorkerRunResult>{return new Promise((resolve)=>{
  const worker=new Worker(new URL("./worker-entry.ts",import.meta.url),{workerData:structuredClone(job)});let settled=false;
  const finish=(result:WorkerRunResult)=>{if(settled)return;settled=true;resolve(result);};
  worker.once("message",finish);worker.once("error",(error)=>finish({id:job.id,ok:false,error:error.message}));worker.once("exit",(code)=>{if(code!==0)finish({id:job.id,ok:false,error:`worker exited with code ${code}`});});
});}

export async function runExperimentsInWorkers(jobs:WorkerRunJob[],options:{concurrency?:number}={}):Promise<WorkerRunResult[]>{
  const concurrency=Math.min(Math.max(1,Math.floor(options.concurrency??2)),Math.max(jobs.length,1)),results:Array<WorkerRunResult>=Array(jobs.length);let next=0;
  async function consume(){for(;;){const index=next++;if(index>=jobs.length)return;results[index]=await runOne(jobs[index]);}}
  await Promise.all(Array.from({length:concurrency},consume));return results;
}
