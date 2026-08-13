import {parentPort,workerData} from "node:worker_threads";
import {HeadlessMarineSimulation} from "../../../packages/core/src/simulation.ts";
import {LegacyProductionEngine,type ProductionAction} from "./legacy-production-engine.ts";
import type {ResolvedExperimentV1} from "../../../packages/experiment-schema/src/index.ts";

interface JobData {id:string;config:ResolvedExperimentV1;actions:ProductionAction[]}
function run(job:JobData){
  const simulation=new HeadlessMarineSimulation(new LegacyProductionEngine());
  try{
    simulation.reset(job.config);let final:any=null,steps=0;
    for(const action of job.actions){final=simulation.step(action);steps++;if(final.terminated||final.truncated)break;}
    return{id:job.id,ok:true,steps,ground_truth:simulation.getGroundTruth(),metrics:simulation.getMetrics(),final};
  }finally{simulation.dispose();}
}
try{parentPort!.postMessage(run(workerData as JobData));}catch(error){parentPort!.postMessage({id:(workerData as JobData)?.id??"unknown",ok:false,error:error instanceof Error?error.message:String(error)});}
