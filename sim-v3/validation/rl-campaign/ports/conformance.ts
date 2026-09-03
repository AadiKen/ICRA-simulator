import {readFileSync,writeFileSync,mkdirSync,existsSync} from "node:fs";
import {dirname,resolve} from "node:path";
import {spawnSync} from "node:child_process";
import {assertTraceV2,type TraceV2} from "./trace-schema-v2.ts";

type Axis="N"|"E"|"yaw"|"u"|"v"|"r";
const axes:Axis[]=["N","E","yaw","u","v","r"];
export function compareTraces(reference:TraceV2,candidate:TraceV2){
  assertTraceV2(reference);assertTraceV2(candidate);
  if(reference.samples.length!==candidate.samples.length)throw new Error("Trace length mismatch");
  const maxima=Object.fromEntries(axes.map((axis,i)=>[axis,Math.max(...reference.samples.map((s,j)=>Math.abs(s.state[i]-candidate.samples[j].state[i])))]));
  const first_divergence=Object.fromEntries(axes.map((axis,i)=>[axis,reference.samples.findIndex((s,j)=>Math.abs(s.state[i]-candidate.samples[j].state[i])>1e-4)]));
  return {samples:reference.samples.length,per_state_max_abs:maxima,first_divergence_step:first_divergence,yaw_divergence_separate:true};
}
function main(){
 const [referencePath,candidatePath,outPath]=process.argv.slice(2);if(!referencePath||!candidatePath||!outPath)throw new Error("usage: conformance.ts reference.json candidate.json result.json");
 const value={schema_version:1,artifact_kind:"external-simulator-conformance",executed:true,reference:resolve(referencePath),candidate:resolve(candidatePath),result:compareTraces(JSON.parse(readFileSync(referencePath,"utf8")),JSON.parse(readFileSync(candidatePath,"utf8")))};
 mkdirSync(dirname(resolve(outPath)),{recursive:true});writeFileSync(outPath,JSON.stringify(value,null,2)+"\n");
}if(process.argv[1]?.endsWith("conformance.ts"))main();
