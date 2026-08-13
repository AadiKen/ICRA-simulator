import {createHash} from "node:crypto";
import {existsSync,mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {dirname,join,resolve} from "node:path";
import {spawnSync} from "node:child_process";

const root=resolve(new URL("../../",import.meta.url).pathname);
const policy=JSON.parse(readFileSync(join(root,"validation/behavior-supersession/policy.json"),"utf8"));
const sha=(data)=>createHash("sha256").update(data).digest("hex");
const args=Object.fromEntries(process.argv.slice(2).map((value,index,array)=>value.startsWith("--")?[value.slice(2),array[index+1]]:null).filter(Boolean));
const selected=policy.allowed.find(({id})=>id===args.policy);
if(!selected)throw new Error("Behavioral golden supersession is forbidden unless --policy names an explicit allowed policy.");
if(!/^[a-f0-9]{7,40}$/.test(args.capability_commit??""))throw new Error("--capability_commit must identify the already-committed capability implementation.");
const resolvedCommit=spawnSync("git",["rev-parse",args.capability_commit],{cwd:root,encoding:"utf8"});
if(resolvedCommit.status!==0)throw new Error("Capability implementation commit does not resolve in Git.");
const capabilityCommit=resolvedCommit.stdout.trim();
if(spawnSync("git",["merge-base","--is-ancestor",capabilityCommit,"HEAD"],{cwd:root}).status!==0)throw new Error("Capability implementation commit must be HEAD or an ancestor of HEAD.");
if(spawnSync("git",["diff","--quiet","--","."],{cwd:root}).status!==0||spawnSync("git",["diff","--cached","--quiet","--","."],{cwd:root}).status!==0)throw new Error("The simulator worktree must be clean before capability-golden capture.");
const legacyPath=join(root,selected.legacy_golden),manifestPath=join(root,selected.legacy_manifest),candidatePath=join(root,selected.candidate_golden),deltaPath=join(root,selected.delta_artifact);
if(existsSync(candidatePath)||existsSync(deltaPath))throw new Error("Supersession artifacts are immutable and already exist; overwriting/rebaselining is forbidden.");
const legacyText=readFileSync(legacyPath),manifest=JSON.parse(readFileSync(manifestPath,"utf8"));
const record=manifest.records.find(({file})=>join(dirname(manifestPath),file)===legacyPath);
if(!record||sha(legacyText)!==record.sha256)throw new Error("Legacy golden checksum does not match its immutable manifest; supersession refused.");
if(!args.candidate||!args.justifications)throw new Error("Provide --candidate and --justifications inputs.");
const candidateInput=resolve(args.candidate),justifications=JSON.parse(readFileSync(resolve(args.justifications),"utf8"));
const candidate=JSON.parse(readFileSync(candidateInput,"utf8")),legacy=JSON.parse(legacyText);
if(justifications.schema_version!==1||!Array.isArray(justifications.changes)||justifications.changes.length===0)throw new Error("Justifications require a non-empty changes array.");
for(const change of justifications.changes)if(!change.path_prefix||!change.justification)throw new Error("Every supersession change needs path_prefix and justification.");
const changes=[];
function diff(before,after,path="$"){
  if(typeof before==="number"&&typeof after==="number"){if(!Object.is(before,after))changes.push({path,before,after,absolute_delta:Math.abs(after-before)});return;}
  if(Array.isArray(before)||Array.isArray(after)){if(!Array.isArray(before)||!Array.isArray(after)||before.length!==after.length){changes.push({path,before,after});return;}before.forEach((value,index)=>diff(value,after[index],`${path}[${index}]`));return;}
  if(before&&after&&typeof before==="object"&&typeof after==="object"){for(const key of new Set([...Object.keys(before),...Object.keys(after)]))diff(before[key],after[key],`${path}.${key}`);return;}
  if(before!==after)changes.push({path,before,after});
}
diff(legacy,candidate);
if(changes.length===0)throw new Error("Candidate has no behavior changes and cannot supersede the legacy golden.");
for(const change of changes){const justification=justifications.changes.find(({path_prefix})=>change.path.startsWith(path_prefix));if(!justification)throw new Error(`Unjustified behavior delta: ${change.path}`);change.justification=justification.justification;}
const candidateText=`${JSON.stringify({...candidate,supersession:{policy:selected.id,capability_commit:capabilityCommit,legacy_sha256:record.sha256}},null,2)}\n`;
const artifact={schema_version:1,artifact_kind:"reviewed-behavior-supersession-delta",policy:selected.id,capability_commit:capabilityCommit,legacy:{path:selected.legacy_golden,sha256:record.sha256},candidate:{path:selected.candidate_golden,sha256:sha(candidateText)},change_count:changes.length,changes};
mkdirSync(dirname(candidatePath),{recursive:true});writeFileSync(candidatePath,candidateText);writeFileSync(deltaPath,`${JSON.stringify(artifact,null,2)}\n`);
console.log(JSON.stringify({created:[selected.candidate_golden,selected.delta_artifact],change_count:changes.length},null,2));
