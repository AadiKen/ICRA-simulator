import {spawnSync} from "node:child_process";
import {createHash} from "node:crypto";
import {mkdirSync,readFileSync,readdirSync,statSync,writeFileSync} from "node:fs";
import {join,relative,resolve} from "node:path";

const root=resolve(new URL("../../",import.meta.url).pathname);
const output=join(root,"validation/pre-migration/golden");
if (statSync(join(output,"manifest.json")).isFile()) throw new Error("Pre-migration goldens are immutable after migration begins; rebaselining is forbidden.");
mkdirSync(output,{recursive:true});
const sha=(data)=>createHash("sha256").update(data).digest("hex");
const run=(name,args)=>{const result=spawnSync(process.execPath,args,{cwd:root,encoding:"utf8"});if(result.status!==0)throw new Error(`${name} failed:\n${result.stderr}\n${result.stdout}`);const start=result.stdout.indexOf("{");if(start<0)throw new Error(`${name} did not emit JSON metrics.`);const parsed=JSON.parse(result.stdout.slice(start));const text=`${JSON.stringify(parsed,null,2)}\n`;writeFileSync(join(output,`${name}.json`),text);return{name,file:`${name}.json`,sha256:sha(text),bytes:Buffer.byteLength(text)};};
const records=[run("mss",["validation/mssAcceptance.js"]),run("sixdof",["validation/sixDofPlantTest.js"]),run("waves",["validation/waveParityTest.js"]),run("convergence",["validation/convergenceTest.js"])];
const generated=spawnSync(process.execPath,["gazebo/generateGazeboParity.js","--all"],{cwd:root,encoding:"utf8"});if(generated.status!==0)throw new Error(`Gazebo generation failed:\n${generated.stderr}\n${generated.stdout}`);
const gazeboRoot=join(root,"gazebo/generated");const files=[];const walk=(directory)=>{for(const name of readdirSync(directory).sort()){const path=join(directory,name),stats=statSync(path);if(stats.isDirectory())walk(path);else if([".sdf",".json",".config"].some((suffix)=>name.endsWith(suffix))){const data=readFileSync(path);files.push({path:relative(gazeboRoot,path),sha256:sha(data),bytes:data.length});}}};walk(gazeboRoot);const gazeboText=`${JSON.stringify({generator:"gazebo/generateGazeboParity.js --all",files},null,2)}\n`;writeFileSync(join(output,"gazebo-generation.json"),gazeboText);records.push({name:"gazebo-generation",file:"gazebo-generation.json",sha256:sha(gazeboText),bytes:Buffer.byteLength(gazeboText),generatedFiles:files.length});
const manifest={schema_version:1,captured_at:"2026-07-31T00:00:00.000Z",source_state:"pre-migration deterministic-clock audit",physics_status:"pre-migration legacy production implementation",comparison_policy:"validation/pre-migration/acceptance-policy.json",records};const manifestText=`${JSON.stringify(manifest,null,2)}\n`;writeFileSync(join(output,"manifest.json"),manifestText);console.log(manifestText);
