import {mkdir,writeFile,appendFile,rename,readdir,readFile} from "node:fs/promises";
import {createHash} from "node:crypto";
import {join,resolve,sep} from "node:path";
import type {RunManifestV1} from "./index.ts";
import {validateManifest} from "./index.ts";
import {outputChecksum} from "./reproducibility-manifest.ts";
export interface RunSummary {success:boolean;failure_reason:string|null;completion_time_s:number;metrics:Record<string,number|string|boolean>}
export class RunArtifactWriter {
  readonly directory:string;
  #manifest?:RunManifestV1;
  constructor(root:string,runId:string){if(!/^[a-zA-Z0-9._-]+$/.test(runId))throw new Error("runId contains unsafe characters.");const base=resolve(root),target=resolve(base,runId);if(!target.startsWith(base+sep))throw new Error("Run directory escapes the output root.");this.directory=target;}
  async initialize(manifest:RunManifestV1,originalConfig:unknown,resolvedConfig:unknown):Promise<void>{validateManifest(manifest);this.#manifest=structuredClone(manifest);await mkdir(this.directory,{recursive:true});await Promise.all([this.#atomicJson("manifest.json",manifest),this.#atomicJson("original_config.json",originalConfig),this.#atomicJson("resolved_config.json",resolvedConfig),mkdir(join(this.directory,"sensor"),{recursive:true}),mkdir(join(this.directory,"plots"),{recursive:true})]);}
  async appendEvent(event:Record<string,unknown>):Promise<void>{await appendFile(join(this.directory,"events.jsonl"),`${JSON.stringify(event)}\n`,"utf8");}
  async writeReplay(replay:unknown):Promise<void>{await this.#atomicJson("replay.json",replay);}
  async finalize(summary:RunSummary):Promise<void>{if(!this.#manifest)throw new Error("Run artifacts must be initialized before finalization.");await this.#atomicJson("summary.json",summary);const files=await this.#files(this.directory),inventory=[];for(const path of files.sort()){const relative=path.slice(this.directory.length+1);if(relative==="manifest.json"||relative.endsWith(".tmp"))continue;const bytes=await readFile(path);inventory.push({path:relative,bytes:bytes.length,checksum_sha256:createHash("sha256").update(bytes).digest("hex"),content_role:this.#role(relative)});}this.#manifest.artifact_inventory=inventory;if(this.#manifest.reproducibility)this.#manifest.reproducibility.output_checksum_sha256=outputChecksum(inventory);validateManifest(this.#manifest);await this.#atomicJson("manifest.json",this.#manifest);}
  async #files(directory:string):Promise<string[]>{const output:string[]=[];for(const entry of await readdir(directory,{withFileTypes:true})){const path=join(directory,entry.name);if(entry.isDirectory())output.push(...await this.#files(path));else if(entry.isFile())output.push(path);}return output;}
  #role(path:string):string{return path.includes("state")?"state-trace":path.includes("actuator")?"actuator-trace":path.includes("metric")||path.includes("summary")?"metrics":path.includes("event")?"events":path.includes("replay")||path.includes("checkpoint")?"replay":"configuration";}
  async #atomicJson(name:string,value:unknown):Promise<void>{const target=join(this.directory,name),temporary=`${target}.tmp`;await writeFile(temporary,`${JSON.stringify(value,null,2)}\n`,"utf8");await rename(temporary,target);}
}
