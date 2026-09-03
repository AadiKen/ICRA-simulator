import {createHash} from "node:crypto";
import {readFile} from "node:fs/promises";
import {cpus,hostname,totalmem,type,release} from "node:os";
import {resolve} from "node:path";
import {spawnSync} from "node:child_process";

export type ManifestProducer="benchmark"|"determinism-sweep"|"gpu-sweep"|"environmental-fetcher";
export interface ReproducibilityManifest {
  producer:ManifestProducer;
  git_sha:string;
  git_dirty:boolean;
  seed:number;
  hardware:{hostname:string;cpu_model:string;logical_cpus:number;memory_bytes:number};
  os:{platform:NodeJS.Platform;type:string;release:string;arch:string};
  runtime_versions:Record<string,string>;
  lockfile_sha256:string;
  lockfile_hashes?:Record<string,string>;
  config_sha256:string;
  config:unknown;
  timestamp:string;
  output_checksum_sha256?:string;
  host_class?:"local"|"cluster"|"synthetic";
  host_id?:string;
  device?:string;
  arch?:string;
  numerics_mode?:string;
  blas_backend?:string;
  env_count?:number;
  aggregate_steps_per_s?:number;
}

const stable=(value:unknown):string=>Array.isArray(value)?`[${value.map(stable).join(",")}]`:value&&typeof value==="object"?`{${Object.entries(value).sort(([a],[b])=>a.localeCompare(b)).map(([key,item])=>`${JSON.stringify(key)}:${stable(item)}`).join(",")}}`:JSON.stringify(value);
export const sha256=(value:string|Uint8Array):string=>createHash("sha256").update(value).digest("hex");
export const hashConfig=(config:unknown):string=>sha256(stable(config));

function git(root:string,args:string[]):string{const result=spawnSync("git",args,{cwd:root,encoding:"utf8"});return result.status===0?result.stdout.trim():"unknown";}
export async function createReproducibilityManifest(input:{root:string;seed:number;config:unknown;configSha256?:string;producer:ManifestProducer;timestamp?:string;lockfileHashes?:Record<string,string>;execution?:{host_class:"local"|"cluster"|"synthetic";host_id:string;device:string;arch:string;numerics_mode:string;blas_backend:string;env_count:number;aggregate_steps_per_s?:number}}):Promise<ReproducibilityManifest>{
  const root=resolve(input.root),lockfile=await readFile(resolve(root,"package-lock.json"));
  const cpu=cpus()[0];
  return {producer:input.producer,git_sha:git(root,["rev-parse","HEAD"]),git_dirty:git(root,["status","--porcelain"]).length>0,seed:input.seed,hardware:{hostname:hostname(),cpu_model:cpu?.model??"unknown",logical_cpus:cpus().length,memory_bytes:totalmem()},os:{platform:process.platform,type:type(),release:release(),arch:process.arch},runtime_versions:{node:process.version,v8:process.versions.v8,typescript:process.versions.typescript??"n/a"},lockfile_sha256:sha256(lockfile),lockfile_hashes:input.lockfileHashes,config_sha256:input.configSha256??hashConfig(input.config),config:structuredClone(input.config),timestamp:input.timestamp??new Date().toISOString(),...(input.execution??{})};
}

export function outputChecksum(inventory:Array<{path:string;bytes:number;checksum_sha256:string}>):string{return sha256(stable(inventory.map(({path,bytes,checksum_sha256})=>({path,bytes,checksum_sha256})).sort((a,b)=>a.path.localeCompare(b.path))));}
export function reconstructFromManifest(manifest:ReproducibilityManifest):{seed:number;config:unknown;producer:ManifestProducer}{if(!/^[a-f0-9]{64}$/.test(manifest.config_sha256))throw new Error("Manifest configuration checksum is invalid.");return{seed:manifest.seed,config:structuredClone(manifest.config),producer:manifest.producer};}

// Stable integration points for producers implemented in later stages.
export const createDeterminismSweepManifest=(input:Omit<Parameters<typeof createReproducibilityManifest>[0],"producer">)=>createReproducibilityManifest({...input,producer:"determinism-sweep"});
export const createGpuSweepManifest=(input:Omit<Parameters<typeof createReproducibilityManifest>[0],"producer">)=>createReproducibilityManifest({...input,producer:"gpu-sweep"});
export const createEnvironmentalFetcherManifest=(input:Omit<Parameters<typeof createReproducibilityManifest>[0],"producer">)=>createReproducibilityManifest({...input,producer:"environmental-fetcher"});
