import {existsSync,readFileSync} from "node:fs";
import {spawnSync} from "node:child_process";
const manifest=JSON.parse(readFileSync(new URL("source-manifest.json",import.meta.url),"utf8")),root=new URL("../../",import.meta.url),cache=new URL(`../../${manifest.cache_path}/`,import.meta.url);
if(!existsSync(cache)){const clone=spawnSync("git",["clone","--branch",manifest.tag,"--depth","1",manifest.source,cache.pathname],{cwd:root,stdio:"inherit"});if(clone.status!==0)process.exit(clone.status??1);}
const revision=spawnSync("git",["-C",cache.pathname,"rev-parse","HEAD"],{encoding:"utf8"});if(revision.status!==0||revision.stdout.trim()!==manifest.commit)throw new Error(`VRX revision mismatch: expected ${manifest.commit}, got ${revision.stdout.trim()||revision.stderr.trim()}`);
console.log(`VRX ${manifest.tag} cache verified at ${manifest.commit}.`);
