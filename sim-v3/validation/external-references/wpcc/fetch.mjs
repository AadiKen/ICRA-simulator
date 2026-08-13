#!/usr/bin/env node
import {mkdir,writeFile} from "node:fs/promises";
import path from "node:path";
const destination=path.resolve("validation/datasets/raw/wpcc-v4");
const endpoint="https://api.mendeley.com/datasets/j5zdrhr9bf/versions/4/files";
try{
  const response=await fetch(endpoint,{headers:{Accept:"application/json"}});
  if(!response.ok)throw new Error(`Mendeley file API returned ${response.status}`);
  const files=await response.json();
  if(!Array.isArray(files)||files.length===0)throw new Error("Mendeley returned no version-4 files");
  await mkdir(destination,{recursive:true});
  for(const file of files){
    const name=String(file.filename??file.name??"");
    const url=file.download_url??file.content_details?.download_url;
    if(!name||name.includes("..")||path.isAbsolute(name)||!url)throw new Error("Unsafe or incomplete Mendeley file record");
    const download=await fetch(url);if(!download.ok)throw new Error(`Download failed for ${name}: ${download.status}`);
    const target=path.join(destination,name);await mkdir(path.dirname(target),{recursive:true});await writeFile(target,Buffer.from(await download.arrayBuffer()));
  }
  console.log(`wPCC v4: fetched ${files.length} files into ignored cache ${destination}`);
}catch(error){
  console.error(`wPCC v4 cache absent/unavailable: ${error.message}`);
  console.error("Supply the DOI 10.17632/j5zdrhr9bf.4 archive, extract it under validation/datasets/raw/wpcc-v4, then run npm run verify:wpcc.");
  process.exitCode=1;
}
