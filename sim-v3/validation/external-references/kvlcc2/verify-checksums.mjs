import {createHash} from "node:crypto";
import {existsSync,readFileSync} from "node:fs";
import {dirname,join} from "node:path";
import {fileURLToPath} from "node:url";

const root=dirname(fileURLToPath(import.meta.url));
const records=readFileSync(join(root,"SHA256SUMS.txt"),"utf8").trim().split(/\n/).map((line)=>{const match=line.match(/^([a-f0-9]{64})  (.+)$/);if(!match)throw new Error(`Malformed SHA256SUMS entry: ${line}`);return{sha256:match[1],path:match[2]};});
for(const record of records){const path=join(root,record.path);if(!existsSync(path))throw new Error(`KVLCC2 cache is absent or incomplete (${record.path}); run fetch_public_assets.sh`);const actual=createHash("sha256").update(readFileSync(path)).digest("hex");if(actual!==record.sha256)throw new Error(`KVLCC2 checksum mismatch for ${record.path}`);}
console.log(`KVLCC2 checksums verified (${records.length} files).`);
