import assert from "node:assert/strict";
import {mkdtemp,readFile} from "node:fs/promises";
import {tmpdir} from "node:os";
import {join} from "node:path";
import {RunArtifactWriter} from "../src/run-artifacts.ts";

const root=await mkdtemp(join(tmpdir(),"bcod-artifact-"));
const writer=new RunArtifactWriter(root,"run-001");
await writer.initialize({
  schema_version:1,run_id:"run-001",experiment_checksum:"b".repeat(64),created_at:"2026-07-31T00:00:00Z",software:{node:"24"},seeds:[42],inputs:[],validation_scope:{vehicle:"validated"},warnings:[],
  backend_capabilities:{node:"passed",tensor_mps:"blocked-external-infrastructure"},sensor_retention:{mode:"summary",raw_plugins:[],max_bytes_per_run:1000},failure_summary:{count:0,categories:[]}
},{name:"original"},{name:"resolved"});
await writer.appendEvent({time_s:0,type:"RESET"});await writer.writeReplay({version:1});await writer.finalize({success:true,failure_reason:null,completion_time_s:1,metrics:{completion:true}});
const manifest=JSON.parse(await readFile(join(writer.directory,"manifest.json"),"utf8"));
assert.equal(manifest.run_id,"run-001");
assert.ok(manifest.artifact_inventory.some((item:any)=>item.path==="events.jsonl"&&/^[a-f0-9]{64}$/.test(item.checksum_sha256)));
assert.ok(!manifest.artifact_inventory.some((item:any)=>item.path==="manifest.json"));
assert.throws(()=>new RunArtifactWriter(root,"../escape"),/unsafe|escapes/);
console.log("Run artifact tests passed.");
