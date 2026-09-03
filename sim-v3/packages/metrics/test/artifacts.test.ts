import assert from "node:assert/strict";
import {mkdtemp,readFile,writeFile,mkdir} from "node:fs/promises";
import {tmpdir} from "node:os";
import {join,resolve} from "node:path";
import {RunArtifactWriter} from "../src/run-artifacts.ts";
import {createReproducibilityManifest,reconstructFromManifest} from "../src/reproducibility-manifest.ts";
import {guardArtifactTree} from "../src/manifest-guard.ts";

const root=await mkdtemp(join(tmpdir(),"bcod-artifact-"));
const writer=new RunArtifactWriter(root,"run-001");
const config={name:"resolved",vehicle:"vehicle-a",seed:42},checksum="b".repeat(64);
const reproducibility=await createReproducibilityManifest({root:resolve(new URL("../../../",import.meta.url).pathname),seed:42,config,configSha256:checksum,producer:"benchmark",timestamp:"2026-07-31T00:00:00Z"});
await writer.initialize({
  schema_version:1,run_id:"run-001",experiment_checksum:checksum,created_at:"2026-07-31T00:00:00Z",software:{node:"24"},seeds:[42],inputs:[],validation_scope:{vehicle:"validated"},warnings:[],reproducibility,
  backend_capabilities:{node:"passed",tensor_mps:"blocked-external-infrastructure"},sensor_retention:{mode:"summary",raw_plugins:[],max_bytes_per_run:1000},failure_summary:{count:0,categories:[]}
},{name:"original"},config);
await writer.appendEvent({time_s:0,type:"RESET"});await writer.writeReplay({version:1});await writer.finalize({success:true,failure_reason:null,completion_time_s:1,metrics:{completion:true}});
const manifest=JSON.parse(await readFile(join(writer.directory,"manifest.json"),"utf8"));
assert.equal(manifest.run_id,"run-001");
assert.ok(manifest.artifact_inventory.some((item:any)=>item.path==="events.jsonl"&&/^[a-f0-9]{64}$/.test(item.checksum_sha256)));
assert.ok(!manifest.artifact_inventory.some((item:any)=>item.path==="manifest.json"));
assert.match(manifest.reproducibility.output_checksum_sha256,/^[a-f0-9]{64}$/);
assert.deepEqual(reconstructFromManifest(manifest.reproducibility),{seed:42,config,producer:"benchmark"});
assert.throws(()=>new RunArtifactWriter(root,"../escape"),/unsafe|escapes/);
const missing=join(root,"missing");await mkdir(missing);await writeFile(join(missing,"state.jsonl"),"{}\n");await assert.rejects(()=>guardArtifactTree(missing),/without manifest/);
console.log("Run artifact tests passed.");
