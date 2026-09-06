import assert from "node:assert/strict";
import {createHash} from "node:crypto";
import {readFile} from "node:fs/promises";

const stable=(v:unknown):string=>Array.isArray(v)?`[${v.map(stable).join(",")}]`:v&&typeof v==="object"?`{${Object.entries(v as Record<string,unknown>).sort(([a],[b])=>a.localeCompare(b)).map(([k,x])=>`${JSON.stringify(k)}:${stable(x)}`).join(",")}}`:JSON.stringify(v);
const document=JSON.parse(await readFile("artifacts/rl-campaign/surveyor/task-contract-frozen.json","utf8")),{content_sha256,hash_scope,...contract}=document;
assert.equal(createHash("sha256").update(stable(contract)).digest("hex"),content_sha256);
assert.equal(content_sha256,"2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36");
const common=contract.tasks.find((task:any)=>task.task_id==="common-waypoint-transit-v1"),components=common.observation.components;
assert.equal(components.reduce((sum:number,x:any)=>sum+x.size,0),15);
assert.deepEqual(components.find((x:any)=>x.id==="gps").fields,["relative_goal_north_m","relative_goal_east_m","fix_valid"]);
assert.deepEqual(contract.observation_revision.removed_fields,["gps.ground_speed_north_m_s","gps.ground_speed_east_m_s"]);
assert.equal(contract.observation_revision.diagnosis_artifact,"artifacts/rl-campaign/gps-velocity-parity-diagnosis.json");
console.log("Frozen Surveyor 15-field contract and rationale passed.");
