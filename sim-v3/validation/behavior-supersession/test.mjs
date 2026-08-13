import assert from "node:assert/strict";
import {createHash} from "node:crypto";
import {readFileSync} from "node:fs";
import {spawnSync} from "node:child_process";
import {join,resolve} from "node:path";

const root=resolve(new URL("../../",import.meta.url).pathname),supplemental=join(root,"validation/pre-migration/supplemental");
const manifest=JSON.parse(readFileSync(join(supplemental,"manifest.json"),"utf8")),record=manifest.records.find(({file})=>file==="actuator-legacy-trace.json");
assert.ok(record);assert.equal(createHash("sha256").update(readFileSync(join(supplemental,record.file))).digest("hex"),record.sha256);
const refused=spawnSync(process.execPath,[join(root,"validation/behavior-supersession/create.mjs")],{cwd:root,encoding:"utf8"});
assert.notEqual(refused.status,0);assert.match(refused.stderr,/supersession is forbidden/);
console.log("Behavior supersession defaults to forbidden and legacy checksum is intact.");
