import assert from "node:assert/strict";
import {manifest,scenarios} from "../manifest.ts";
assert.equal(manifest.base_scenario_count,36);assert.equal(manifest.configuration_count,108);assert.equal(new Set(scenarios.map((v)=>v.id)).size,36);
for(const axis of ["maneuvering","environment","sensing","resources","geography","mission-integration"])assert.equal(scenarios.filter((v)=>v.axis===axis).length,6);
assert.ok(scenarios.every((v)=>v.severity.score>=0&&v.severity.score<=1));
console.log("USV-Bench-36 manifest tests passed.");
