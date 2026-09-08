import assert from "node:assert/strict";
import {generate} from "./generate-environment-sweep.ts";
const report=generate();
assert.equal(report.artifact_kind,"environment-effect-sweep");
assert.equal(report.arms.length,5);
assert.equal(report.arms.find(x=>x.id==="idealized-zero")?.final_separation_from_idealized_m,0);
assert.ok(report.arms.every(x=>x.samples.length===180));
assert.ok(report.checks.controller_requests_within_limits);
console.log("Environment-effect sweep generator contract passed.");
