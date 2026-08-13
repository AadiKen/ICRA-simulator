import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {run} from "./run.ts";
const artifact=run();assert.equal(artifact.status,"passed-policy-software-evaluation-not-physical-validation");assert.equal(artifact.base_scenario_count,36);assert.equal(artifact.vehicle_configuration_count,108);assert.equal(artifact.policy_configuration_count,324);assert.equal(artifact.aggregate.length,3);assert.ok(artifact.rows.every((row)=>row.finite&&row.steps>0));assert.deepEqual(JSON.parse(readFileSync("artifacts/baselines/usv-bench-policy-report.json","utf8")),artifact);console.log("USV-Bench-36 policy campaign tests passed.");
