import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {losAction,vehicleAWrenchToActuators} from "./portable-controllers.ts";
const action=losAction("LOS-PID-v2",{north_m:0,east_m:0,heading_rad:0,surge_mps:0,yaw_rate_rad_s:0},[0,0],[20,0]);
assert.deepEqual(action,[100,0]);
assert.deepEqual(vehicleAWrenchToActuators([100,0]),[2/3,2/3,0,0]);
for(const file of["container-conversion.json","local-image-resolution.json","portable-controllers.json","vrx-conformance.json","gazebo-conformance.json","vrx-port-status.json","gazebo-port-status.json","vrx-conformance-short.json","gazebo-conformance-short.json","timing-episode-spec.json","timing-comparison-local.json","timing-comparison-cluster-STUB.json","platform-discovery.json"]){const doc=JSON.parse(readFileSync(`artifacts/rl-campaign/${file}`,"utf8"));assert.equal(doc.host_class,"local");assert.equal(doc.paper_measurement_eligible,false);}
const frozen=JSON.parse(readFileSync("artifacts/rl-campaign/surveyor/task-contract-frozen.json","utf8"));const vrx=JSON.parse(readFileSync("artifacts/rl-campaign/vrx-conformance.json","utf8"));assert.equal(vrx.task_contract.content_sha256,frozen.content_sha256);assert.deepEqual(vrx.task_contract.route.unrotated_relative_waypoints_m,frozen.tasks.find((x:any)=>x.task_id==="common-waypoint-transit-v1").reset_randomization.route_relative_m);assert.equal(vrx.result.executed,false);console.log("Portable controllers and honest external-runtime conformance artifacts passed.");
