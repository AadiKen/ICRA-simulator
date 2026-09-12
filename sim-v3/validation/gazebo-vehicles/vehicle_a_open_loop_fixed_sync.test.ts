import assert from "node:assert/strict";
import {readFile} from "node:fs/promises";
const artifact=JSON.parse(await readFile("artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison-iteration-synchronized.json","utf8"));
assert.deepEqual(artifact.scenarios.map((x:any)=>x.id),["constant-thrust","turning-circle","yaw-turn","zig-zag","coast-down","current-drift","impulse-hold"]);
assert.equal(artifact.vehicle,"vehicle-a-otter-reference");assert.equal(artifact.plant_match.bcod_sim_added_mass,false);assert.equal(artifact.plant_match.gazebo_added_mass,false);
assert.equal(artifact.timing.physics_timestep_s,.05);assert.equal(artifact.timing.sample_interval_s,.05);assert.equal(artifact.timing.physics_iterations_per_sample,1);
for(const scenario of artifact.scenarios){
  assert.equal(scenario.synchronization.blind_retries,false);assert.equal(scenario.synchronization.step_requests_per_target,1);assert.equal(scenario.synchronization.asserted_iteration_delta,1);assert.equal(scenario.synchronization.asserted_sim_time_delta_s,.05);
  assert.equal(scenario.bcod_sim.length,scenario.gazebo_harmonic.length);assert.equal(scenario.bcod_sim.length,scenario.error_time_series.length);
  assert.ok(scenario.gazebo_harmonic.every((row:any,index:number)=>Number.isInteger(row.gazebo_iteration)&&Math.abs(row.time_s-(index+1)*.05)<1e-9));
  assert.ok(Number.isFinite(scenario.summary.relative_position_error));
}
console.log("Vehicle A iteration-synchronized comparison passed");
