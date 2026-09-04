import assert from "node:assert/strict";
import {resolveExperiment} from "../../../packages/experiment-schema/src/index.ts";
import {LegacyProductionEngine} from "./legacy-production-engine.ts";

for(const yaw of [-.3,-.1,0,.1,.3]){
  const config=resolveExperiment({schema_version:1,experiment:{name:`initial-yaw-${yaw}`,seed:73,timestep_s:.05,duration_s:2},backend:{type:"node"},vehicle:{preset:"searobotics-surveyor-m1.8",plant:"planar3"},initial_state:{position_ned_m:[10000,10000,0],attitude_rad:[.07,-.04,yaw]},mission:{type:"rl-common-waypoint-v1",waypoints:[{north_m:19000,east_m:19000}]},sensors:[]} as any);
  const engine=new LegacyProductionEngine();engine.reset(config);const truth:any=engine.getGroundTruth();
  assert.ok(Math.abs(truth.attitude_rad[0]-.07)<=1e-9,`roll did not round-trip for yaw ${yaw}`);
  assert.ok(Math.abs(truth.attitude_rad[1]+.04)<=1e-9,`pitch did not round-trip for yaw ${yaw}`);
  assert.ok(Math.abs(truth.attitude_rad[2]-yaw)<=1e-9,`yaw did not round-trip for yaw ${yaw}: ${truth.attitude_rad[2]}`);
  engine.dispose();
}
{
  const config=resolveExperiment({schema_version:1,experiment:{name:"legacy-derived-heading",seed:74,timestep_s:.05,duration_s:2},backend:{type:"node"},vehicle:{preset:"searobotics-surveyor-m1.8",plant:"planar3"},initial_state:{position_ned_m:[10000,10000,0]},mission:{type:"rl-common-waypoint-v1",waypoints:[{north_m:10010,east_m:10010}]},sensors:[]} as any);
  const engine=new LegacyProductionEngine();engine.reset(config);const truth:any=engine.getGroundTruth();
  assert.ok(Math.abs(truth.attitude_rad[2]-Math.PI/4)<=1e-9,"legacy waypoint-derived heading changed");engine.dispose();
}
console.log("Explicit initial roll, pitch, and yaw round-trip through the Node backend.");
