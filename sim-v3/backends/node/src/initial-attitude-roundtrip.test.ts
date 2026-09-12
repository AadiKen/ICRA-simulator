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
{
  const initial:[number,number,number]=[-.681711,.269088,0],initialRate:[number,number,number]=[0,0,.21],yaw=.37;
  const config=resolveExperiment({schema_version:1,experiment:{name:"initial-body-velocity",seed:75,timestep_s:.05,duration_s:2},backend:{type:"node"},vehicle:{preset:"searobotics-surveyor-m1.8",plant:"planar3"},initial_state:{position_ned_m:[10000,10000,0],attitude_rad:[0,0,yaw],body_velocity_mps:initial,angular_rate_body_rad_s:initialRate},mission:{type:"rl-common-waypoint-v1",waypoints:[{north_m:19000,east_m:19000}]},sensors:[]} as any);
  const engine=new LegacyProductionEngine();engine.reset(config);
  const resetTruth:any=engine.getGroundTruth();
  assert.deepEqual(resetTruth.velocity_body_mps,initial,"requested body velocity must survive reset");
  assert.deepEqual(resetTruth.angular_rate_body_rad_s,initialRate,"requested body angular rate must survive reset");
  engine.step({actuators:{effectors:{port:{command:0},starboard:{command:0}}}});
  const steppedTruth:any=engine.getGroundTruth();
  assert.ok(Math.hypot(...steppedTruth.velocity_body_mps.slice(0,2))>.6,"planar adapter erased initial velocity before the first physics step");
  assert.ok(Math.abs(steppedTruth.angular_rate_body_rad_s[2])>.15,"planar adapter erased initial yaw rate before the first physics step");
  engine.dispose();
}
console.log("Explicit initial roll, pitch, and yaw round-trip through the Node backend.");
