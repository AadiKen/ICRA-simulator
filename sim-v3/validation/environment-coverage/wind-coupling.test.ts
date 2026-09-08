import assert from "node:assert/strict";
import {HeadlessMarineSimulation} from "../../packages/core/src/simulation.ts";
import {resolveExperiment} from "../../packages/experiment-schema/src/index.ts";
import {LegacyProductionEngine} from "../../backends/node/src/legacy-production-engine.ts";

function initialAcceleration(wind:[number,number,number]){
  const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());
  sim.reset(resolveExperiment({schema_version:1,experiment:{name:`wind-coupling-${wind.join("-")}`,seed:7319,timestep_s:.05,duration_s:.1},backend:{type:"node"},vehicle:{preset:"vehicle-a-otter",plant:"planar3"},environment:{current_mps:[0,0,0],wind_mps:wind},mission:{type:"waypoint-navigation",waypoints:[{north_m:100,east_m:0}]},sensors:[]} as any));
  sim.step({actuators:{desiredWrench:[0,0,0,0,0,0]}});const truth:any=sim.getGroundTruth();sim.dispose();return truth.acceleration_body_mps2 as number[];
}
const calm=initialAcceleration([0,0,0]),one=initialAcceleration([1,0,0]),two=initialAcceleration([2,0,0]),reverse=initialAcceleration([-1,0,0]),lateral=initialAcceleration([0,1,0]);
const oneEffect=one[0]-calm[0],twoEffect=two[0]-calm[0];
assert.ok(oneEffect>0,"north wind must increase surge acceleration relative to calm");assert.ok(reverse[0]-calm[0]<0,"reversed wind must reverse surge-force direction");assert.ok(lateral[1]-calm[1]>0,"east wind must produce lateral acceleration");assert.ok(Math.abs(twoEffect/oneEffect-4)<.02,`wind response must be approximately quadratic; ratio=${twoEffect/oneEffect}`);
console.log("Vehicle A planar3 wind coupling is nonzero and quadratic.");
