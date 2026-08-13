import assert from "node:assert/strict";
import {HeadlessMarineSimulation} from "../../../packages/core/src/simulation.ts";
import {resolveExperiment} from "../../../packages/experiment-schema/src/index.ts";
import {LegacyProductionEngine} from "./legacy-production-engine.ts";

const config=resolveExperiment({schema_version:1,experiment:{name:"mid-lag-replay",seed:7,timestep_s:0.02,duration_s:10},backend:{type:"node"},vehicle:{preset:"vehicle-a-otter",plant:"planar3"},mission:{type:"waypoint",waypoints:[{north_m:80,east_m:80}]}});
const engine=new LegacyProductionEngine();
const sim=new HeadlessMarineSimulation(engine);
sim.reset(config);
for(let step=0;step<3;step++)sim.step(null);
const checkpoint=JSON.parse(JSON.stringify(sim.saveCheckpoint()));
const actuator=(checkpoint.payload as any).actuatorState.effectors;
assert(actuator.some((effector:any)=>Math.abs(effector.value)>0),"Checkpoint must be taken after lag state begins moving.");
const randomFirst=[engine.nextRandomForService(),engine.nextRandomForService(),engine.nextRandomForService()];
const first=[];for(let step=0;step<12;step++){sim.step(null);first.push(structuredClone(sim.getGroundTruth()));}
sim.loadCheckpoint(checkpoint);
const randomReplay=[engine.nextRandomForService(),engine.nextRandomForService(),engine.nextRandomForService()];
assert.deepEqual(randomReplay,randomFirst,"Seeded RNG sequence must resume bit-exactly after restore.");
const replay=[];for(let step=0;step<12;step++){sim.step(null);replay.push(structuredClone(sim.getGroundTruth()));}
assert.deepEqual(replay,first,"Mid-lag checkpoint replay must be bit-exact.");
sim.dispose();
console.log("Mid-lag actuator checkpoint replay test passed.");
