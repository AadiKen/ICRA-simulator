import assert from "node:assert/strict";
import {assertCheckpointNumbersRoundTrip,HeadlessMarineSimulation, type SimulationEngine} from "../src/simulation.ts";
import {resolveExperiment} from "../../experiment-schema/src/index.ts";
import {controlCommand,controlWaypoint,skipperModel,vec3} from "../../../schema.js";
import {legacyGuidanceToActuatorCommand} from "../../../adapters/legacyGuidanceAdapter.js";

let state={time_s:0};
const engine:SimulationEngine={
  reset:()=>{state={time_s:0};return {time_s:0,sensors:{gps:{x:0}}};},
  step:()=>{state.time_s+=1;return {observation:{time_s:state.time_s,sensors:{}},reward:0,terminated:false,truncated:false,info:{}};},
  getGroundTruth:()=>({...state}),getMetrics:()=>({steps:state.time_s}),saveState:()=>({...state}),loadState:(next)=>{state=next as typeof state;},dispose:()=>{}
};
const config=resolveExperiment({schema_version:1,experiment:{name:"test",seed:1,timestep_s:1,duration_s:10},backend:{type:"node"},vehicle:{preset:"otter",plant:"planar3"},mission:{type:"hold"}});
const sim=new HeadlessMarineSimulation(engine);
assert.equal(sim.reset(config).time_s,0);
sim.step({});const checkpoint=sim.saveCheckpoint();sim.step({});sim.loadCheckpoint(checkpoint);
assert.equal((sim.getGroundTruth() as typeof state).time_s,1);
sim.pause();assert.throws(()=>sim.step({}),/paused/);sim.resume();sim.step({});
sim.dispose();assert.throws(()=>sim.getMetrics(),/disposed/);
assert.throws(()=>assertCheckpointNumbersRoundTrip({value:NaN}),/NaN/);
const skipper=new skipperModel({maxSpeed:3,maxAcceleration:1,maxDeceleration:1,maxTurn:.5,maxAngularVelocity:{y:1}},{guidanceMode:"relative"}),wrapped=new controlCommand([new controlWaypoint(new vec3(1,0,12),[])],[]),guidance=skipper.getGuidance(wrapped,{goal:{tolerance:1,waypoints:[]},boatBelief:{pos:new vec3(12,0,12),velocity:new vec3(0,0,0),angularVel:new vec3(0,0,0),heading:-Math.PI/2}});assert.ok(Number.isFinite(guidance.a)&&guidance.a>0,"Wrapped controlWaypoint.pos must reach guidance as a finite acceleration command.");assert.throws(()=>legacyGuidanceToActuatorCommand({a:NaN,w:0},{massProps:{mass:1,inertia:{Iz:1}},actuator:{maxThrust:1,beam:1}},{}),/finite acceleration/);
console.log("Simulation contract tests passed.");
