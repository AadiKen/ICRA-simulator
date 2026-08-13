import assert from "node:assert/strict";
import {DeterministicVectorMarineSimulation,HeadlessMarineSimulation,type SimulationEngine} from "../src/simulation.ts";
import {resolveExperiment,type ExperimentV1} from "../../experiment-schema/src/index.ts";

class Engine implements SimulationEngine{
  stepIndex=0;limit=2;seed=0;
  reset(config:any){this.stepIndex=0;this.seed=config.experiment.seed;return this.observation();}
  step(action:any){this.stepIndex++;return{observation:this.observation(),reward:Number(action??0),terminated:this.stepIndex>=this.limit,truncated:false,info:{seed:this.seed}};}
  observation(){return{time_s:this.stepIndex*0.1,sensors:{seed:this.seed}};}
  getGroundTruth(){return{step:this.stepIndex};}getMetrics(){return{elapsed_s:this.stepIndex*0.1};}
  saveState(){return{stepIndex:this.stepIndex,seed:this.seed};}loadState(state:any){this.stepIndex=state.stepIndex;this.seed=state.seed;}dispose(){}
}
const config=(seed:number)=>resolveExperiment({schema_version:1,experiment:{name:`vector-${seed}`,seed,timestep_s:0.1,duration_s:1},backend:{type:"node"},vehicle:{preset:"vehicle-a-otter",plant:"planar3"},mission:{type:"hold"}} satisfies ExperimentV1);
const vector=new DeterministicVectorMarineSimulation(3,()=>new HeadlessMarineSimulation(new Engine()));
assert.deepEqual(vector.reset([config(1),config(2),config(3)]).map((v)=>v.sensors.seed),[1,2,3]);
const first=vector.step([1,2,3],[true,false,true]);assert.deepEqual(first.rewards,[1,0,3]);assert.equal(first.infos[1].masked,true);
const checkpoint=vector.saveCheckpoint();const terminal=vector.step([1,2,3],[true,false,true]);assert.deepEqual(terminal.terminated,[true,false,true]);
vector.loadCheckpoint(checkpoint);assert.deepEqual(vector.step([1,2,3],[true,false,true]),terminal);
assert.throws(()=>vector.step([1,2,3],[true,false,true]),/inactive/);
const reset=vector.reset([config(11),config(12),config(13)],[true,false,true]);assert.deepEqual(reset.map((v)=>v.sensors.seed),[11,2,13]);
vector.dispose();assert.throws(()=>vector.step([0,0,0]),/disposed/);
console.log("Vector simulation tests passed.");
