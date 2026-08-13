import assert from "node:assert/strict";
import {resolveExperiment,type ExperimentV1} from "../../../packages/experiment-schema/src/index.ts";
import {runExperimentsInWorkers} from "./worker-runner.ts";

const resolved=(id:string,seed:number,preset="vehicle-a-otter",plant:"planar3"|"coupled6"="planar3")=>resolveExperiment({schema_version:1,experiment:{name:id,seed,timestep_s:.05,duration_s:.2},backend:{type:"node"},vehicle:{preset,plant},mission:{type:"hold",waypoints:[{north_m:1,east_m:0}]}} as ExperimentV1);
const jobs=[{id:"a",config:resolved("a",11),actions:[{},{}]},{id:"bad",config:resolved("bad",12,"vehicle-b-rudder","planar3"),actions:[{}]},{id:"b",config:resolved("b",13),actions:[{},{}]}];
const first=await runExperimentsInWorkers(jobs,{concurrency:2}),second=await runExperimentsInWorkers(jobs,{concurrency:3});
assert.equal(first[0].ok,true);assert.equal(first[1].ok,false);assert.match(first[1].error!,/requires coupled6/i);assert.equal(first[2].ok,true);
assert.deepEqual(first,second);assert.deepEqual(first.map((result)=>result.id),jobs.map((job)=>job.id));
console.log("Worker runner tests passed.");
