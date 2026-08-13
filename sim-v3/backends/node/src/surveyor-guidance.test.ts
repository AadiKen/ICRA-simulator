import assert from "node:assert/strict";
import {resolveExperiment,type ExperimentV1} from "../../../packages/experiment-schema/src/index.ts";
import {SurveyorGuidanceMapper,validateSurveyorMission} from "../../../packages/vehicle-sdk/src/surveyor.ts";
import {LegacyProductionEngine} from "./legacy-production-engine.ts";

const mission={type:"surveyor-waypoint" as const,origin:{lat:25.7565,lon:-80.3745,heading_deg:0},waypoints:[{lat:25.7568,lon:-80.3745},{lat:25.7568,lon:-80.3741}],erp:{lat:25.7565,lon:-80.3745},max_thrust_command:45};
validateSurveyorMission(mission);
assert.throws(()=>validateSurveyorMission({...mission,erp:undefined}),/ERP/);
const aggressive=new SurveyorGuidanceMapper().map({a:100,w:-100,desiredSpeed:2});assert.deepEqual(aggressive.hardware_command,{mode:"set_thruster_mode",thrust:70,thrust_diff:-70});
const capped=new SurveyorGuidanceMapper({max_thrust_command:30}).map({a:1,w:1,desiredSpeed:2});assert.deepEqual(capped.hardware_command,{mode:"set_thruster_mode",thrust:30,thrust_diff:30});

const config=resolveExperiment({schema_version:1,experiment:{name:"surveyor-waypoint-smoke",seed:73,timestep_s:.1,duration_s:8},backend:{type:"node"},vehicle:{preset:"searobotics-surveyor-m1.8",plant:"planar3"},mission} as ExperimentV1),engine=new LegacyProductionEngine();engine.reset(config);
const trace=[] as Array<{time_s:number;thrust:number;thrust_diff:number;full_wrench:number[]}>;
for(let step=0;step<60;step++){const result=engine.step(null),diagnostics=(result.info as any).vehicle_diagnostics,command=diagnostics.hardware_command;assert.equal(command.mode,"set_thruster_mode");assert.ok(Number.isInteger(command.thrust)&&Math.abs(command.thrust)<=45);assert.ok(Number.isInteger(command.thrust_diff)&&Math.abs(command.thrust_diff)<=45);trace.push({time_s:result.observation.time_s,thrust:command.thrust,thrust_diff:command.thrust_diff,full_wrench:diagnostics.full_wrench});}
assert.ok(trace.some(row=>row.thrust!==0),"Waypoint mission must produce nonzero Surveyor thrust.");assert.ok(trace.some(row=>row.thrust_diff!==0),"Waypoint mission must produce differential steering.");
console.log(JSON.stringify({message:"Surveyor hardware-command guidance tests passed.",sample_trace:trace.filter((_,index)=>index%10===0)},null,2));engine.dispose();
