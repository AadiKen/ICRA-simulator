import {writeFileSync,mkdirSync} from "node:fs";
import {dirname} from "node:path";
import type {FrozenAction} from "./shared-actuators.ts";

export function commandAt(step:number):FrozenAction {return [Math.max(-1,Math.min(1,.55*Math.sin(.013*step)+.25*Math.cos(.031*step))),Math.max(-1,Math.min(1,.50*Math.sin(.017*step+.4)-.20*Math.cos(.029*step))),0,0];}
export function fixedActionTrace(steps=2400){return Array.from({length:steps/2},(_,step)=>({step,time_s:step*.1,command:commandAt(step*2)}));}
if(process.argv[1]?.endsWith("frozen-conformance-trace.ts")){const target=process.argv[2];if(!target)throw new Error("output path required");mkdirSync(dirname(target),{recursive:true});writeFileSync(target,JSON.stringify({schema_version:"trace-schema-v2-action-trace",physics_timestep_s:.05,control_interval_s:.1,actions:fixedActionTrace()},null,2)+"\n");}
