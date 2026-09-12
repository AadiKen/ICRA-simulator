import {resolveExperiment,type ExperimentV1} from "../../packages/experiment-schema/src/index.ts";
import {executeResolvedExperiment} from "../../packages/mcp-server/src/tools.ts";
export const scenario=(name:string,preset:string,plant:"planar3"|"coupled6",duration_s:number,seed:number,north_m:number,east_m=0,sensors?:string[]):ExperimentV1=>({schema_version:1,experiment:{name,seed,timestep_s:.05,duration_s},backend:{type:"node"},vehicle:{preset,plant},sensors:sensors?.map(plugin=>({plugin})),mission:{type:"waypoint",waypoints:[{north_m,east_m}]},metrics:["completion","propulsion_energy"]});
export async function groundTruth(config:ExperimentV1){return executeResolvedExperiment(resolveExperiment(config));}

