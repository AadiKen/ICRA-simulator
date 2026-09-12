import {randomUUID} from "node:crypto";
import {readFile} from "node:fs/promises";
import {join} from "node:path";
import {resolveExperiment,validateExperiment,type ExperimentV1,type ResolvedExperimentV1} from "../../experiment-schema/src/index.ts";
import {VEHICLES} from "../../vehicle-sdk/src/index.ts";
import {BUILT_IN_SENSOR_CAPABILITIES} from "../../sensor-sdk/src/index.ts";
import {HeadlessMarineSimulation} from "../../core/src/simulation.ts";
import {LegacyProductionEngine} from "../../../backends/node/src/legacy-production-engine.ts";
import {RunArtifactWriter} from "../../metrics/src/run-artifacts.ts";
import type {RunManifestV1,RunSummary} from "../../metrics/src/index.ts";
import {ScenarioStore} from "./scenario-store.ts";

export type ErrorResult={error:string};
export const defaultStore=new ScenarioStore();
export function listVehiclePresets(){return Object.entries(VEHICLES).map(([id,v])=>({id,name:v.name,plant:v.plant,validation_status:v.validation.status,validation_claim:v.validation.claim}));}
export function listSensorPlugins(){return BUILT_IN_SENSOR_CAPABILITIES.map(x=>structuredClone(x));}
const message=(error:unknown)=>error instanceof Error?error.message:String(error);
export function validateScenario(config:ExperimentV1):{valid:true}|{valid:false;error:string}{try{validateExperiment(config);return{valid:true};}catch(error){return{valid:false,error:message(error)};}}
export function constructScenario(config:ExperimentV1,store=defaultStore){try{const resolved=resolveExperiment(config),scenario_id=`scenario-${resolved.resolution.checksum_sha256.slice(0,16)}`;store.scenarios.set(scenario_id,resolved);return{scenario_id,checksum_sha256:resolved.resolution.checksum_sha256,warnings:[...resolved.resolution.warnings]};}catch(error){return{valid:false as const,error:message(error)};}}

export async function executeResolvedExperiment(config:ResolvedExperimentV1){
  const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());let result;let steps=0;
  try{sim.reset(config);const cap=Math.ceil(config.experiment.duration_s/config.experiment.timestep_s)+1;do{result=sim.step(null);steps++;}while(!result.terminated&&!result.truncated&&steps<cap);const truth=sim.getGroundTruth() as {time_s:number},raw=sim.getMetrics();return{success:Boolean(result.terminated||result.truncated),completion_time_s:truth.time_s,metrics:{...raw,propulsion_energy:raw.movement_cost}};}finally{sim.dispose();}
}
export async function runScenario({scenario_id}:{scenario_id:string},store=defaultStore){
  const config=store.scenarios.get(scenario_id);if(!config)return{error:`Unknown scenario_id: ${scenario_id}`};
  try{const run_id=`run-${randomUUID()}`,root=await store.artifactRoot(),writer=new RunArtifactWriter(root,run_id),manifest:RunManifestV1={schema_version:1,run_id,experiment_checksum:config.resolution.checksum_sha256,created_at:new Date().toISOString(),software:{node:process.version,physics:"legacy-production-adapter"},seeds:[config.experiment.seed],inputs:[],validation_scope:{vehicle:VEHICLES[config.vehicle.preset]?.validation.claim??"registry preset",api:"MCP scenario construction"},warnings:[...config.resolution.warnings]};await writer.initialize(manifest,config,config);const outcome=await executeResolvedExperiment(config);await writer.finalize({success:outcome.success,failure_reason:outcome.success?null:"Simulation did not terminate",completion_time_s:outcome.completion_time_s,metrics:outcome.metrics});store.runs.set(run_id,{manifest,metrics:outcome.metrics,directory:writer.directory});return{run_id,...outcome};}catch(error){return{error:message(error)};}
}
export async function getRunMetrics({run_id}:{run_id:string},store=defaultStore){const run=store.runs.get(run_id);if(!run)return{error:`Unknown run_id: ${run_id}`};try{const summary=JSON.parse(await readFile(join(run.directory,"summary.json"),"utf8")) as RunSummary;return{run_id,success:summary.success,completion_time_s:summary.completion_time_s,metrics:summary.metrics};}catch(error){return{error:message(error)};}}
