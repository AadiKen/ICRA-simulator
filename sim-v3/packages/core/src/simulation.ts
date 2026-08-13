import type {ResolvedExperimentV1} from "../../experiment-schema/src/index.ts";

export interface Observation {time_s: number; sensors: Record<string, unknown>; estimated_state?: unknown}
export interface StepResult {observation: Observation; reward: number; terminated: boolean; truncated: boolean; info: Record<string, unknown>}
export interface SimulationCheckpoint {version: 1; config_checksum: string; payload: unknown}
export interface SimulationEngine {
  reset(config: ResolvedExperimentV1): Observation;
  step(action: unknown): StepResult;
  getGroundTruth(): unknown;
  getMetrics(): Record<string, number>;
  saveState(): unknown;
  loadState(state: unknown): void;
  dispose(): void;
}
export interface MarineSimulation {
  reset(config: ResolvedExperimentV1): Observation;
  step(action: unknown): StepResult;
  getGroundTruth(): unknown;
  getMetrics(): Record<string, number>;
  saveCheckpoint(): SimulationCheckpoint;
  loadCheckpoint(checkpoint: SimulationCheckpoint): void;
  pause(): void;
  resume(): void;
  dispose(): void;
}

export interface VectorStepResult {
  observations: Observation[];
  rewards: number[];
  terminated: boolean[];
  truncated: boolean[];
  infos: Record<string, unknown>[];
}
export interface VectorSimulationCheckpoint {version: 1; checkpoints: SimulationCheckpoint[]; active: boolean[]; observations: Observation[]}
export interface VectorMarineSimulation {
  readonly size: number;
  reset(configs: ResolvedExperimentV1[], mask?: boolean[]): Observation[];
  step(actions: unknown[], mask?: boolean[]): VectorStepResult;
  saveCheckpoint(): VectorSimulationCheckpoint;
  loadCheckpoint(checkpoint: VectorSimulationCheckpoint): void;
  dispose(): void;
}

export class DeterministicVectorMarineSimulation implements VectorMarineSimulation {
  readonly size: number;
  #simulations: MarineSimulation[];
  #active: boolean[];
  #observations: Observation[];
  #disposed = false;
  constructor(size: number, factory: (index: number) => MarineSimulation) {
    if (!Number.isInteger(size) || size < 1) throw new Error("Vector size must be a positive integer.");
    this.size=size;this.#simulations=Array.from({length:size},(_,index)=>factory(index));this.#active=Array(size).fill(false);this.#observations=Array(size);
  }
  reset(configs: ResolvedExperimentV1[], mask: boolean[] = Array(this.size).fill(true)): Observation[] {
    this.#assertLive();this.#assertLength(configs,"configs");this.#assertMask(mask);
    for(let i=0;i<this.size;i++)if(mask[i]){this.#observations[i]=this.#simulations[i].reset(configs[i]);this.#active[i]=true;}
    if(this.#observations.some((value)=>value===undefined))throw new Error("Every vector slot must be reset before a masked reset can return observations.");
    return structuredClone(this.#observations);
  }
  step(actions: unknown[], mask: boolean[] = [...this.#active]): VectorStepResult {
    this.#assertLive();this.#assertLength(actions,"actions");this.#assertMask(mask);
    const rewards=Array(this.size).fill(0),terminated=Array(this.size).fill(false),truncated=Array(this.size).fill(false),infos=Array.from({length:this.size},()=>({} as Record<string,unknown>));
    for(let i=0;i<this.size;i++){
      if(!mask[i]){infos[i]={masked:true,active:this.#active[i]};continue;}
      if(!this.#active[i])throw new Error(`Vector slot ${i} is inactive and must be reset before stepping.`);
      const result=this.#simulations[i].step(actions[i]);this.#observations[i]=result.observation;rewards[i]=result.reward;terminated[i]=result.terminated;truncated[i]=result.truncated;infos[i]=result.info;
      if(result.terminated||result.truncated)this.#active[i]=false;
    }
    return{observations:structuredClone(this.#observations),rewards,terminated,truncated,infos};
  }
  saveCheckpoint(): VectorSimulationCheckpoint {this.#assertLive();return{version:1,checkpoints:this.#simulations.map((simulation,index)=>{if(!this.#observations[index])throw new Error(`Vector slot ${index} has not been reset.`);return simulation.saveCheckpoint();}),active:[...this.#active],observations:structuredClone(this.#observations)};}
  loadCheckpoint(checkpoint: VectorSimulationCheckpoint): void {this.#assertLive();if(checkpoint.version!==1||checkpoint.checkpoints.length!==this.size||checkpoint.active.length!==this.size||checkpoint.observations.length!==this.size)throw new Error("Vector checkpoint is incompatible.");checkpoint.checkpoints.forEach((value,index)=>this.#simulations[index].loadCheckpoint(value));this.#active=[...checkpoint.active];this.#observations=structuredClone(checkpoint.observations);}
  dispose():void{if(!this.#disposed)this.#simulations.forEach((simulation)=>simulation.dispose());this.#disposed=true;this.#active.fill(false);}
  #assertLength(value:unknown[],name:string){if(value.length!==this.size)throw new Error(`${name} must contain exactly ${this.size} entries.`);}
  #assertMask(mask:boolean[]){this.#assertLength(mask,"mask");if(mask.some((value)=>typeof value!=="boolean"))throw new Error("mask entries must be boolean.");}
  #assertLive(){if(this.#disposed)throw new Error("Vector simulation has been disposed.");}
}

export function assertCheckpointNumbersRoundTrip(value: unknown, path = "checkpoint"): void {
  if (typeof value === "number") {
    if (Number.isNaN(value)) throw new Error(`${path} contains NaN, indicating invalid simulation state.`);
    return;
  }
  if (Array.isArray(value)) { value.forEach((entry, index) => assertCheckpointNumbersRoundTrip(entry, `${path}[${index}]`)); return; }
  if (value && typeof value === "object") for (const [key, entry] of Object.entries(value)) assertCheckpointNumbersRoundTrip(entry, `${path}.${key}`);
}
type EncodedNumber = {__bcod_number: "-0"|"Infinity"|"-Infinity"};
function encodeCheckpointPayload(value: unknown): unknown {
  assertCheckpointNumbersRoundTrip(value);
  if (typeof value === "number") {
    if(Object.is(value,-0))return {__bcod_number:"-0"} satisfies EncodedNumber;
    if(value===Infinity)return {__bcod_number:"Infinity"} satisfies EncodedNumber;
    if(value===-Infinity)return {__bcod_number:"-Infinity"} satisfies EncodedNumber;
    return value;
  }
  if (Array.isArray(value)) return value.map(encodeCheckpointPayload);
  if (value && typeof value === "object") return Object.fromEntries(Object.entries(value).map(([key,entry])=>[key,encodeCheckpointPayload(entry)]));
  return value;
}
function decodeCheckpointPayload(value: unknown): unknown {
  if (value && typeof value === "object" && !Array.isArray(value) && "__bcod_number" in value) {
    const encoded=(value as EncodedNumber).__bcod_number;if(encoded==="-0")return -0;if(encoded==="Infinity")return Infinity;if(encoded==="-Infinity")return -Infinity;
  }
  if (Array.isArray(value)) return value.map(decodeCheckpointPayload);
  if (value && typeof value === "object") return Object.fromEntries(Object.entries(value).map(([key,entry])=>[key,decodeCheckpointPayload(entry)]));
  return value;
}

export class HeadlessMarineSimulation implements MarineSimulation {
  #config?: ResolvedExperimentV1;
  #paused = false;
  #disposed = false;
  private readonly engine: SimulationEngine;
  constructor(engine: SimulationEngine) { this.engine = engine; }
  reset(config: ResolvedExperimentV1): Observation { this.#assertLive(); this.#config=structuredClone(config); this.#paused=false; return this.engine.reset(config); }
  step(action: unknown): StepResult { this.#assertReady(); if(this.#paused) throw new Error("Simulation is paused."); return this.engine.step(action); }
  getGroundTruth(): unknown { this.#assertReady(); return this.engine.getGroundTruth(); }
  getMetrics(): Record<string, number> { this.#assertReady(); return this.engine.getMetrics(); }
  saveCheckpoint(): SimulationCheckpoint { this.#assertReady(); const payload=encodeCheckpointPayload(structuredClone(this.engine.saveState()));return {version:1,config_checksum:this.#config!.resolution.checksum_sha256,payload}; }
  loadCheckpoint(checkpoint: SimulationCheckpoint): void { this.#assertReady(); if(checkpoint.version!==1||checkpoint.config_checksum!==this.#config!.resolution.checksum_sha256) throw new Error("Checkpoint is incompatible with the resolved experiment."); this.engine.loadState(decodeCheckpointPayload(structuredClone(checkpoint.payload))); }
  pause(): void { this.#assertReady(); this.#paused=true; }
  resume(): void { this.#assertReady(); this.#paused=false; }
  dispose(): void { if(!this.#disposed)this.engine.dispose(); this.#disposed=true; this.#config=undefined; }
  #assertLive(): void { if(this.#disposed) throw new Error("Simulation has been disposed."); }
  #assertReady(): void { this.#assertLive(); if(!this.#config) throw new Error("Simulation must be reset before use."); }
}
