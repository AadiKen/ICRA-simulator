import {PublishedSensorsService,type PublishedSensorReader} from "./services/published-sensors.ts";
import type {PlatformStateService} from "./services/platform-state.ts";
import type {SkyOcclusionService} from "./services/sky-occlusion.ts";
import type {BathymetryField} from "@bcod/environment";
export type {PublishedSensorReader} from "./services/published-sensors.ts";

export type JSONPrimitive = string | number | boolean | null;
export type JSONValue = JSONPrimitive | JSONValue[] | {[key:string]:JSONValue};
export type SerializableState = JSONValue;
export type Vec3 = [number,number,number];

export type LifecycleState="OFF"|"STARTING"|"WARMING"|"ACTIVE"|"DEGRADED"|"FAILED"|"COOLDOWN";
export type SensorStatus=LifecycleState;
export type SensorDomain="LIGHT"|"ACOUSTIC"|"RF"|"IN_SITU_AIR"|"IN_SITU_WATER"|"ABSOLUTE_POSITIONING"|"RELATIVE_POSITIONING"|"FIELD_SENSING"|"PLATFORM_INTERNAL"|"DERIVED";
export type ServiceId="time"|"seededRng"|"groundTruth"|"environment"|"bathymetry"|"raycast"|"skyOcclusion"|"platformState"|"publishedSensors";
export type OutputPayloadClass="compact"|"camera"|"lidar";

export interface SensorSample<T=unknown>{stepIndex:number;timestampS:number;valid:boolean;status:SensorStatus;payload:T;powerW:number;bytes:number}
export interface SampleContext{stepIndex:number;simTimeS:number;lifecycleState:LifecycleState}
export interface SensorPluginStateDTO{schema_version:number;plugin_id:string;plugin_version:string;state:Record<string,JSONValue>}
export type GrantedServices=Partial<{
  time:()=>number;
  seededRng:()=>number;
  groundTruth:()=>unknown;
  environment:(position:Vec3)=>unknown;
  bathymetry:BathymetryField&{raycast_terrain?:(origin:Vec3,direction:Vec3,maxRangeM:number)=>unknown};
  raycast:(origin:Vec3,direction:Vec3,maxRangeM:number)=>unknown;
  platformState:PlatformStateService;
  skyOcclusion:SkyOcclusionService;
  publishedSensors:PublishedSensorReader;
}>;

export interface SensorPluginMetadata{
  id:string;version:string;domain:SensorDomain;domainParams:Record<string,JSONValue>;
  configSchema:Record<string,unknown>;outputSchema:Record<string,unknown>;
  requiredServices:ServiceId[];nominalRateHz:number;nominalLatencyS:number;
  nominalPowerW:number;nominalBandwidthBps:number;
  outputPayloadClass?:OutputPayloadClass;
  maxPublishedSampleBytes?:number;
  dependencies?:string[];
}
export interface SensorPlugin{
  readonly metadata:SensorPluginMetadata;
  init(config:unknown,services:GrantedServices):void;
  sample(context:SampleContext):SensorSample|null;
  reset():void;
  saveState():SensorPluginStateDTO;
  validateState(state:SensorPluginStateDTO):void;
  loadState(state:SensorPluginStateDTO):void;
  dispose():void;
}

export const MAX_INLINE_PUBLISHED_SAMPLE_BYTES=256*1024;
export const DOMAIN_SERVICE_ALLOWLIST:Readonly<Record<SensorDomain,readonly ServiceId[]>>={
  LIGHT:["time","seededRng","groundTruth","environment","raycast"],
  ACOUSTIC:["time","seededRng","groundTruth","environment","bathymetry","raycast"],
  RF:["time","seededRng","groundTruth","environment","raycast"],
  IN_SITU_AIR:["time","seededRng","environment"],
  IN_SITU_WATER:["time","seededRng","environment","bathymetry"],
  ABSOLUTE_POSITIONING:["time","seededRng","groundTruth","environment","skyOcclusion"],
  RELATIVE_POSITIONING:["time","seededRng","groundTruth","platformState"],
  FIELD_SENSING:["time","seededRng","environment","platformState"],
  PLATFORM_INTERNAL:["time","seededRng","platformState"],
  DERIVED:["time","seededRng","publishedSensors"]
};

/** FNV-1a 32-bit over UTF-8(decimal experiment seed + NUL byte + plugin id). */
export function derivePluginSeed(experimentSeed:number,pluginId:string):number{
  if(!Number.isInteger(experimentSeed)||!pluginId)throw new Error("Plugin seed requires an integer experiment seed and non-empty plugin id.");
  const prefix=new TextEncoder().encode(String(experimentSeed)),suffix=new TextEncoder().encode(pluginId);
  let hash=0x811c9dc5;
  for(const byte of [...prefix,0,...suffix]){hash^=byte;hash=Math.imul(hash,0x01000193)>>>0;}
  return hash>>>0;
}

class RuntimeRandom{
  #state:number;
  constructor(seed:number){this.#state=seed>>>0;}
  next():number{this.#state=(this.#state+0x6D2B79F5)>>>0;let value=this.#state;value=Math.imul(value^value>>>15,value|1);value^=value+Math.imul(value^value>>>7,value|61);return((value^value>>>14)>>>0)/4294967296;}
  save(){return{algorithm:"mulberry32" as const,state_u32:this.#state};}
  validate(state:{algorithm:string;state_u32:number}){if(state?.algorithm!=="mulberry32"||!Number.isInteger(state.state_u32)||state.state_u32<0||state.state_u32>0xffffffff)throw new Error("Invalid plugin RNG checkpoint state.");}
  load(state:{algorithm:string;state_u32:number}){this.validate(state);this.#state=state.state_u32>>>0;}
}

function canonical(value:unknown):string{
  if(value===null||typeof value==="boolean"||typeof value==="string")return JSON.stringify(value);
  if(typeof value==="number"){
    if(Number.isNaN(value))throw new Error("Checkpoint-compatible JSON cannot contain NaN.");
    if(Object.is(value,-0))return '{"__bcod_number":"-0"}';
    if(value===Infinity)return '{"__bcod_number":"Infinity"}';
    if(value===-Infinity)return '{"__bcod_number":"-Infinity"}';
    return JSON.stringify(value);
  }
  if(Array.isArray(value))return`[${value.map(canonical).join(",")}]`;
  if(value&&typeof value==="object")return`{${Object.keys(value as object).sort().map((key)=>`${JSON.stringify(key)}:${canonical((value as Record<string,unknown>)[key])}`).join(",")}}`;
  throw new Error(`Checkpoint-compatible JSON cannot contain ${typeof value}.`);
}
export function deterministicJsonByteLength(value:unknown):number{return new TextEncoder().encode(canonical(value)).byteLength;}

export interface SensorDeclaration{pluginId:string;declarationIndex?:number;config:unknown;enabled?:boolean}
export interface SensorRuntimeSpec{declaration:SensorDeclaration;plugin:SensorPlugin}
export type RuntimeServiceProvider=(service:Exclude<ServiceId,"seededRng"|"publishedSensors">,pluginId:string)=>unknown;
export interface SensorRuntimeCheckpointV1{
  schema_version:1;registry_fingerprint:string;evaluation_order:string[];
  plugins:Array<{plugin_id:string;plugin_version:string;enabled:boolean;lifecycle_state:LifecycleState;schedule:{sample_index:number;next_sample_step:number};rng_state:{algorithm:"mulberry32";state_u32:number};published_sample:SensorSample|null;plugin_state:SensorPluginStateDTO}>;
}
interface Entry{plugin:SensorPlugin;declaration:SensorDeclaration;enabled:boolean;lifecycle:LifecycleState;sampleIndex:number;nextSampleStep:number;rng:RuntimeRandom;published:SensorSample|null;inlinePublished:boolean}

function assertMetadata(metadata:SensorPluginMetadata):void{
  if(!metadata.id||!metadata.version)throw new Error("Sensor metadata requires id and version.");
  if(!metadata.configSchema||!metadata.outputSchema)throw new Error(`Sensor ${metadata.id} requires config and output schemas.`);
  for(const value of [metadata.nominalRateHz,metadata.nominalLatencyS,metadata.nominalPowerW,metadata.nominalBandwidthBps])if(!Number.isFinite(value)||value<0)throw new Error(`Sensor ${metadata.id} has invalid nominal metadata.`);
  const allowed=DOMAIN_SERVICE_ALLOWLIST[metadata.domain];
  if(!allowed)throw new Error(`Sensor ${metadata.id} has unknown domain ${metadata.domain}.`);
  for(const service of metadata.requiredServices)if(!allowed.includes(service))throw new Error(`Sensor ${metadata.id} domain ${metadata.domain} may not request service ${service}.`);
  const payloadClass=metadata.outputPayloadClass??"compact";
  if((payloadClass==="camera"||payloadClass==="lidar")&&metadata.domain!=="LIGHT")throw new Error(`${payloadClass} payloads must declare the LIGHT domain.`);
  if(payloadClass==="compact"){
    const limit=metadata.maxPublishedSampleBytes??MAX_INLINE_PUBLISHED_SAMPLE_BYTES;
    if(!Number.isInteger(limit)||limit<0||limit>MAX_INLINE_PUBLISHED_SAMPLE_BYTES)throw new Error(`Sensor ${metadata.id} published-sample limit exceeds 256 KiB.`);
  }
}
export function validateJsonSchemaValue(value:unknown,schema:Record<string,any>,label="value"):void{const types=Array.isArray(schema.type)?schema.type:schema.type?[schema.type]:[];const actual=value===null?"null":Array.isArray(value)?"array":typeof value;if(types.length&&!types.includes(actual))throw new Error(`${label} must have schema type ${types.join(" or ")}.`);if(actual==="object"){const object=value as Record<string,unknown>;for(const key of schema.required??[])if(!(key in object))throw new Error(`${label} requires ${key}.`);for(const[key,child]of Object.entries(schema.properties??{}))if(key in object)validateJsonSchemaValue(object[key],child as Record<string,any>,`${label}.${key}`);if(schema.additionalProperties===false&&schema.properties)for(const key of Object.keys(object))if(!(key in schema.properties))throw new Error(`${label} contains unsupported property ${key}.`);}if(actual==="number"&&!Number.isFinite(value))throw new Error(`${label} must be finite.`);}

function orderEntries(entries:Entry[]):Entry[]{
  const byId=new Map(entries.map((entry)=>[entry.plugin.metadata.id,entry]));
  const indegree=new Map(entries.map((entry)=>[entry.plugin.metadata.id,0]));
  const outgoing=new Map(entries.map((entry)=>[entry.plugin.metadata.id,[] as string[]]));
  for(const entry of entries)for(const dependency of entry.plugin.metadata.dependencies??[]){if(!byId.has(dependency))throw new Error(`Sensor ${entry.plugin.metadata.id} depends on unknown sensor ${dependency}.`);indegree.set(entry.plugin.metadata.id,indegree.get(entry.plugin.metadata.id)!+1);outgoing.get(dependency)!.push(entry.plugin.metadata.id);}
  const compare=(a:Entry,b:Entry)=>{const ai=a.declaration.declarationIndex,bi=b.declaration.declarationIndex;if(ai!==undefined&&bi!==undefined&&ai!==bi)return ai-bi;if(ai!==undefined&&bi===undefined)return-1;if(ai===undefined&&bi!==undefined)return 1;return a.plugin.metadata.id.localeCompare(b.plugin.metadata.id);};
  const ready=entries.filter((entry)=>indegree.get(entry.plugin.metadata.id)===0).sort(compare),ordered:Entry[]=[];
  while(ready.length){const entry=ready.shift()!;ordered.push(entry);for(const id of outgoing.get(entry.plugin.metadata.id)!){indegree.set(id,indegree.get(id)!-1);if(indegree.get(id)===0){ready.push(byId.get(id)!);ready.sort(compare);}}}
  if(ordered.length!==entries.length)throw new Error("Sensor dependency graph contains a cycle.");
  return ordered;
}

export class SensorRuntimeRegistry{
  readonly evaluationOrder:string[];
  readonly registryFingerprint:string;
  #entries:Entry[];#byId:Map<string,Entry>;#services:RuntimeServiceProvider;#stepIndex=0;#simTimeS=0;
  constructor(specs:SensorRuntimeSpec[],experimentSeed:number,services:RuntimeServiceProvider=()=>undefined){
    const ids=new Set<string>();
    for(const {declaration,plugin} of specs){assertMetadata(plugin.metadata);validateJsonSchemaValue(declaration.config,plugin.metadata.configSchema,`${plugin.metadata.id} config`);if(declaration.pluginId!==plugin.metadata.id)throw new Error(`Declaration ${declaration.pluginId} does not match plugin ${plugin.metadata.id}.`);if(ids.has(plugin.metadata.id))throw new Error(`Duplicate sensor plugin id ${plugin.metadata.id}.`);ids.add(plugin.metadata.id);}
    this.#services=services;
    this.#entries=orderEntries(specs.map(({declaration,plugin})=>({plugin,declaration:{...declaration},enabled:declaration.enabled??true,lifecycle:"OFF",sampleIndex:0,nextSampleStep:0,rng:new RuntimeRandom(derivePluginSeed(experimentSeed,plugin.metadata.id)),published:null,inlinePublished:(plugin.metadata.outputPayloadClass??"compact")==="compact"})));
    this.#byId=new Map(this.#entries.map((entry)=>[entry.plugin.metadata.id,entry]));this.evaluationOrder=this.#entries.map((entry)=>entry.plugin.metadata.id);
    this.registryFingerprint=canonical(this.#entries.map((entry)=>({declaration_index:entry.declaration.declarationIndex??null,id:entry.plugin.metadata.id,version:entry.plugin.metadata.version,domain:entry.plugin.metadata.domain,domain_params:entry.plugin.metadata.domainParams,required_services:[...entry.plugin.metadata.requiredServices].sort(),dependencies:[...(entry.plugin.metadata.dependencies??[])].sort(),config:entry.declaration.config}))); 
    for(const entry of this.#entries)entry.plugin.init(structuredClone(entry.declaration.config),this.#grant(entry));
  }
  #grant(entry:Entry):GrantedServices{
    const granted:GrantedServices={};
    for(const service of entry.plugin.metadata.requiredServices){
      if(service==="seededRng")granted.seededRng=()=>entry.rng.next();
      else if(service==="publishedSensors")granted.publishedSensors=new PublishedSensorsService((id)=>this.#byId.get(id)?.published??null);
      else if(service==="time")granted.time=()=>this.#simTimeS;
      else (granted as Record<string,unknown>)[service]=this.#services(service,entry.plugin.metadata.id);
    }
    return Object.freeze(granted);
  }
  setLifecycle(pluginId:string,state:LifecycleState):void{const entry=this.#byId.get(pluginId);if(!entry)throw new Error(`Unknown sensor ${pluginId}.`);entry.lifecycle=state;}
  sampleStep(stepIndex:number,simTimeS:number):Record<string,SensorSample>{
    if(!Number.isInteger(stepIndex)||!Number.isFinite(simTimeS))throw new Error("Sensor sampling requires integer step and finite time.");this.#stepIndex=stepIndex;this.#simTimeS=simTimeS;const result:Record<string,SensorSample>={};
    for(const entry of this.#entries){if(!entry.enabled||stepIndex<entry.nextSampleStep)continue;const sample=entry.plugin.sample({stepIndex,simTimeS,lifecycleState:entry.lifecycle});entry.sampleIndex++;entry.nextSampleStep=stepIndex+1;if(sample){validateJsonSchemaValue(sample.payload,entry.plugin.metadata.outputSchema,`${entry.plugin.metadata.id} payload`);if(entry.inlinePublished){const size=deterministicJsonByteLength(sample),limit=entry.plugin.metadata.maxPublishedSampleBytes??MAX_INLINE_PUBLISHED_SAMPLE_BYTES;if(size>limit)throw new Error(`Sensor ${entry.plugin.metadata.id} published sample is ${size} bytes, exceeding ${limit}.`);entry.published=structuredClone(sample);}result[entry.plugin.metadata.id]=structuredClone(sample);}}
    return result;
  }
  saveState():SensorRuntimeCheckpointV1{return{schema_version:1,registry_fingerprint:this.registryFingerprint,evaluation_order:[...this.evaluationOrder],plugins:this.#entries.map((entry)=>({plugin_id:entry.plugin.metadata.id,plugin_version:entry.plugin.metadata.version,enabled:entry.enabled,lifecycle_state:entry.lifecycle,schedule:{sample_index:entry.sampleIndex,next_sample_step:entry.nextSampleStep},rng_state:entry.rng.save(),published_sample:entry.inlinePublished?structuredClone(entry.published):null,plugin_state:structuredClone(entry.plugin.saveState())}))};}
  validateState(state:SensorRuntimeCheckpointV1):void{
    if(state?.schema_version!==1||state.registry_fingerprint!==this.registryFingerprint)throw new Error("Sensor registry checkpoint fingerprint mismatch.");if(JSON.stringify(state.evaluation_order)!==JSON.stringify(this.evaluationOrder))throw new Error("Sensor checkpoint evaluation order mismatch.");if(state.plugins.length!==this.#entries.length)throw new Error("Sensor checkpoint plugin set mismatch.");
    state.plugins.forEach((saved,index)=>{const entry=this.#entries[index];if(saved.plugin_id!==entry.plugin.metadata.id||saved.plugin_version!==entry.plugin.metadata.version)throw new Error(`Sensor checkpoint plugin mismatch at index ${index}.`);if(!Number.isInteger(saved.schedule?.sample_index)||!Number.isInteger(saved.schedule?.next_sample_step))throw new Error(`Sensor ${saved.plugin_id} has invalid schedule state.`);entry.rng.validate(saved.rng_state);if(!entry.inlinePublished&&saved.published_sample!==null)throw new Error(`Large-payload sensor ${saved.plugin_id} cannot checkpoint published samples.`);if(saved.published_sample&&deterministicJsonByteLength(saved.published_sample)>(entry.plugin.metadata.maxPublishedSampleBytes??MAX_INLINE_PUBLISHED_SAMPLE_BYTES))throw new Error(`Sensor ${saved.plugin_id} checkpoint sample exceeds its inline limit.`);entry.plugin.validateState(saved.plugin_state);});
  }
  loadState(state:SensorRuntimeCheckpointV1):void{
    this.validateState(state);const rollback=this.saveState();
    try{state.plugins.forEach((saved,index)=>{const entry=this.#entries[index];entry.enabled=saved.enabled;entry.lifecycle=saved.lifecycle_state;entry.sampleIndex=saved.schedule.sample_index;entry.nextSampleStep=saved.schedule.next_sample_step;entry.rng.load(saved.rng_state);entry.published=structuredClone(saved.published_sample);entry.plugin.loadState(structuredClone(saved.plugin_state));});}
    catch(error){rollback.plugins.forEach((saved,index)=>{const entry=this.#entries[index];entry.enabled=saved.enabled;entry.lifecycle=saved.lifecycle_state;entry.sampleIndex=saved.schedule.sample_index;entry.nextSampleStep=saved.schedule.next_sample_step;entry.rng.load(saved.rng_state);entry.published=structuredClone(saved.published_sample);entry.plugin.loadState(structuredClone(saved.plugin_state));});throw error;}
  }
  dispose():void{for(const entry of this.#entries)entry.plugin.dispose();this.#entries=[];this.#byId.clear();}
}
