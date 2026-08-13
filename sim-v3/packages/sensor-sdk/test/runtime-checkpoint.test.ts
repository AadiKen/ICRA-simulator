import assert from "node:assert/strict";
import {derivePluginSeed,deterministicJsonByteLength,MAX_INLINE_PUBLISHED_SAMPLE_BYTES,SensorRuntimeRegistry,type GrantedServices,type LifecycleState,type SensorDomain,type SensorPlugin,type SensorPluginMetadata,type SensorPluginStateDTO,type SensorSample,type ServiceId} from "../src/runtime.ts";

class FixturePlugin implements SensorPlugin{
  readonly metadata:SensorPluginMetadata;
  services:GrantedServices={};counter=0;queue:Array<{delivery:number;value:number}>=[];failLoad=false;
  constructor(id:string,options:{domain?:SensorDomain;services?:ServiceId[];dependencies?:string[];payloadClass?:"compact"|"camera"|"lidar";limit?:number}={}){this.metadata={id,version:"1.0.0",domain:options.domain??"IN_SITU_AIR",domainParams:{},configSchema:{type:"object"},outputSchema:{type:"object"},requiredServices:options.services??["seededRng"],nominalRateHz:1,nominalLatencyS:0,nominalPowerW:1,nominalBandwidthBps:32,dependencies:options.dependencies,outputPayloadClass:options.payloadClass,maxPublishedSampleBytes:options.limit};}
  init(_config:unknown,services:GrantedServices){this.services=services;}
  sample(context:{stepIndex:number;simTimeS:number;lifecycleState:LifecycleState}):SensorSample{const random=this.services.seededRng?.()??0;this.counter++;this.queue.push({delivery:context.stepIndex+2,value:random});return{stepIndex:context.stepIndex,timestampS:context.simTimeS,valid:true,status:context.lifecycleState,payload:{counter:this.counter,random,input:this.services.publishedSensors?.latest("source")?.payload??null},powerW:1,bytes:32};}
  reset(){this.counter=0;this.queue=[];}
  saveState():SensorPluginStateDTO{return{schema_version:1,plugin_id:this.metadata.id,plugin_version:this.metadata.version,state:{counter:this.counter,queue:structuredClone(this.queue)}};}
  validateState(value:SensorPluginStateDTO){if(value?.schema_version!==1||value.plugin_id!==this.metadata.id||value.plugin_version!==this.metadata.version||!Number.isInteger(value.state?.counter)||!Array.isArray(value.state?.queue))throw new Error(`Invalid ${this.metadata.id} state.`);}
  loadState(value:SensorPluginStateDTO){this.validateState(value);if(this.failLoad)throw new Error("deliberate load failure");this.counter=value.state.counter as number;this.queue=structuredClone(value.state.queue) as Array<{delivery:number;value:number}>;}
  dispose(){}
}
const spec=(plugin:FixturePlugin,index?:number)=>({plugin,declaration:{pluginId:plugin.metadata.id,declarationIndex:index,config:{},enabled:true}});

// 1. Per-plugin RNG streams are byte-reproducible and independent of registry construction order.
assert.equal(derivePluginSeed(7,"gps"),derivePluginSeed(7,"gps"));
assert.notEqual(derivePluginSeed(7,"gps"),derivePluginSeed(7,"imu"));
const orderA=new SensorRuntimeRegistry([spec(new FixturePlugin("alpha"),0),spec(new FixturePlugin("beta"),1)],17);
const orderB=new SensorRuntimeRegistry([spec(new FixturePlugin("beta"),1),spec(new FixturePlugin("alpha"),0)],17);
assert.deepEqual(orderA.sampleStep(0,0),orderB.sampleStep(0,0));

// 2. Plugin and registry DTOs survive a real JSON round trip.
const dto=JSON.parse(JSON.stringify(orderA.saveState()));
orderA.validateState(dto);
assert.deepEqual(dto,orderA.saveState());

// 3. Lifecycle and generic cadence state restore exactly.
orderA.setLifecycle("alpha","ACTIVE");orderA.sampleStep(1,.1);const cadence=JSON.parse(JSON.stringify(orderA.saveState()));orderA.setLifecycle("alpha","FAILED");orderA.sampleStep(2,.2);orderA.loadState(cadence);assert.equal(orderA.saveState().plugins[0].lifecycle_state,"ACTIVE");assert.deepEqual(orderA.saveState().plugins[0].schedule,cadence.plugins[0].schedule);

// 4. Plugin-owned latency queues round-trip through plugin_state.
const alpha=orderA.saveState().plugins.find((value)=>value.plugin_id==="alpha")!;assert.ok((alpha.plugin_state.state.queue as unknown[]).length>0);orderA.sampleStep(2,.2);orderA.loadState(cadence);assert.deepEqual(orderA.saveState().plugins.find((value)=>value.plugin_id==="alpha")!.plugin_state,alpha.plugin_state);

// 5. Compact most-recent publications restore; camera/LiDAR publications are categorically excluded.
const published=orderA.saveState();const remembered=structuredClone(published.plugins[0].published_sample);orderA.sampleStep(3,.3);orderA.loadState(published);assert.deepEqual(orderA.saveState().plugins[0].published_sample,remembered);
const camera=new SensorRuntimeRegistry([spec(new FixturePlugin("camera",{domain:"LIGHT",services:["seededRng"],payloadClass:"camera"}),0)],1);camera.sampleStep(0,0);assert.equal(camera.saveState().plugins[0].published_sample,null);
const lidar=new SensorRuntimeRegistry([spec(new FixturePlugin("lidar",{domain:"LIGHT",services:["seededRng"],payloadClass:"lidar"}),0)],1);lidar.sampleStep(0,0);assert.equal(lidar.saveState().plugins[0].published_sample,null);
assert.throws(()=>new SensorRuntimeRegistry([spec(new FixturePlugin("oversize",{limit:MAX_INLINE_PUBLISHED_SAMPLE_BYTES+1}),0)],1),/256 KiB/);
assert.ok(deterministicJsonByteLength({text:"é"})>JSON.stringify({text:"é"}).length,"Size accounting must use UTF-8 bytes.");

// 6. Derived dependencies override declaration order and cycles are rejected.
const derived=new FixturePlugin("fusion",{domain:"DERIVED",services:["publishedSensors"],dependencies:["source"]}),source=new FixturePlugin("source");
const graph=new SensorRuntimeRegistry([spec(derived,0),spec(source,1)],2);assert.deepEqual(graph.evaluationOrder,["source","fusion"]);const samples=graph.sampleStep(0,0);assert.deepEqual((samples.fusion.payload as any).input,samples.source.payload);
const cycleA=new FixturePlugin("cycle-a",{domain:"DERIVED",services:["publishedSensors"],dependencies:["cycle-b"]}),cycleB=new FixturePlugin("cycle-b",{domain:"DERIVED",services:["publishedSensors"],dependencies:["cycle-a"]});assert.throws(()=>new SensorRuntimeRegistry([spec(cycleA,0),spec(cycleB,1)],1),/cycle/);

// 7. Run/checkpoint/run/restore/run is bit-identical, including RNG and plugin state.
const replayPlugin=new FixturePlugin("replay"),replay=new SensorRuntimeRegistry([spec(replayPlugin,0)],99);for(let i=0;i<4;i++)replay.sampleStep(i,i*.1);const checkpoint=JSON.parse(JSON.stringify(replay.saveState()));const first=[];for(let i=4;i<10;i++)first.push(replay.sampleStep(i,i*.1));replay.loadState(checkpoint);const second=[];for(let i=4;i<10;i++)second.push(replay.sampleStep(i,i*.1));assert.deepEqual(second,first);

// 8. Missing/additional/version/configuration mismatches are rejected.
const missing=structuredClone(checkpoint);missing.plugins=[];assert.throws(()=>replay.validateState(missing),/plugin set/);const wrongVersion=structuredClone(checkpoint);wrongVersion.plugins[0].plugin_version="2.0.0";assert.throws(()=>replay.validateState(wrongVersion),/plugin mismatch/);const differentlyConfigured=new SensorRuntimeRegistry([{plugin:new FixturePlugin("replay"),declaration:{pluginId:"replay",declarationIndex:0,config:{different:true}}}],99);assert.throws(()=>differentlyConfigured.validateState(checkpoint),/fingerprint/);assert.throws(()=>new SensorRuntimeRegistry([spec(new FixturePlugin("same"),0),spec(new FixturePlugin("same"),1)],1),/Duplicate/);

// 9. Validation and load failures leave the running registry unchanged.
const before=structuredClone(replay.saveState()),invalid=structuredClone(before);invalid.plugins[0].rng_state.state_u32=-1;assert.throws(()=>replay.loadState(invalid),/RNG/);assert.deepEqual(replay.saveState(),before);
replayPlugin.failLoad=true;assert.throws(()=>replay.loadState(before),/deliberate/);replayPlugin.failLoad=false;assert.deepEqual(replay.saveState(),before);

// Service-scope lint fixtures: Derived ground truth and ordinary cross-domain overreach fail construction.
assert.throws(()=>new SensorRuntimeRegistry([spec(new FixturePlugin("cheat",{domain:"DERIVED",services:["groundTruth"]}),0)],1),/may not request/);
assert.throws(()=>new SensorRuntimeRegistry([spec(new FixturePlugin("air-ray",{domain:"IN_SITU_AIR",services:["raycast"]}),0)],1),/may not request/);
assert.throws(()=>new SensorRuntimeRegistry([spec(new FixturePlugin("air-sky",{domain:"IN_SITU_AIR",services:["skyOcclusion"]}),0)],1),/may not request/);
assert.throws(()=>new SensorRuntimeRegistry([spec(new FixturePlugin("gps-general-ray",{domain:"ABSOLUTE_POSITIONING",services:["raycast"]}),0)],1),/may not request/);

orderA.dispose();orderB.dispose();camera.dispose();lidar.dispose();graph.dispose();replay.dispose();differentlyConfigured.dispose();
console.log("Sensor runtime checkpoint determinism tests passed.");
