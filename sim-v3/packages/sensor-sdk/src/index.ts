import {AcousticDomain} from "./domains/acoustic.ts";
import type {GrantedServices,SampleContext,SensorPlugin as TypedSensorPlugin,SensorPluginMetadata,SensorPluginStateDTO,SensorSample as TypedSensorSample} from "./runtime.ts";
import {GpsPlugin} from "./plugins/gps.ts";
import {ImuPlugin} from "./plugins/imu.ts";
import {AnemometerPlugin,BarometerPlugin,CameraPlugin,InsGpsFusionPlugin,LidarPlugin,MagnetometerPlugin,PlatformTelemetryPlugin,RadarPlugin,WaterQualityPlugin} from "./plugins/remaining.ts";

export * from "./runtime.ts";
export * from "./services/index.ts";
export * from "./domains/index.ts";
export * from "./plugins/index.ts";

/** @deprecated Legacy compatibility interface; new plugins implement the metadata-based interface from runtime.ts. */
export type SensorState="OFF"|"STARTING"|"WARMING"|"ACTIVE"|"DEGRADED"|"FAILED"|"COOLDOWN";
export interface SensorServices {time_s():number;seededRandom():number;groundTruth():unknown;environment(position:[number,number,number]):unknown;raycast(origin:[number,number,number],direction:[number,number,number],maxRange:number):{distance_m:number}|null}
export interface SensorSample<T=unknown>{sensor_id:string;timestamp_s:number;valid:boolean;value:T|null;status:SensorState;power_w:number;bytes:number}
export interface LegacySensorPlugin<TConfig=unknown,TOutput=unknown>{readonly id:string;readonly version:string;readonly outputSchema:Record<string,unknown>;initialize(config:TConfig,services:SensorServices):void;setState(state:SensorState):void;sample():SensorSample<TOutput>;reset():void;dispose():void}

export class SensorLifecycle {
  state:SensorState="OFF";
  readonly startup_s:number;readonly warmup_s:number;readonly cooldown_s:number;readonly standby_w:number;readonly active_w:number;
  constructor(startup_s:number,warmup_s:number,cooldown_s:number,standby_w:number,active_w:number){this.startup_s=startup_s;this.warmup_s=warmup_s;this.cooldown_s=cooldown_s;this.standby_w=standby_w;this.active_w=active_w;}
  transition(target:SensorState):void{
    const allowed:Record<SensorState,SensorState[]>={OFF:["STARTING","FAILED"],STARTING:["WARMING","FAILED","OFF"],WARMING:["ACTIVE","FAILED","COOLDOWN"],ACTIVE:["DEGRADED","FAILED","COOLDOWN"],DEGRADED:["ACTIVE","FAILED","COOLDOWN"],FAILED:["OFF"],COOLDOWN:["OFF","FAILED"]};
    if(!allowed[this.state].includes(target))throw new Error(`Invalid sensor transition ${this.state} -> ${target}.`);
    this.state=target;
  }
  power():number{return this.state==="ACTIVE"||this.state==="DEGRADED"?this.active_w:this.state==="OFF"?0:this.standby_w;}
}

export interface DepthSounderConfig{mount_m:[number,number,number];direction_body:[number,number,number];min_range_m:number;max_range_m:number;frequency_hz:number;noise_std_m?:number;source_level_db?:number;target_strength_db?:number;target_size_m2?:number;detection_threshold_db?:number;power_w?:number}
export interface DepthSounderOutput{depth_m:number;range_m:number;sound_speed_mps:number;transmission_loss_db:number;snr_db:number}
export class DepthSounderPlugin implements TypedSensorPlugin{
  readonly metadata:SensorPluginMetadata;
  #config?:DepthSounderConfig;#services?:GrantedServices;#sampleCount=0;#gaussianSpare:number|null=null;
  constructor(frequencyHz=200000,id="single-beam-depth-sounder"){if(!Number.isFinite(frequencyHz)||frequencyHz<=0)throw new Error("Depth sounder frequency must be positive and finite.");this.metadata={id,version:"3.0.0",domain:"ACOUSTIC",domainParams:{frequencyHz},configSchema:{type:"object",required:["mount_m","direction_body","min_range_m","max_range_m","frequency_hz"]},outputSchema:{type:["object","null"],properties:{depth_m:{type:"number"},range_m:{type:"number"},sound_speed_mps:{type:"number"},transmission_loss_db:{type:"number"},snr_db:{type:"number"}}},requiredServices:["seededRng","environment","bathymetry"],nominalRateHz:10,nominalLatencyS:.02,nominalPowerW:8,nominalBandwidthBps:400};}
  init(config:unknown,services:GrantedServices):void{const value=structuredClone(config) as DepthSounderConfig,frequency=this.metadata.domainParams.frequencyHz as number;if(!value||value.min_range_m<0||value.max_range_m<=value.min_range_m||value.frequency_hz!==frequency||!Array.isArray(value.mount_m)||!Array.isArray(value.direction_body))throw new Error("Invalid depth sounder config or frequency does not match domain metadata.");if(!services.seededRng||!services.environment||!services.bathymetry)throw new Error("Depth sounder requires RNG, environment, and bathymetry services.");this.#config=value;this.#services=services;this.reset();}
  sample(context:SampleContext):TypedSensorSample<DepthSounderOutput|null>|null{
    if(!this.#config||!this.#services)throw new Error("Depth sounder is not initialized.");if(context.lifecycleState!=="ACTIVE"&&context.lifecycleState!=="DEGRADED")return null;this.#sampleCount++;const config=this.#config,base={stepIndex:context.stepIndex,timestampS:context.simTimeS,status:context.lifecycleState,powerW:config.power_w??this.metadata.nominalPowerW,bytes:40},field=this.#services.bathymetry!,terrainHit=field.raycast_terrain?.(config.mount_m,config.direction_body,config.max_range_m)as{distance_m:number}|null|undefined,depth=field.sample({position_ned_m:config.mount_m,time_s:context.simTimeS}).water_depth_m,verticalRange=depth===null?null:(depth-config.mount_m[2])/Math.max(config.direction_body[2],1e-12),hit=terrainHit??(verticalRange===null?null:{distance_m:verticalRange});if(!hit||!Number.isFinite(hit.distance_m)||hit.distance_m<config.min_range_m||hit.distance_m>config.max_range_m)return{...base,valid:false,payload:null};const params={frequencyHz:this.metadata.domainParams.frequencyHz as number},environment=AcousticDomain.queryEnvironment(this.#services,config.mount_m),loss=AcousticDomain.primitives.transmissionLossDb(hit.distance_m,params,environment),noiseFloor=AcousticDomain.primitives.ambientNoiseDb(params,environment),received=(config.source_level_db??170)-2*loss+(config.target_strength_db??-10),snr=received-noiseFloor-(config.detection_threshold_db??10),probability=AcousticDomain.primitives.detectionProbability(snr,config.target_size_m2??1);if(this.#services.seededRng!()>=probability)return{...base,valid:false,payload:null};const std=config.noise_std_m??AcousticDomain.defaultNoiseProfile(params).measurementStd,range=Math.max(hit.distance_m+this.#normal()*std,0),soundSpeed=AcousticDomain.primitives.soundSpeedMps(params,environment);return{...base,valid:true,payload:{range_m:range,depth_m:config.mount_m[2]+range*config.direction_body[2],sound_speed_mps:soundSpeed,transmission_loss_db:loss,snr_db:snr}};
  }
  #normal():number{if(this.#gaussianSpare!==null){const value=this.#gaussianSpare;this.#gaussianSpare=null;return value;}const u1=Math.max(this.#services!.seededRng!(),Number.EPSILON),u2=this.#services!.seededRng!(),radius=Math.sqrt(-2*Math.log(u1)),angle=2*Math.PI*u2;this.#gaussianSpare=radius*Math.sin(angle);return radius*Math.cos(angle);}
  reset():void{this.#sampleCount=0;this.#gaussianSpare=null;}
  saveState():SensorPluginStateDTO{return{schema_version:1,plugin_id:this.metadata.id,plugin_version:this.metadata.version,state:{sample_count:this.#sampleCount,gaussian_spare:this.#gaussianSpare}};}
  validateState(state:SensorPluginStateDTO):void{const count=state?.state?.sample_count,spare=state?.state?.gaussian_spare;if(state?.schema_version!==1||state.plugin_id!==this.metadata.id||state.plugin_version!==this.metadata.version||!Number.isInteger(count)||typeof count!=="number"||count<0||(spare!==null&&!Number.isFinite(spare)))throw new Error("Invalid depth sounder checkpoint state.");}
  loadState(state:SensorPluginStateDTO):void{this.validateState(state);this.#sampleCount=state.state.sample_count as number;this.#gaussianSpare=state.state.gaussian_spare as number|null;}
  dispose():void{this.#config=undefined;this.#services=undefined;this.reset();}
}
export class SurveySonar38KhzPlugin extends DepthSounderPlugin{constructor(){super(38000,"survey-sonar-38khz");}}

export interface BuiltInSensorRegistration {
  create:()=>TypedSensorPlugin;
  defaultConfig:Record<string,unknown>;
  legacyAliases:readonly string[];
}

/** Authoritative built-in typed-plugin catalog used by production and capability UIs. */
export const BUILT_IN_SENSOR_REGISTRY:Readonly<Record<string,BuiltInSensorRegistration>>={
  gps:{create:()=>new GpsPlugin(),defaultConfig:{},legacyAliases:["gps"]},
  imu:{create:()=>new ImuPlugin(),defaultConfig:{},legacyAliases:["imu"]},
  camera:{create:()=>new CameraPlugin(),defaultConfig:{},legacyAliases:["dayCam","nightCam"]},
  lidar:{create:()=>new LidarPlugin(),defaultConfig:{},legacyAliases:["lidar"]},
  radar:{create:()=>new RadarPlugin(),defaultConfig:{},legacyAliases:[]},
  "single-beam-depth-sounder":{create:()=>new DepthSounderPlugin(),defaultConfig:{mount_m:[0,0,0],direction_body:[0,0,1],min_range_m:.2,max_range_m:100,frequency_hz:200000},legacyAliases:[]},
  "survey-sonar-38khz":{create:()=>new SurveySonar38KhzPlugin(),defaultConfig:{mount_m:[0,0,0],direction_body:[0,0,1],min_range_m:1,max_range_m:1000,frequency_hz:38000},legacyAliases:[]},
  anemometer:{create:()=>new AnemometerPlugin(),defaultConfig:{},legacyAliases:[]},
  barometer:{create:()=>new BarometerPlugin(),defaultConfig:{},legacyAliases:[]},
  "water-quality":{create:()=>new WaterQualityPlugin(),defaultConfig:{},legacyAliases:["exo2"]},
  "platform-telemetry":{create:()=>new PlatformTelemetryPlugin(),defaultConfig:{},legacyAliases:[]},
  magnetometer:{create:()=>new MagnetometerPlugin(),defaultConfig:{},legacyAliases:[]},
  "ins-gps-fusion":{create:()=>new InsGpsFusionPlugin(),defaultConfig:{},legacyAliases:[]}
};

export const BUILT_IN_SENSOR_CAPABILITIES=Object.freeze(Object.entries(BUILT_IN_SENSOR_REGISTRY).map(([id,registration])=>{
  const metadata=registration.create().metadata;
  return Object.freeze({id,version:metadata.version,domain:metadata.domain,outputSchema:structuredClone(metadata.outputSchema),outputPayloadClass:metadata.outputPayloadClass??"compact",legacyAliases:[...registration.legacyAliases]});
}));

export const STANDARD_SENSOR_IDS=Object.freeze(BUILT_IN_SENSOR_CAPABILITIES.map(({id})=>id));
