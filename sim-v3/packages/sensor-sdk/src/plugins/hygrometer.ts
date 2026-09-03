import {InSituAirDomain} from "../domains/direct.ts";
import type {GrantedServices,SampleContext,SensorPlugin,SensorPluginMetadata,SensorPluginStateDTO,SensorSample} from "../runtime.ts";
import {GaussianNoise} from "./noise.ts";

export interface HygrometerConfig {noise_std_fraction?:number}
export interface HygrometerOutput {relative_humidity_fraction:number}

export class HygrometerPlugin implements SensorPlugin {
  readonly metadata:SensorPluginMetadata={id:"hygrometer",version:"1.0.0",domain:"IN_SITU_AIR",domainParams:{quantity:"relative-humidity"},configSchema:{type:"object",properties:{noise_std_fraction:{type:"number",minimum:0}}},outputSchema:{type:"object",properties:{relative_humidity_fraction:{type:"number",minimum:0,maximum:1}}},requiredServices:["seededRng","environment"],nominalRateHz:2,nominalLatencyS:0,nominalPowerW:.05,nominalBandwidthBps:64,outputPayloadClass:"compact"};
  #services?:GrantedServices;#noiseStd=.005;#noise=new GaussianNoise();#sampleCount=0;
  init(config:unknown,services:GrantedServices):void{const value=(config??{}) as HygrometerConfig;if(!services.seededRng||!services.environment||value.noise_std_fraction!==undefined&&(!Number.isFinite(value.noise_std_fraction)||value.noise_std_fraction<0))throw new Error("Invalid hygrometer configuration or services.");this.#services=services;this.#noiseStd=value.noise_std_fraction??.005;this.reset();}
  sample(context:SampleContext):SensorSample<HygrometerOutput>|null{if(!this.#services)throw new Error("Hygrometer is not initialized.");if(context.lifecycleState!=="ACTIVE"&&context.lifecycleState!=="DEGRADED")return null;const environment=InSituAirDomain.queryEnvironment(this.#services,[0,0,0]),relativeHumidity=Math.max(0,Math.min(1,environment.humidityFraction+this.#noise.next(this.#services.seededRng!)*this.#noiseStd));this.#sampleCount++;return{stepIndex:context.stepIndex,timestampS:context.simTimeS,valid:true,status:context.lifecycleState,payload:{relative_humidity_fraction:relativeHumidity},powerW:this.metadata.nominalPowerW,bytes:8};}
  reset():void{this.#sampleCount=0;this.#noise.spare=null;}
  saveState():SensorPluginStateDTO{return{schema_version:1,plugin_id:this.metadata.id,plugin_version:this.metadata.version,state:{sample_count:this.#sampleCount,gaussian_spare:this.#noise.spare}};}
  validateState(state:SensorPluginStateDTO):void{if(state?.schema_version!==1||state.plugin_id!==this.metadata.id||state.plugin_version!==this.metadata.version||!Number.isInteger(state.state?.sample_count)||(state.state.gaussian_spare!==null&&!Number.isFinite(state.state.gaussian_spare)))throw new Error("Invalid hygrometer state.");}
  loadState(state:SensorPluginStateDTO):void{this.validateState(state);this.#sampleCount=state.state.sample_count as number;this.#noise.spare=state.state.gaussian_spare as number|null;}
  dispose():void{this.#services=undefined;this.reset();}
}
