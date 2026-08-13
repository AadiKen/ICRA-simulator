import {toGroupedEnvironmentSample,type EnvironmentSample,type SoundSpeedPoint} from "@bcod/environment";
import type {GrantedServices,Vec3} from "../runtime.ts";
import type {DomainModule,NoiseProfile,UnitConventions} from "./types.ts";

export interface AcousticParams{frequencyHz:number}
export interface AcousticEnvironment{
  water:{temperatureC:number;salinityPsu:number;pressurePa:number;soundSpeedProfile?:SoundSpeedPoint[]};
  surface:{seaStateBeaufort:number;significantWaveHeightM?:number};
  sensorDepthM:number;
  waterDepthM:number|null;
}
export interface AcousticPrimitives{
  soundSpeedMps(params:AcousticParams,environment:AcousticEnvironment,depthM?:number):number;
  absorptionDbPerKm(params:AcousticParams):number;
  transmissionLossDb(rangeM:number,params:AcousticParams,environment:AcousticEnvironment):number;
  ambientNoiseDb(params:AcousticParams,environment:AcousticEnvironment):number;
  detectionProbability(snrDb:number,targetSizeM2?:number):number;
}
const clamp=(value:number,min:number,max:number)=>Math.min(Math.max(value,min),max);
function validateParams(params:AcousticParams):void{if(!Number.isFinite(params.frequencyHz)||params.frequencyHz<=0)throw new Error("Acoustic frequency must be positive and finite.");}
function interpolateProfile(profile:SoundSpeedPoint[],depthM:number):number{
  if(!profile.length)throw new Error("Sound-speed profile cannot be empty.");
  const points=[...profile].sort((a,b)=>a.depth_m-b.depth_m);for(const point of points)if(!Number.isFinite(point.depth_m)||point.depth_m<0||!Number.isFinite(point.sound_speed_mps)||point.sound_speed_mps<=0)throw new Error("Invalid sound-speed profile point.");
  if(depthM<=points[0].depth_m)return points[0].sound_speed_mps;if(depthM>=points.at(-1)!.depth_m)return points.at(-1)!.sound_speed_mps;
  const upper=points.findIndex((point)=>point.depth_m>=depthM),a=points[upper-1],b=points[upper],weight=(depthM-a.depth_m)/(b.depth_m-a.depth_m);return a.sound_speed_mps+(b.sound_speed_mps-a.sound_speed_mps)*weight;
}
export const acousticPrimitives:AcousticPrimitives={
  soundSpeedMps(params,environment,depthM=environment.sensorDepthM){validateParams(params);if(environment.water.soundSpeedProfile?.length)return interpolateProfile(environment.water.soundSpeedProfile,Math.max(depthM,0));const temperature=environment.water.temperatureC,salinity=environment.water.salinityPsu,pressure=environment.water.pressurePa;if(![temperature,salinity,pressure].every(Number.isFinite)||salinity<0||pressure<0)throw new Error("Invalid acoustic water environment.");const depth=Math.max((pressure-101325)/(1025*9.80665),0);return 1448.96+4.591*temperature-5.304e-2*temperature**2+2.374e-4*temperature**3+1.34*(salinity-35)+1.63e-2*depth+1.675e-7*depth**2-1.025e-2*temperature*(salinity-35)-7.139e-13*temperature*depth**3;},
  absorptionDbPerKm(params){validateParams(params);const f=params.frequencyHz/1000,f2=f*f;return .11*f2/(1+f2)+44*f2/(4100+f2)+2.75e-4*f2+.003;},
  transmissionLossDb(rangeM,params,environment){validateParams(params);if(!Number.isFinite(rangeM)||rangeM<0)throw new Error("Acoustic range must be non-negative and finite.");if(rangeM===0)return 0;acousticPrimitives.soundSpeedMps(params,environment);return 20*Math.log10(Math.max(rangeM,1e-9))+acousticPrimitives.absorptionDbPerKm(params)*rangeM/1000;},
  ambientNoiseDb(params,environment){validateParams(params);const sea=clamp(environment.surface.seaStateBeaufort,0,12),wave=Math.max(environment.surface.significantWaveHeightM??0,0),frequencyKhz=params.frequencyHz/1000;if(!Number.isFinite(sea)||!Number.isFinite(wave))throw new Error("Sea state and wave height must be finite.");const surfaceDb=45+7.5*Math.sqrt(sea)+2.5*Math.sqrt(wave)-17*Math.log10(Math.max(frequencyKhz,.001)),thermalDb=-15+20*Math.log10(Math.max(frequencyKhz,.001));return 10*Math.log10(10**(surfaceDb/10)+10**(thermalDb/10));},
  detectionProbability(snrDb,targetSizeM2=1){if(!Number.isFinite(snrDb)||!Number.isFinite(targetSizeM2)||targetSizeM2<=0)throw new Error("Detection inputs must be finite with positive target size.");const adjusted=snrDb+10*Math.log10(targetSizeM2),value=1/(1+Math.exp(-adjusted/3));return clamp(value,0,1);}
};
export const acousticUnits:UnitConventions={distance:"m",frequency:"Hz",pressure:"Pa absolute",level:"dB re 1 uPa",temperature:"degC",salinity:"PSU"};
export const AcousticDomain:DomainModule<AcousticParams,AcousticEnvironment,AcousticPrimitives>={
  domain:"ACOUSTIC",version:"1.0.0",units:acousticUnits,primitives:acousticPrimitives,
  queryEnvironment(services:GrantedServices,at:Vec3):AcousticEnvironment{if(!services.environment)throw new Error("Acoustic domain requires the environment service.");const sample=toGroupedEnvironmentSample(services.environment(at) as EnvironmentSample),water=sample.water,surface=sample.surface,depth=services.bathymetry?.sample({position_ned_m:at,time_s:services.time?.()??0}).water_depth_m??null;return{water:{temperatureC:water.temperature_c??10,salinityPsu:water.salinity_psu??35,pressurePa:water.pressure_pa??101325,...water.sound_speed_profile?{soundSpeedProfile:structuredClone(water.sound_speed_profile)}:{}},surface:{seaStateBeaufort:surface.sea_state_beaufort??0,...surface.significant_wave_height_m!==undefined?{significantWaveHeightM:surface.significant_wave_height_m}:{}},sensorDepthM:Math.max(at[2],0),waterDepthM:depth};},
  defaultNoiseProfile(params:AcousticParams):NoiseProfile{validateParams(params);return{measurementStd:Math.max(.005,1500/params.frequencyHz*.02),biasStd:0,driftStdPerSqrtS:0,dropoutProbability:0};}
};
