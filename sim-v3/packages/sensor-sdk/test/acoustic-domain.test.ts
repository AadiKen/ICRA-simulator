import assert from "node:assert/strict";
import {AcousticDomain,acousticPrimitives} from "../src/domains/acoustic.ts";

const params={frequencyHz:200000},calm={water:{temperatureC:10,salinityPsu:35,pressurePa:101325},surface:{seaStateBeaufort:0},sensorDepthM:0},rough={...calm,surface:{seaStateBeaufort:7}};
const losses=[1,10,100,1000].map((range)=>acousticPrimitives.transmissionLossDb(range,params,calm));for(let index=1;index<losses.length;index++)assert.ok(losses[index]>losses[index-1],"Transmission loss must increase monotonically with range.");
for(const snr of [-100,-10,0,10,100]){const probability=acousticPrimitives.detectionProbability(snr);assert.ok(probability>=0&&probability<=1,"Detection probability must remain bounded.");}assert.ok(acousticPrimitives.detectionProbability(20)>acousticPrimitives.detectionProbability(-20));
assert.ok(acousticPrimitives.ambientNoiseDb(params,rough)>acousticPrimitives.ambientNoiseDb(params,calm),"Ambient noise must rise with sea state.");
const derived=acousticPrimitives.soundSpeedMps(params,calm);assert.ok(derived>1400&&derived<1600);const profiled={...calm,water:{...calm.water,soundSpeedProfile:[{depth_m:0,sound_speed_mps:1490},{depth_m:100,sound_speed_mps:1510}]}};assert.equal(acousticPrimitives.soundSpeedMps(params,profiled,50),1500,"A supplied profile must override formula-derived sound speed.");
const grouped:any={wind_ned_mps:[0,0,0],current_ned_mps:[0,0,0],air_temperature_c:1,pressure_pa:1,humidity_fraction:0,rain_rate_mm_h:0,visibility_m:1,fog_extinction_per_m:0,illumination_lux:1,water:{current_ned_mps:[0,0,0],temperature_c:7,salinity_psu:33,pressure_pa:202650},surface:{sea_state_beaufort:4,significant_wave_height_m:2}};const queried=AcousticDomain.queryEnvironment({environment:()=>grouped},[0,0,3]);assert.equal(queried.water.temperatureC,7);assert.equal(queried.surface.seaStateBeaufort,4);assert.equal(queried.sensorDepthM,3);
assert.deepEqual(acousticPrimitives.transmissionLossDb(25,params,calm),acousticPrimitives.transmissionLossDb(25,params,calm),"Domain primitives must be stateless and repeatable.");
console.log("Acoustic domain contract tests passed.");
