import assert from "node:assert/strict";
import {DepthSounderPlugin,SensorRuntimeRegistry} from "../src/index.ts";
import {ConstantBathymetryField} from "@bcod/environment";

const config={mount_m:[0,0,2] as [number,number,number],direction_body:[0,0,1] as [number,number,number],min_range_m:.2,max_range_m:100,frequency_hz:120000,noise_std_m:.04,source_level_db:170,power_w:8};
const environment={wind_ned_mps:[0,0,0],current_ned_mps:[0,0,0],air_temperature_c:15,pressure_pa:101325,humidity_fraction:.5,rain_rate_mm_h:0,visibility_m:10000,fog_extinction_per_m:0,illumination_lux:10000,water:{current_ned_mps:[0,0,0],temperature_c:9,salinity_psu:34,pressure_pa:121000},surface:{sea_state_beaufort:2}};
const registry=new SensorRuntimeRegistry([{plugin:new DepthSounderPlugin(120000),declaration:{pluginId:"single-beam-depth-sounder",declarationIndex:0,config,enabled:true}}],123,(service)=>{if(service==="environment")return()=>environment;if(service==="bathymetry")return new ConstantBathymetryField(20);throw new Error(`Unexpected service ${service}`);});registry.setLifecycle("single-beam-depth-sounder","ACTIVE");
for(let step=0;step<5;step++)registry.sampleStep(step,step*.1);const checkpoint=JSON.parse(JSON.stringify(registry.saveState()));const first=[];for(let step=5;step<15;step++)first.push(registry.sampleStep(step,step*.1));registry.loadState(checkpoint);const replay=[];for(let step=5;step<15;step++)replay.push(registry.sampleStep(step,step*.1));assert.deepEqual(replay,first,"Real depth-sounder output and Gaussian spare state must replay bit-identically.");registry.dispose();
console.log("Depth sounder checkpoint replay tests passed.");
