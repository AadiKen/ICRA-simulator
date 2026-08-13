import assert from "node:assert/strict";
import {BUILT_IN_SENSOR_CAPABILITIES,BUILT_IN_SENSOR_REGISTRY,DepthSounderPlugin,SensorLifecycle,STANDARD_SENSOR_IDS,SurveySonar38KhzPlugin} from "../src/index.ts";
import {ConstantBathymetryField} from "@bcod/environment";

const lifecycle=new SensorLifecycle(1,2,1,2,10);lifecycle.transition("STARTING");lifecycle.transition("WARMING");lifecycle.transition("ACTIVE");assert.equal(lifecycle.power(),10);assert.throws(()=>lifecycle.transition("STARTING"),/Invalid/);
const environment={wind_ned_mps:[0,0,0],current_ned_mps:[0,0,0],air_temperature_c:15,pressure_pa:101325,humidity_fraction:.5,rain_rate_mm_h:0,visibility_m:10000,fog_extinction_per_m:0,illumination_lux:10000,water:{current_ned_mps:[0,0,0],temperature_c:10,salinity_psu:35,pressure_pa:101325},surface:{sea_state_beaufort:0}};
const sensor=new DepthSounderPlugin();sensor.init({mount_m:[0,0,0],direction_body:[0,0,1],min_range_m:.2,max_range_m:50,frequency_hz:200000,noise_std_m:0,power_w:8},{seededRng:()=>.5,environment:()=>environment,bathymetry:new ConstantBathymetryField(12)});
assert.equal(sensor.sample({stepIndex:0,simTimeS:0,lifecycleState:"WARMING"}),null);
const sample=sensor.sample({stepIndex:1,simTimeS:.1,lifecycleState:"ACTIVE"});assert.equal(sample?.valid,true);assert.equal(sample?.payload?.depth_m,12);assert.ok(Number.isFinite(sample?.payload?.sound_speed_mps));assert.ok((sample?.payload?.transmission_loss_db??0)>0);sensor.dispose();
const survey=new SurveySonar38KhzPlugin();assert.equal(survey.metadata.domainParams.frequencyHz,38000);assert.equal(survey.metadata.id,"survey-sonar-38khz");
assert.deepEqual(STANDARD_SENSOR_IDS,BUILT_IN_SENSOR_CAPABILITIES.map(({id})=>id));
for(const capability of BUILT_IN_SENSOR_CAPABILITIES){const plugin=BUILT_IN_SENSOR_REGISTRY[capability.id].create();assert.equal(plugin.metadata.id,capability.id);assert.equal(plugin.metadata.domain,capability.domain);plugin.dispose();}
assert.equal(BUILT_IN_SENSOR_REGISTRY.camera.create().metadata.outputPayloadClass,"camera");
assert.equal(BUILT_IN_SENSOR_REGISTRY.lidar.create().metadata.outputPayloadClass,"lidar");
console.log("Sensor SDK tests passed.");
