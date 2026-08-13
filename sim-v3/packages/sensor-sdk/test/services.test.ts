import assert from "node:assert/strict";
import {platformStateFromActuationModel,PublishedSensorsService,SnapshotPlatformStateService} from "../src/services/index.ts";

const platform=platformStateFromActuationModel({effectors:[{id:"port",command:.5,thrust:80,omega:14,lastPowerW:120,energyJ:30,propulsionEnergyJ:25,failureMode:"healthy"}],getEnergyMetrics:()=>({actuator_energy_j:30,propulsion_energy_j:25,actuator_power_w:120})});
assert.deepEqual(platform.actuator_states[0],{id:"port",command:.5,thrust_n:80,rotor_speed_rad_s:14,power_w:120,energy_j:30,propulsion_energy_j:25,failure_mode:"healthy"});assert.equal(platform.actuator_energy_j,30);assert.equal("battery_voltage_v" in platform,false);
const platformService=new SnapshotPlatformStateService(()=>platform),copy=platformService.sample();copy.actuator_states[0].command=0;assert.equal(platformService.sample().actuator_states[0].command,.5);

const original:any={stepIndex:1,timestampS:.1,valid:true,status:"ACTIVE",payload:{position:[1,2,3]},powerW:1,bytes:24};const published=new PublishedSensorsService((id)=>id==="gps"?original:null),read=published.latest("gps")!;(read.payload as any).position[0]=99;assert.equal(original.payload.position[0],1);assert.equal(published.latest("missing"),null);
console.log("Sensor controlled-service tests passed.");
