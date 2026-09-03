import {createDemoScenario} from "../../../scenarioPresets.js";
import {simulator as LegacySimulator,simState as LegacySimState,controlCommand,controlWaypoint,vec3} from "../../../schema.js";
import {RigidBodyState} from "../../../core/rigidBodyState.js";
import type {ResolvedExperimentV1} from "../../../packages/experiment-schema/src/index.ts";
import type {Observation,SimulationEngine,StepResult} from "../../../packages/core/src/simulation.ts";
import {SeededRandom} from "../../../packages/core/src/random.ts";
import {SensorRuntimeRegistry,type SensorPlugin,type SensorRuntimeCheckpointV1} from "../../../packages/sensor-sdk/src/runtime.ts";
import {BUILT_IN_SENSOR_REGISTRY,ClearSkyOcclusionService,HullMotionPlatformStateService,SnapshotPlatformStateService,platformStateFromActuationModel} from "../../../packages/sensor-sdk/src/index.ts";
import {buildVehicleBProductionConfiguration} from "./vehicle-b-production.ts";
import {buildVehicleCProductionConfiguration} from "./vehicle-c-production.ts";
import {UnresolvedBathymetryField} from "../../../packages/environment/src/index.ts";
import {LocalGeographicFrame} from "../../../packages/environment/src/geography.ts";
import {SURVEYOR_PUBLIC_SPEC,SurveyorGuidanceMapper,validateSurveyorMission} from "../../../packages/vehicle-sdk/src/surveyor.ts";
import {VehicleParameters} from "../../../core/vehicleParameters.js";

export interface ProductionAction {waypoints?:Array<{north_m:number;east_m:number}|{lat:number;lon:number}>;active_sensors?:string[];actuators?:{propeller_rps?:number;rudder_rad?:number;surgeForce?:number;yawMoment?:number;desiredWrench?:number[];effectors?:Record<string,{command?:number}>}}
function restoreVectorPrototypes(value:any):void{if(!value||typeof value!=="object")return;if(!Array.isArray(value)&&Number.isFinite(value.x)&&Number.isFinite(value.y)&&Number.isFinite(value.z)&&value.w===undefined)Object.setPrototypeOf(value,vec3.prototype);for(const child of Object.values(value))restoreVectorPrototypes(child);}

export class LegacyProductionEngine implements SimulationEngine {
  #sim?:LegacySimulator;
  #config?:ResolvedExperimentV1;
  #rng?:SeededRandom;
  #sensorRuntime?:SensorRuntimeRegistry;
  #typedOutputs:Record<string,unknown>={};
  #typedPowerW=new Map<string,number>();
  readonly #sensorFactories:Readonly<Record<string,()=>SensorPlugin>>;
  constructor(sensorFactories:Readonly<Record<string,()=>SensorPlugin>>=Object.fromEntries(Object.entries(BUILT_IN_SENSOR_REGISTRY).map(([id,registration])=>[id,registration.create]))){this.#sensorFactories=sensorFactories;}
  reset(config:ResolvedExperimentV1):Observation{
    const scenario=createDemoScenario({physicsMode:config.vehicle.plant,logEvery:1});
    if(config.vehicle.preset==="vehicle-b-rudder"){
      if(config.vehicle.plant!=="coupled6")throw new Error("Vehicle B production integration requires coupled6.");
      const production=buildVehicleBProductionConfiguration();
      scenario.boatConfig.vehicleId=production.definition.id;scenario.boatConfig.vehicleParameters=production.parameters;scenario.boatConfig.maneuveringModelParameters=production.mmg;scenario.boatConfig.productionManifest=production.manifest;
      scenario.boatConfig.mass=production.definition.mass.value;scenario.boatConfig.dimensions=new vec3(production.definition.geometry.width.value,production.definition.geometry.draft.value*2,production.definition.geometry.length.value);scenario.boatConfig.inertia=new vec3(production.definition.inertia.value[1],production.definition.inertia.value[2],production.definition.inertia.value[0]);scenario.boatConfig.maxSpeed=3;scenario.boatConfig.maxAcceleration=production.configuration.command_mapping.max_surge_force_n/production.definition.mass.value;
    }
    if(config.vehicle.preset==="vehicle-c-azimuth"){
      if(config.vehicle.plant!=="coupled6")throw new Error("Vehicle C production integration requires coupled6.");
      const production=buildVehicleCProductionConfiguration();scenario.boatConfig.vehicleId=production.definition.id;scenario.boatConfig.vehicleParameters=production.parameters;scenario.boatConfig.productionManifest=production.manifest;scenario.boatConfig.mass=production.definition.mass.value;scenario.boatConfig.dimensions=new vec3(production.definition.geometry.width.value,production.definition.geometry.draft.value*2,production.definition.geometry.length.value);scenario.boatConfig.inertia=new vec3(production.definition.inertia.value[1],production.definition.inertia.value[2],production.definition.inertia.value[0]);scenario.boatConfig.maxSpeed=3;scenario.boatConfig.maxAcceleration=1;
    }
    if(config.vehicle.preset==="searobotics-surveyor-m1.8"){
      if(config.mission.type==="surveyor-waypoint")validateSurveyorMission(config.mission);const mission=config.mission as any;
      const bootstrapNewtonsPerCommand=1,[Ix,Iy,Iz]=SURVEYOR_PUBLIC_SPEC.inertia_diagonal_kg_m2,[Xu,Yv,Nr]=SURVEYOR_PUBLIC_SPEC.damping.linear_planar,[Xuu,Yvv,Nrr]=SURVEYOR_PUBLIC_SPEC.damping.quadratic_planar,dynamics=SURVEYOR_PUBLIC_SPEC.dynamics;
      scenario.boatConfig.vehicleId=SURVEYOR_PUBLIC_SPEC.id;scenario.boatConfig.mass=SURVEYOR_PUBLIC_SPEC.mass_kg;scenario.boatConfig.dimensions=new vec3(SURVEYOR_PUBLIC_SPEC.beam_m,SURVEYOR_PUBLIC_SPEC.draft_m*2,SURVEYOR_PUBLIC_SPEC.length_m);scenario.boatConfig.inertia=new vec3(Iy,Iz,Ix);scenario.boatConfig.maxSpeed=4*.514444;scenario.boatConfig.maxAcceleration=2*70/SURVEYOR_PUBLIC_SPEC.mass_kg;scenario.boatConfig.maxTurn=1;scenario.boatConfig.guidanceActuatorMapper=new SurveyorGuidanceMapper({max_thrust_command:mission.max_thrust_command,bootstrap_newtons_per_command:bootstrapNewtonsPerCommand});scenario.boatConfig.productionManifest={vehicle_id:SURVEYOR_PUBLIC_SPEC.id,model_id:SURVEYOR_PUBLIC_SPEC.model_id,model_status:SURVEYOR_PUBLIC_SPEC.model_status,plant:"planar3",mass_kg:SURVEYOR_PUBLIC_SPEC.mass_kg,geometry_m:{length:SURVEYOR_PUBLIC_SPEC.length_m,beam:SURVEYOR_PUBLIC_SPEC.beam_m,draft:SURVEYOR_PUBLIC_SPEC.draft_m},inertia_diagonal_kg_m2:[Ix,Iy,Iz],thruster_positions_body_m:SURVEYOR_PUBLIC_SPEC.effectors.map((x:any)=>x.position_body_m),actuation:"twin-fixed-differential",hardware_api:"set_thruster_mode(thrust, thrust_diff)",command_range:dynamics.hardware_command_range,actuator_lag:{time_constant_s:dynamics.time_constant_s,provenance:"borrowed generic Vehicle A fallback; not Surveyor-specific"},calibration_status:SURVEYOR_PUBLIC_SPEC.calibration_status,unresolved:SURVEYOR_PUBLIC_SPEC.unresolved};scenario.boatConfig.vehicleParameters=VehicleParameters.fromGeometry(SURVEYOR_PUBLIC_SPEC.length_m,SURVEYOR_PUBLIC_SPEC.beam_m,SURVEYOR_PUBLIC_SPEC.draft_m,SURVEYOR_PUBLIC_SPEC.mass_kg,{id:SURVEYOR_PUBLIC_SPEC.id,height:.34,Ix,Iy,Iz,Xu,Yv,Nr,Xuu,Yvv,Nrr,maxAcceleration:scenario.boatConfig.maxAcceleration,maxThrust:70*bootstrapNewtonsPerCommand,motorTimeConstant:dynamics.time_constant_s,effectors:SURVEYOR_PUBLIC_SPEC.effectors.map((x:any)=>({id:x.id,type:"FixedThruster",pos:x.position_body_m,axis:x.axis_body,dynamics:{tau:dynamics.time_constant_s,min:dynamics.force_range_n_each[0],max:dynamics.force_range_n_each[1]},conversion:{type:"linear"}})),controlledDOF:["surge","yaw"],allocator:{mode:"pinv",saturation:"scale"}});
    }
    scenario.simConfig.simHz=1/config.experiment.timestep_s;
    scenario.simConfig.durationSec=config.experiment.duration_s;
    scenario.simConfig.seed=config.experiment.seed;
    scenario.simConfig.allowGroundTruth=false;
    scenario.goalConfig.waypoints=this.#missionWaypoints(config);
    if(config.mission.type==="rl-common-waypoint-v1"){scenario.envConfig.obstacles=[];scenario.envConfig.bounds={width:20_000,height:20_000};}
    const initial=config.initial_state;if(initial?.position_ned_m){const[n,e,d]=initial.position_ned_m;scenario.boatConfig.startPos=new vec3(e,-d,n);}if(initial?.attitude_rad){const[roll,pitch,yaw]=initial.attitude_rad;scenario.boatConfig.startOrientation=new vec3(pitch,yaw,roll);}
    const current=config.environment?.current_mps;if(current)scenario.envConfig.waterFieldConfig.current=new vec3(current[1],-current[2],current[0]);const wind=config.environment?.wind_mps;if(wind)scenario.envConfig.wind={N:wind[0],E:wind[1],D:wind[2]};
    const typedDeclarations=config.sensors.flatMap((sensor,declarationIndex)=>{const factory=this.#sensorFactories[sensor.plugin];return factory?[{sensor,declarationIndex,factory}]:[];}),typedIds=new Set(typedDeclarations.map(({sensor})=>sensor.plugin));
    scenario.sensorConfig.sensors=scenario.sensorConfig.sensors.filter((sensor:any)=>!typedIds.has(sensor.id));
    this.#sim=new LegacySimulator(scenario);if(initial?.body_velocity_mps&&this.#sim.state.boat.rigidBody){const[u,v,w]=initial.body_velocity_mps;Object.assign(this.#sim.state.boat.rigidBody.velocity,{u,v,w});}if(initial?.angular_rate_body_rad_s&&this.#sim.state.boat.rigidBody){const[p,q,r]=initial.angular_rate_body_rad_s;Object.assign(this.#sim.state.boat.rigidBody.angularRate,{p,q,r});}this.#config=config;this.#rng=new SeededRandom(config.experiment.seed);this.#typedOutputs={};this.#typedPowerW.clear();
    // Resolved experiment IDs use the authoritative typed catalog. The separate
    // browser demo may still construct explicitly labelled legacy sensor objects.
    const typedSpecs=typedDeclarations.map(({sensor,declarationIndex,factory})=>{const plugin=factory(),config=structuredClone(BUILT_IN_SENSOR_REGISTRY[sensor.plugin]?.defaultConfig??{});this.#typedPowerW.set(sensor.plugin,plugin.metadata.nominalPowerW);return{plugin,declaration:{pluginId:sensor.plugin,declarationIndex,config,enabled:sensor.enabled}};});
    this.#sensorRuntime=new SensorRuntimeRegistry(typedSpecs,config.experiment.seed,(service)=>{
      if(service==="groundTruth")return()=>this.getGroundTruth();
      if(service==="environment")return()=>this.#environmentSample();
      if(service==="bathymetry")return new UnresolvedBathymetryField();
      if(service==="raycast")return()=>null;
      if(service==="skyOcclusion")return new ClearSkyOcclusionService();
      if(service==="platformState"){const base=new SnapshotPlatformStateService(()=>platformStateFromActuationModel(this.#sim!.boatModel.actuatorModel));return new HullMotionPlatformStateService(base,()=>this.#config!.environment?.data_sources?this.#environmentSample():{surface:{}},()=>this.getGroundTruth());}
      throw new Error(`Production service ${service} is not wired yet.`);
    });
    for(const {sensor} of typedDeclarations)if(sensor.enabled)this.#sensorRuntime.setLifecycle(sensor.plugin,"ACTIVE");
    return this.#observation();
  }
  step(action:ProductionAction|null):StepResult{
    this.#ready();const command=action?new controlCommand((action.waypoints??[]).map((p)=>new controlWaypoint(this.#actionWaypoint(p),action.active_sensors??[])),action.active_sensors??[]):undefined;
    this.#sim!.step({...command?{controlCommand:command}:{},...action?.actuators?{actuatorCommand:action.actuators}:{}});
    Object.assign(this.#typedOutputs,this.#sensorRuntime!.sampleStep(this.#sim!.state.steps,this.#elapsed()));
    const active=new Set(action?.active_sensors??[]),typedSensorCost=[...this.#typedPowerW].reduce((sum,[id,power])=>active.has(id)?sum+power*this.#config!.experiment.timestep_s:sum,0);
    if(typedSensorCost){const metrics=this.#sim!.state.metrics;metrics.lastSensorCost+=typedSensorCost;metrics.lastTotalCost=metrics.lastSensorCost+metrics.lastMovementCost;metrics.totalSensorCost+=typedSensorCost;metrics.totalEnergy=metrics.totalSensorCost+metrics.movementCost;}
    const model=this.#sim!.boatModel.maneuveringModel,actuator=this.#sim!.boatModel.actuatorModel,hardwareCommand=this.#sim!.state.boat.lastActuatorCommand?.hardware_command,vehicleDiagnostics=model?{kind:"vehicle-b-mmg",applied_command:structuredClone(model.lastCommand),force_components:structuredClone(model.lastBreakdown),full_wrench:[...model.lastFullWrench]}:{kind:hardwareCommand?"searobotics-surveyor":"generic-actuator-allocation",hardware_command:structuredClone(hardwareCommand??null),applied_command:structuredClone(actuator.lastEffectorCommands),allocation_diagnostics:structuredClone(actuator.lastAllocationDiagnostics),effectors:actuator.effectors.map((effector:any)=>({id:effector.id,command:effector.command,value:effector.value,thrust:effector.thrust,azimuth:effector.azimuth})),full_wrench:[...actuator.lastFullWrench]};
    return{observation:this.#observation(),reward:0,terminated:this.#sim!.state.goal.completed||this.#sim!.state.goal.failed,truncated:this.#sim!.state.stopReason==="duration_elapsed",info:{stop_reason:this.#sim!.state.stopReason,validation_scope:this.#config!.vehicle.preset==="vehicle-b-rudder"?"Vehicle B MMG integrated through production coupled6; USV coefficients unvalidated":this.#config!.vehicle.preset==="vehicle-c-azimuth"?"Vehicle C allocation/composability demonstrated; behavioral dynamics unvalidated":"MSS-validated planar3 production physics",vehicle_path:this.#sim!.boatModel.boatConfig.productionManifest??null,vehicle_diagnostics:vehicleDiagnostics}};
  }
  getGroundTruth(){this.#ready();const b=this.#sim!.state.boat;return{time_s:this.#elapsed(),position_ned_m:[b.pos.z,b.pos.x,-b.pos.y],attitude_rad:[b.orientation.z,b.orientation.x,b.heading],velocity_body_mps:b.rigidBody?[b.rigidBody.velocity.u,b.rigidBody.velocity.v,b.rigidBody.velocity.w]:[b.velocity.z,b.velocity.x,-b.velocity.y],angular_rate_body_rad_s:b.rigidBody?[b.rigidBody.angularRate.p,b.rigidBody.angularRate.q,b.rigidBody.angularRate.r]:[b.angularVel.z,b.angularVel.x,b.angularVel.y],acceleration_body_mps2:b.rigidBody?[b.rigidBody.acceleration.uDot,b.rigidBody.acceleration.vDot,b.rigidBody.acceleration.wDot]:[b.acceleration.z,b.acceleration.x,-b.acceleration.y]};}
  getMetrics(){this.#ready();const m=this.#sim!.state.metrics;return{total_energy:m.totalEnergy,total_sensor_cost:m.totalSensorCost,movement_cost:m.movementCost,steps:this.#sim!.state.steps,elapsed_s:this.#elapsed()};}
  saveState(){this.#ready();const coupledPlant=this.#sim!.boatModel.coupledPlant;return {schema_version:3,simState:structuredClone(this.#sim!.state),actuatorState:this.#sim!.boatModel.savePropulsionState(),rngState:this.#rng!.saveState(),sensorRuntimeState:this.#sensorRuntime!.saveState(),typedOutputs:structuredClone(this.#typedOutputs),plantRuntimeState:{coupled6:{equilibriumD:coupledPlant.equilibriumD,lastWrench:structuredClone(coupledPlant.lastWrench),forceBreakdown:structuredClone(coupledPlant.forceBreakdown)}}};}
  loadState(state:unknown){
    this.#ready();const checkpoint=structuredClone(state) as any;
    const legacy=checkpoint?.schema_version===undefined;
    if(!legacy&&![2,3].includes(checkpoint.schema_version))throw new Error("Unsupported production-engine checkpoint version.");
    if(legacy&&this.#sensorRuntime!.evaluationOrder.length)throw new Error("Legacy checkpoint cannot restore an engine with typed sensor plugins.");
    if(!legacy)this.#sensorRuntime!.validateState(checkpoint.sensorRuntimeState as SensorRuntimeCheckpointV1);
    const restored=checkpoint.simState;Object.setPrototypeOf(restored,LegacySimState.prototype);restoreVectorPrototypes(restored);for(const boat of [restored.boat,restored.boatBelief])if(boat?.rigidBody)Object.setPrototypeOf(boat.rigidBody,RigidBodyState.prototype);
    this.#sim!.state=restored;this.#sim!.boatModel.loadPropulsionState(checkpoint.actuatorState);this.#rng!.loadState(checkpoint.rngState);if(!legacy){this.#sensorRuntime!.loadState(checkpoint.sensorRuntimeState);this.#typedOutputs=structuredClone(checkpoint.typedOutputs??{});const coupledPlant=this.#sim!.boatModel.coupledPlant,plantState=checkpoint.plantRuntimeState?.coupled6;if(this.#config!.vehicle.plant==="coupled6"&&!plantState)throw new Error("Coupled6 checkpoint lacks plant runtime state; replay would rebase hydrostatics.");if(plantState){if(plantState.equilibriumD!==null&&!Number.isFinite(plantState.equilibriumD))throw new Error("Invalid coupled6 hydrostatic reference depth.");coupledPlant.equilibriumD=plantState.equilibriumD;coupledPlant.lastWrench=structuredClone(plantState.lastWrench??Array(6).fill(0));coupledPlant.forceBreakdown=structuredClone(plantState.forceBreakdown??{});}}
  }
  nextRandomForService(){this.#ready();return this.#rng!.next();}
  dispose(){this.#sensorRuntime?.dispose();this.#sim=undefined;this.#config=undefined;this.#rng=undefined;this.#sensorRuntime=undefined;this.#typedOutputs={};this.#typedPowerW.clear();}
  #missionWaypoints(config:ResolvedExperimentV1):vec3[]{if(config.vehicle.preset==="searobotics-surveyor-m1.8"&&config.mission.type==="surveyor-waypoint"){validateSurveyorMission(config.mission);const frame=new LocalGeographicFrame({latitude_deg:config.mission.origin.lat,longitude_deg:config.mission.origin.lon},config.mission.origin.heading_deg??0);return config.mission.waypoints.map(point=>{const ned=frame.wgs84ToNed({latitude_deg:point.lat,longitude_deg:point.lon});return new vec3(ned[1],-ned[2],ned[0]);});}const raw=Array.isArray(config.mission.waypoints)?config.mission.waypoints as Array<{north_m:number;east_m:number}>:[];return raw.length?raw.map((p)=>new vec3(p.east_m,0,p.north_m)):[new vec3(22,0,22)];}
  #actionWaypoint(point:{north_m:number;east_m:number}|{lat:number;lon:number}){if("lat" in point){if(this.#config!.vehicle.preset!=="searobotics-surveyor-m1.8")throw new Error("Raw lat/lon action waypoints require the Surveyor preset.");validateSurveyorMission(this.#config!.mission);const origin=this.#config!.mission.origin,frame=new LocalGeographicFrame({latitude_deg:origin.lat,longitude_deg:origin.lon},origin.heading_deg??0),ned=frame.wgs84ToNed({latitude_deg:point.lat,longitude_deg:point.lon});return new vec3(ned[1],-ned[2],ned[0]);}return new vec3(point.east_m,0,point.north_m);}
  #elapsed(){return this.#sim!.state.time-this.#sim!.state.startTime;}
  #observation():Observation{this.#ready();const belief=this.#sim!.state.boatBelief;return{time_s:this.#elapsed(),sensors:{...structuredClone(this.#sim!.state.lastObservation??{}),...structuredClone(this.#typedOutputs)},estimated_state:{position_ned_m:[belief.pos.z,belief.pos.x,-belief.pos.y],heading_rad:belief.heading}};}
  #environmentSample(){const environment=this.#config!.environment??{},local=this.#sim!.state.localEnv;return{wind_ned_mps:environment.wind_mps??[0,0,0],current_ned_mps:environment.current_mps??[0,0,0],air_temperature_c:15,pressure_pa:101325,humidity_fraction:.5,rain_rate_mm_h:0,visibility_m:10000,fog_extinction_per_m:0,illumination_lux:10000,atmosphere:{wind_ned_mps:environment.wind_mps??[0,0,0],air_temperature_c:15,pressure_pa:101325,humidity_fraction:.5,rain_rate_mm_h:0,visibility_m:10000,fog_extinction_per_m:0,illumination_lux:10000},water:{current_ned_mps:environment.current_mps??[0,0,0]},surface:{...local?.waveAmplitudeM!==undefined?{significant_wave_height_m:2*local.waveAmplitudeM}:{}},electromagnetic:{},positioning:{sky_view_fraction:1,visible_satellites:12,hdop:1,vdop:1.5,pdop:1.8,multipath_factor:0,gnss_interference_factor:0}};}
  #ready(){if(!this.#sim||!this.#config||!this.#sensorRuntime)throw new Error("Production engine must be reset before use.");}
}
