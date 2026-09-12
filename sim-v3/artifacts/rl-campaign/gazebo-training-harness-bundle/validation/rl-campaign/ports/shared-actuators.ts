import {createDemoScenario} from "../../../scenarioPresets.js";
import {VehicleParameters} from "../../../core/vehicleParameters.js";
import {buildEffectors} from "../../../packages/core/src/actuators.js";
import {SURVEYOR_PUBLIC_SPEC} from "../../../packages/vehicle-sdk/src/surveyor.ts";

export type FrozenAction=[number,number,number,number];
export interface EffectorSpec {id:string;tau_s:number;min:number;max:number;rate_limit_per_s:number|null;position_body_m:[number,number,number];axis_body:[number,number,number];mapping:{action_index:number;normalisation:"linear"};}
export interface ActuatorSpec {vehicle:"vehicle-a-otter"|"searobotics-surveyor-m1.8";effectors:[EffectorSpec,EffectorSpec];unused_action_indices:[2,3];resolution_path:string;}

/** Resolve from the same default scenario inputs used by LegacyProductionEngine. */
export function resolveVehicleAActuatorSpec():ActuatorSpec {
  const scenario:any=createDemoScenario({physicsMode:"planar3"}),boat=scenario.boatConfig ?? scenario.boat ?? scenario;
  const parameters=VehicleParameters.fromGeometry(Math.max(boat.dimensions.z||1,.001),Math.max(boat.dimensions.x||1,.001),Math.max(boat.hydrodynamics.draft||boat.dimensions.y*.25,.001),Math.max(boat.mass||1,.001),{id:boat.vehicleId||"bcod_usv",height:Math.max(boat.dimensions.y||1,.001),maxAcceleration:boat.maxAcceleration,maxThrust:boat.maxThrust||Math.max(boat.mass*boat.maxAcceleration,1),motorTimeConstant:boat.motorTimeConstant||.35});
  const effectors=buildEffectors(parameters).filter((e:any)=>e.type==="FixedThruster").slice(0,2).map((e:any,index:number)=>({id:e.id,tau_s:e.dynamics.tau,min:e.dynamics.min,max:e.dynamics.max,rate_limit_per_s:Number.isFinite(e.dynamics.rateMax)?e.dynamics.rateMax:null,position_body_m:[...e.pos] as [number,number,number],axis_body:[...e.axis] as [number,number,number],mapping:{action_index:index,normalisation:"linear" as const}}));
  if(effectors.length!==2) throw new Error("Vehicle A must resolve two fixed thrusters.");
  return {vehicle:"vehicle-a-otter",effectors:effectors as [EffectorSpec,EffectorSpec],unused_action_indices:[2,3],resolution_path:"createDemoScenario -> boatModel -> VehicleParameters.fromGeometry -> ScalarEffector"};
}

/** Exact serialization of the established Surveyor single source of truth. */
export function resolveSurveyorActuatorSpec():ActuatorSpec {
  const dynamics=SURVEYOR_PUBLIC_SPEC.dynamics;
  const effectors=SURVEYOR_PUBLIC_SPEC.effectors.map((effector:any,index:number)=>({
    id:effector.id,
    tau_s:dynamics.time_constant_s,
    min:dynamics.force_range_n_each[0],
    max:dynamics.force_range_n_each[1],
    rate_limit_per_s:dynamics.rate_limit_per_s,
    position_body_m:[...effector.position_body_m] as [number,number,number],
    axis_body:[...effector.axis_body] as [number,number,number],
    mapping:{action_index:index,normalisation:"linear" as const}
  })) as [EffectorSpec,EffectorSpec];
  return {vehicle:"searobotics-surveyor-m1.8",effectors,unused_action_indices:[2,3],resolution_path:"SURVEYOR_PUBLIC_SPEC -> per-effector shared actuator lag"};
}

/** Yaw-first projection onto |surge|/140 + |yaw|/46.2 <= 1. */
export function allocateSurveyorWrenchYawFirst(surge_n:number,yaw_nm:number):FrozenAction {
  const forceLimit=SURVEYOR_PUBLIC_SPEC.dynamics.force_range_n_each[1];
  const arm=Math.abs(SURVEYOR_PUBLIC_SPEC.effectors[0].position_body_m[1]);
  const boundedYaw=Math.max(-2*forceLimit*arm,Math.min(2*forceLimit*arm,yaw_nm));
  const differential=boundedYaw/(2*arm);
  const remaining=Math.max(0,forceLimit-Math.abs(differential));
  const halfSurge=Math.max(-remaining,Math.min(remaining,surge_n/2));
  return [(halfSurge-differential)/forceLimit,(halfSurge+differential)/forceLimit,0,0];
}

export class FrozenActuatorBank {
  readonly spec:ActuatorSpec;
  #applied_newtons:[number,number]=[0,0];
  constructor(spec:ActuatorSpec=resolveVehicleAActuatorSpec()){this.spec=spec;}
  reset(){this.#applied_newtons=[0,0];}
  step(action:FrozenAction,dt_s:number):FrozenAction {
    if(action[2]!==0||action[3]!==0) throw new Error("Vehicle A requires zero unused actuator fields.");
    this.#applied_newtons=this.spec.effectors.map((e,index)=>{
      const normalized=Math.max(-1,Math.min(1,action[e.mapping.action_index]));
      const target=normalized>=0?normalized*e.max:normalized*Math.abs(e.min);
      const rawDelta=(target-this.#applied_newtons[index])*(1-Math.exp(-dt_s/Math.max(e.tau_s,.001)));
      const maxDelta=e.rate_limit_per_s===null?Infinity:Math.max(e.rate_limit_per_s,0)*Math.max(dt_s,0);
      return this.#applied_newtons[index]+Math.max(-maxDelta,Math.min(maxDelta,rawDelta));
    }) as [number,number];
    return this.#applied_newtons.map((value,index)=>{const e=this.spec.effectors[index];return value>=0?value/e.max:value/Math.abs(e.min);}).concat([0,0]) as FrozenAction;
  }
  thrustNewtons():[number,number]{return [...this.#applied_newtons];}
}
