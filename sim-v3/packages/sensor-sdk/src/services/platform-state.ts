export interface PlatformActuatorState{
  id:string;command:number;thrust_n?:number;rotor_speed_rad_s?:number;power_w?:number;energy_j?:number;propulsion_energy_j?:number;failure_mode?:string;
}
export interface PlatformStateSample{
  actuator_states:PlatformActuatorState[];
  actuator_energy_j:number;
  propulsion_energy_j:number;
  actuator_power_w:number;
  component_temperature_c?:Record<string,number>;
  vibration_rms_mps2?:number;
}
export interface PlatformStateService{sample():PlatformStateSample}
export interface HullMotionSample{acceleration_body_mps2?:number[];angular_rate_body_rad_s?:number[]}
export interface SeaStateSample{surface?:{sea_state_beaufort?:number;significant_wave_height_m?:number}}

export class SnapshotPlatformStateService implements PlatformStateService{
  readonly #read:()=>PlatformStateSample;
  constructor(read:()=>PlatformStateSample){this.#read=read;}
  sample():PlatformStateSample{return structuredClone(this.#read());}
}

/**
 * High-frequency hull disturbance omitted by the rigid-body plant. The actual
 * 6-DOF acceleration/rates only modulate this sea-state floor; they are never
 * added to it, because the IMU ground-truth signal already contains them.
 */
export function seaStateHullVibrationRmsMps2(environment:SeaStateSample,motion:HullMotionSample={}):number{
  const surface=environment.surface??{},height=surface.significant_wave_height_m,beaufort=surface.sea_state_beaufort;
  if(height===undefined&&beaufort===undefined)return 0;
  for(const [name,value] of [["significant wave height",height],["sea state Beaufort",beaufort]] as const)if(value!==undefined&&(!Number.isFinite(value)||value<0))throw new Error(`${name} must be finite and non-negative.`);
  const hs=height??0,b=beaufort??Math.min(12,Math.max(0,1.7*Math.sqrt(hs)));
  const acceleration=motion.acceleration_body_mps2??[0,0,0],rates=motion.angular_rate_body_rad_s??[0,0,0];
  const response=Math.hypot(acceleration[2]??0,2*(rates[0]??0),2*(rates[1]??0));
  const responseModifier=1+Math.min(Math.max(response,0)/4,.5);
  return (.008*b**1.35+.018*hs)*responseModifier;
}

export class HullMotionPlatformStateService implements PlatformStateService{
  readonly #base:PlatformStateService;readonly #environment:()=>SeaStateSample;readonly #motion:()=>HullMotionSample;
  constructor(base:PlatformStateService,environment:()=>SeaStateSample,motion:()=>HullMotionSample){this.#base=base;this.#environment=environment;this.#motion=motion;}
  sample():PlatformStateSample{return{...this.#base.sample(),vibration_rms_mps2:seaStateHullVibrationRmsMps2(this.#environment(),this.#motion())};}
}

interface ExistingActuationSurface{
  effectors?:Array<{id?:string;command?:number;thrust?:number;omega?:number;lastPowerW?:number;energyJ?:number;propulsionEnergyJ?:number;failureMode?:string}>;
  getEnergyMetrics?:()=>{actuator_energy_j?:number;propulsion_energy_j?:number;actuator_power_w?:number};
}
/** Adapter for fields already carried by the current actuator implementation. */
export function platformStateFromActuationModel(model:ExistingActuationSurface):PlatformStateSample{
  const actuator_states=(model.effectors??[]).map((effector,index)=>({id:effector.id??`actuator-${index}`,command:effector.command??0,...effector.thrust!==undefined?{thrust_n:effector.thrust}:{},...effector.omega!==undefined?{rotor_speed_rad_s:effector.omega}:{},...effector.lastPowerW!==undefined?{power_w:effector.lastPowerW}:{},...effector.energyJ!==undefined?{energy_j:effector.energyJ}:{},...effector.propulsionEnergyJ!==undefined?{propulsion_energy_j:effector.propulsionEnergyJ}:{},...effector.failureMode!==undefined?{failure_mode:effector.failureMode}:{}}));
  const measured=model.getEnergyMetrics?.();
  return{actuator_states,actuator_energy_j:measured?.actuator_energy_j??actuator_states.reduce((sum,value)=>sum+(value.energy_j??0),0),propulsion_energy_j:measured?.propulsion_energy_j??actuator_states.reduce((sum,value)=>sum+(value.propulsion_energy_j??0),0),actuator_power_w:measured?.actuator_power_w??actuator_states.reduce((sum,value)=>sum+(value.power_w??0),0)};
}
