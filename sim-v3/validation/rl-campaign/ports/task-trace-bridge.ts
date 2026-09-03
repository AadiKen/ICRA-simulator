import {FrozenActuatorBank,resolveSurveyorActuatorSpec,type FrozenAction} from "./shared-actuators.ts";
import {assertTraceV2,type TraceState,type TraceV2} from "./trace-schema-v2.ts";

export interface OdomSample {time_s:number;N_m:number;E_m:number;yaw_rad:number;u_mps:number;v_mps:number;r_rad_s:number;imu_linear_accel_body?:[number,number,number];imu_angular_rate_body?:[number,number,number];gps_fix_valid?:number;}
export interface TaskReset {seed:number;initial_state:TraceState;route_ned_m:Array<[number,number]>;disturbance:{wind_speed_m_s:number;wind_direction_deg:number;current_speed_m_s:number;current_direction_deg:number};}

/**
 * The only simulator-specific input is calibrated/transported sensor state.
 * Actuator semantics, cadence, and the contract observation ordering remain here.
 */
export class TaskTraceBridge {
  readonly bank=new FrozenActuatorBank(resolveSurveyorActuatorSpec());
  #previousAction:FrozenAction=[0,0,0,0];
  #previousOdom:OdomSample|null=null;
  readonly simulator:"Gazebo Harmonic"|"VRX";
  readonly reset:TaskReset;
  constructor(simulator:"Gazebo Harmonic"|"VRX",reset:TaskReset) {this.simulator=simulator;this.reset=reset;}
  resetBridge(){this.bank.reset();this.#previousAction=[0,0,0,0];this.#previousOdom=null;}
  apply(action:FrozenAction,dt_s=.05):FrozenAction {this.#previousAction=this.bank.step(action,dt_s);return this.#previousAction;}
  sample(step:number,odom:OdomSample):TraceV2["samples"][number] {
    const goal=this.reset.route_ned_m[0] ?? [0,0];
    const previous=this.#previousOdom;
    const accel=odom.imu_linear_accel_body ?? (previous ? [(odom.u_mps-previous.u_mps)/Math.max(odom.time_s-previous.time_s,1e-9),(odom.v_mps-previous.v_mps)/Math.max(odom.time_s-previous.time_s,1e-9),0] : [0,0,0]);
    const angular=odom.imu_angular_rate_body ?? [0,0,odom.r_rad_s];
    this.#previousOdom=odom;
    const normalizedTimeRemaining=Math.max(0,(2400-(step+1))/2400);
    return {step,time_s:odom.time_s,state:[odom.N_m,odom.E_m,odom.yaw_rad,odom.u_mps,odom.v_mps,odom.r_rad_s],observation:[...accel,...angular,goal[0]-odom.N_m,goal[1]-odom.E_m,Math.cos(odom.yaw_rad)*odom.u_mps-Math.sin(odom.yaw_rad)*odom.v_mps,Math.sin(odom.yaw_rad)*odom.u_mps+Math.cos(odom.yaw_rad)*odom.v_mps,odom.gps_fix_valid ?? 1,...this.#previousAction,normalizedTimeRemaining],applied_action:[...this.#previousAction]};
  }
  trace(actions:Array<{step:number;time_s:number;command:FrozenAction}>,samples:TraceV2["samples"]):TraceV2 {const out={schema_version:"trace-schema-v2" as const,simulator:this.simulator,reset:this.reset,action_trace:actions,samples};assertTraceV2(out);return out;}
}

/** Convert Gazebo ENU true odometry to the NED/body ordering required by Node. */
export function gazeboOdomToTask(time_s:number,enu:{x:number;y:number;vx:number;vy:number;yaw_rad:number;angular_z:number},imu?:Omit<OdomSample,"time_s"|"N_m"|"E_m"|"yaw_rad"|"u_mps"|"v_mps"|"r_rad_s">):OdomSample {
  const yaw=Math.PI/2-enu.yaw_rad, Ndot=enu.vy, Edot=enu.vx;
  return {time_s,N_m:enu.y,E_m:enu.x,yaw_rad:yaw,u_mps:Math.cos(yaw)*Ndot+Math.sin(yaw)*Edot,v_mps:-Math.sin(yaw)*Ndot+Math.cos(yaw)*Edot,r_rad_s:-enu.angular_z,...imu};
}
