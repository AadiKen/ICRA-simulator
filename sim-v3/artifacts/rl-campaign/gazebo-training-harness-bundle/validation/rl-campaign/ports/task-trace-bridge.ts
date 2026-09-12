import {FrozenActuatorBank,resolveSurveyorActuatorSpec,type FrozenAction} from "./shared-actuators.ts";
import {assertTraceV2,type TraceState,type TraceV2} from "./trace-schema-v2.ts";

export interface ImuSample {timestamp_s:number;valid:boolean;linear_accel_body:[number,number,number];angular_rate_body:[number,number,number];yaw_ned_rad:number;}
export interface GpsSample {timestamp_s:number;valid:boolean;position_ned_m:[number,number];}
export interface OdomSample {time_s:number;N_m:number;E_m:number;yaw_rad:number;u_mps:number;v_mps:number;r_rad_s:number;imu?:ImuSample;gps?:GpsSample;}
export interface TaskReset {seed:number;initial_state:TraceState;route_ned_m:Array<[number,number]>;disturbance:{wind_speed_m_s:number;wind_direction_deg:number;current_speed_m_s:number;current_direction_deg:number};}

/** Canonical ENU heading to wrapped NED heading conversion. */
export function enuYawToNed(yawEnu:number):number {
  const wrapped=((Math.PI/2-yawEnu+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
  // Stable language boundary: Python's exact port applies the same 15-digit
  // decimal quantization, eliminating platform-libm last-bit differences.
  return Number(wrapped.toPrecision(15));
}

/** Canonical quaternion entry point used by external IMU adapters. */
export function enuQuaternionToNedYaw(w:number,x:number,y:number,z:number):number {
  return enuYawToNed(Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)));
}

/**
 * The only simulator-specific input is calibrated/transported sensor state.
 * Actuator semantics, cadence, and the contract observation ordering remain here.
 */
export class TaskTraceBridge {
  readonly bank=new FrozenActuatorBank(resolveSurveyorActuatorSpec());
  #previousAction:FrozenAction=[0,0,0,0];
  readonly simulator:"Gazebo Harmonic"|"VRX";
  readonly reset:TaskReset;
  constructor(simulator:"Gazebo Harmonic"|"VRX",reset:TaskReset) {this.simulator=simulator;this.reset=reset;}
  resetBridge(){this.bank.reset();this.#previousAction=[0,0,0,0];}
  apply(action:FrozenAction,dt_s=.05):FrozenAction {this.#previousAction=this.bank.step(action,dt_s);return this.#previousAction;}
  sample(step:number,odom:OdomSample):TraceV2["samples"][number] {
    const goal=this.reset.route_ned_m[0] ?? [0,0],imu=odom.imu,gps=odom.gps;
    const fresh=(timestamp:number,maxAge:number)=>Number.isFinite(timestamp)&&timestamp<=odom.time_s+1e-12&&odom.time_s-timestamp<=maxAge+1e-12;
    const imuValid=Boolean(imu?.valid&&fresh(imu.timestamp_s,1/50+.01+.05));
    const gpsValid=Boolean(gps?.valid&&fresh(gps.timestamp_s,1/2+.2+.05));
    const imuFields=imuValid?[...imu!.linear_accel_body,...imu!.angular_rate_body,imu!.yaw_ned_rad]:[0,0,0,0,0,0,0];
    const gpsFields=gpsValid?[goal[0]-gps!.position_ned_m[0],goal[1]-gps!.position_ned_m[1],1]:[0,0,0];
    const normalizedTimeRemaining=Math.max(0,(2400-(step+1))/2400);
    return {step,time_s:odom.time_s,state:[odom.N_m,odom.E_m,odom.yaw_rad,odom.u_mps,odom.v_mps,odom.r_rad_s],observation:[...imuFields,...gpsFields,...this.#previousAction,normalizedTimeRemaining],applied_action:[...this.#previousAction]};
  }
  trace(actions:Array<{step:number;time_s:number;command:FrozenAction}>,samples:TraceV2["samples"]):TraceV2 {const out={schema_version:"trace-schema-v2" as const,simulator:this.simulator,reset:this.reset,action_trace:actions,samples};assertTraceV2(out);return out;}
}

/** Convert Gazebo odometry (ENU pose, body-FLU twist) to task NED/body ordering. */
export function gazeboOdomToTask(time_s:number,enu:{x:number;y:number;vx:number;vy:number;yaw_rad:number;angular_z:number},sensors?:Pick<OdomSample,"imu"|"gps">):OdomSample {
  const yaw=enuYawToNed(enu.yaw_rad);
  // OdometryPublisher serializes twist in the child/body frame.  Its X axis is
  // already forward, so rotating vx/vy as if they were world ENU applies the
  // hull heading twice.  Only FLU-left -> NED-starboard needs a sign change.
  return {time_s,N_m:enu.y,E_m:enu.x,yaw_rad:yaw,u_mps:enu.vx,v_mps:-enu.vy,r_rad_s:-enu.angular_z,...sensors};
}
