/** Shared external-simulator trace contract for the time-aware Surveyor task. */
export const TRACE_SCHEMA_V2={
  schema_version:"trace-schema-v2",
  task_contract:{task_id:"common-waypoint-transit-v1",content_sha256:"63eb33ad66078e1f28b20504d4187c9d0efd42538dd018f592a307f0f98b55a2"},
  timing:{physics_timestep_s:.05,control_interval_s:.1,episode_length_steps:2400},
  coordinate_convention:{world:"local NED",state:"[N_m, E_m, yaw_rad, u_mps, v_mps, r_rad_s]",yaw:"clockwise from north",body_velocity:"surge/sway in the vessel body frame",gazebo_enu_mapping:{position:"N=enu.y; E=enu.x; D=-enu.z",yaw:"yaw_ned=pi/2-yaw_enu",yaw_rate:"r_ned=-r_enu",body_velocity:"convert ENU world velocity to N/E first, then use nedWorldToBody2D(N_dot,E_dot,yaw_ned)"}},
  reset:{required:["seed","initial_state","route_ned_m","disturbance"],initial_state:{fields:["N_m","E_m","yaw_rad","u_mps","v_mps","r_rad_s"]},disturbance:{fields:["wind_speed_m_s","wind_direction_deg","current_speed_m_s","current_direction_deg"]}},
  action_trace:{required:["step","time_s","command"],command:{fields:["effector_0","effector_1","steer_0","steer_1"],range:[-1,1],unused_fields_must_be_zero:["steer_0","steer_1"]}},
  sample:{required:["step","time_s","state","observation","applied_action"],state:{fields:["N_m","E_m","yaw_rad","u_mps","v_mps","r_rad_s"]},observation:{encoding:"float64-vector",fields:["imu.linear_accel_x","imu.linear_accel_y","imu.linear_accel_z","imu.angular_rate_x","imu.angular_rate_y","imu.angular_rate_z","gps.relative_goal_north_m","gps.relative_goal_east_m","gps.ground_speed_north_m_s","gps.ground_speed_east_m_s","gps.fix_valid","previous_action.effector_0","previous_action.effector_1","previous_action.steer_0","previous_action.steer_1","normalized_time_remaining"],time_remaining:"remaining episode physics steps / 2400; appended as field 16"},applied_action:{fields:["effector_0","effector_1","steer_0","steer_1"],meaning:"actual normalized actuator command applied after any simulator-native lag/transport handling; never a derived body wrench"}},
  trace:{required:["schema_version","simulator","reset","action_trace","samples"],sample_interval_s:.05}
} as const;

export type ActuatorCommand=[number,number,number,number];
export type TraceState=[number,number,number,number,number,number];
export interface TraceSampleV2 {step:number;time_s:number;state:TraceState;observation:number[];applied_action:ActuatorCommand;}
export interface TraceV2 {schema_version:"trace-schema-v2";simulator:"Gazebo Harmonic"|"VRX";reset:{seed:number;initial_state:TraceState;route_ned_m:Array<[number,number]>;disturbance:{wind_speed_m_s:number;wind_direction_deg:number;current_speed_m_s:number;current_direction_deg:number}};action_trace:Array<{step:number;time_s:number;command:ActuatorCommand}>;samples:TraceSampleV2[];}

export function assertTraceV2(trace:TraceV2):void {
  if(trace.schema_version!==TRACE_SCHEMA_V2.schema_version) throw new Error("Unsupported trace schema.");
  if(trace.action_trace.length!==trace.samples.length/2) throw new Error("Trace control/physics sample cadence mismatch.");
  for(const [index,sample] of trace.samples.entries()){
    const command=trace.action_trace[Math.floor(index/2)];
    if(sample.step!==index||sample.time_s!==index*.05||command.step!==Math.floor(index/2)||command.time_s!==Math.floor(index/2)*.1) throw new Error(`Invalid trace timestamp at step ${index}.`);
    if(sample.state.length!==6||sample.observation.length!==16||sample.applied_action.length!==4||command.command.length!==4) throw new Error(`Invalid trace field width at step ${index}.`);
    if(sample.observation[15] < 0 || sample.observation[15] > 1) throw new Error(`Invalid normalized time remaining at step ${index}.`);
    if(sample.applied_action[2]!==0||sample.applied_action[3]!==0||command.command[2]!==0||command.command[3]!==0) throw new Error(`Non-zero unused actuator at step ${index}.`);
  }
}
