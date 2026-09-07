import assert from "node:assert/strict";
import {TaskTraceBridge,gazeboOdomToTask,type OdomSample,type TaskReset} from "./task-trace-bridge.ts";

const reset:TaskReset={seed:1,initial_state:[10,20,.1,0,0,0],route_ned_m:[[30,50]],disturbance:{wind_speed_m_s:0,wind_direction_deg:0,current_speed_m_s:0,current_direction_deg:0}};
const bridge=new TaskTraceBridge("VRX",reset);
const privileged:OdomSample={time_s:1,N_m:999,E_m:-999,yaw_rad:2,u_mps:8,v_mps:-7,r_rad_s:4};
const absent=bridge.sample(0,privileged).observation;
assert.deepEqual(absent.slice(0,10),Array(10).fill(0),"missing sensors must zero-fill, never fall back to odometry");

const sensors={imu:{timestamp_s:1,valid:true,linear_accel_body:[1,2,3] as [number,number,number],angular_rate_body:[4,5,6] as [number,number,number],yaw_ned_rad:.7},gps:{timestamp_s:1,valid:true,position_ned_m:[11,22] as [number,number]}};
const sensed=bridge.sample(1,{...privileged,...sensors}).observation;
assert.deepEqual(sensed.slice(0,10),[1,2,3,4,5,6,.7,19,28,1]);
// Active topic-stall regression: keep presenting the last received messages
// while simulation time advances. They may be held inside their documented
// age windows, but must then disappear instead of being mistaken for fresh
// simply because a subscriber still has a message object.
const heldDuringStall=bridge.sample(2,{...privileged,time_s:1.04,...sensors}).observation;
assert.deepEqual(heldDuringStall.slice(0,10),sensed.slice(0,10));
const imuExpiredFirst=bridge.sample(3,{...privileged,time_s:1.2,...sensors}).observation;
assert.deepEqual(imuExpiredFirst.slice(0,7),Array(7).fill(0));
assert.deepEqual(imuExpiredFirst.slice(7,10),[19,28,1]);
const allExpired=bridge.sample(4,{...privileged,time_s:1.76,...sensors}).observation;
assert.deepEqual(allExpired.slice(0,10),Array(10).fill(0),"stalled sensor topics must zero-fill by simulation timestamp");
const changedTruth=bridge.sample(2,{...privileged,N_m:-100,E_m:100,yaw_rad:-2,u_mps:-9,v_mps:6,r_rad_s:-3,...sensors}).observation;
assert.deepEqual(changedTruth.slice(0,10),sensed.slice(0,10),"privileged odometry must not affect policy sensor fields");
const stale=bridge.sample(3,{...privileged,time_s:2,...sensors}).observation;
assert.deepEqual(stale.slice(0,10),Array(10).fill(0));
const gazebo=gazeboOdomToTask(1,{x:1,y:2,vx:3,vy:4,yaw_rad:.2,angular_z:.3});
assert.equal(gazebo.imu,undefined);assert.equal(gazebo.gps,undefined);
assert.equal(gazebo.N_m,2);assert.equal(gazebo.E_m,1);
assert.ok(Math.abs(gazebo.yaw_rad-(Math.PI/2-.2))<1e-12);
assert.ok(Math.abs(gazebo.r_rad_s+.3)<1e-12);
assert.ok(Math.abs(gazebo.u_mps-(Math.cos(gazebo.yaw_rad)*4+Math.sin(gazebo.yaw_rad)*3))<1e-12);
assert.ok(Math.abs(gazebo.v_mps-(-Math.sin(gazebo.yaw_rad)*4+Math.cos(gazebo.yaw_rad)*3))<1e-12);
console.log("External observation oracle exclusion passed for VRX and Gazebo adapters.");
