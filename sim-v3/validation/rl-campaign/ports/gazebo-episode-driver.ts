/** Gazebo adapter endpoint.  captureGazeboLog owns launch, Transport and cleanup. */
import {assembleTrace,prepareEpisode,writeTrace,PHYSICS_DT} from "./episode-driver.ts";
import {gazeboOdomToTask} from "./task-trace-bridge.ts";

export {prepareEpisode};
export function gazeboTraceFromTrueOdometry(seed:number,rows:Array<{x:number;y:number;vx:number;vy:number;yaw_rad:number;angular_z:number;imu?:any;gps_fix_valid?:number}>,out:string){
 const odom=rows.map((r,step)=>gazeboOdomToTask(step*PHYSICS_DT,r,r.imu?{imu_linear_accel_body:r.imu.slice(0,3),imu_angular_rate_body:r.imu.slice(3,6),gps_fix_valid:r.gps_fix_valid}:undefined));
 const trace=assembleTrace("Gazebo Harmonic",seed,odom,rows.length);writeTrace(out,trace);return trace;
}
