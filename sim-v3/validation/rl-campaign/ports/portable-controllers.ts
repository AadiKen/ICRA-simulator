export type NavigationState={north_m:number;east_m:number;heading_rad:number;surge_mps:number;yaw_rate_rad_s:number};
export type ControllerName="LOS-PID-v2"|"LOS-SPEEDCAP-v2";
export const FROZEN_GAINS:Record<ControllerName,{lookahead:number;kp:number;kd:number;speed:number}>={
  "LOS-PID-v2":{lookahead:8,kp:100,kd:35,speed:1},
  "LOS-SPEEDCAP-v2":{lookahead:4,kp:100,kd:35,speed:1}
};
const clamp=(value:number,min:number,max:number)=>Math.min(max,Math.max(min,value));
const wrap=(angle:number)=>Math.atan2(Math.sin(angle),Math.cos(angle));

/** Shared, simulator-neutral guidance and control implementation. Adapters only map its wrench. */
export function losAction(name:ControllerName,state:NavigationState,legStart:[number,number],goal:[number,number]):[number,number]{
  const gains=FROZEN_GAINS[name],dn=goal[0]-legStart[0],de=goal[1]-legStart[1],length=Math.hypot(dn,de),cn=dn/length,ce=de/length;
  const along=(state.north_m-legStart[0])*cn+(state.east_m-legStart[1])*ce;
  const cross=-ce*(state.north_m-legStart[0])+cn*(state.east_m-legStart[1]);
  const distance=Math.hypot(goal[0]-state.north_m,goal[1]-state.east_m);
  const desired=Math.atan2(ce,cn)-Math.atan2(cross,gains.lookahead);
  const headingError=wrap(desired-state.heading_rad);
  let speed=gains.speed;
  if(name==="LOS-SPEEDCAP-v2") speed=Math.min(speed,Math.sqrt(Math.max(0,.8*distance)),speed*Math.max(.25,Math.cos(Math.min(Math.PI/2,Math.abs(headingError)))));
  // [surge force N, yaw moment Nm], exactly as the frozen calibration implementation.
  return [clamp(100*(speed-state.surge_mps),-150,150),clamp(gains.kp*headingError-gains.kd*state.yaw_rate_rad_s,-100,100)];
}

export function vehicleAWrenchToActuators([surge,yaw]:[number,number]):[number,number,number,number]{
  // Vehicle A's 4-vector reserves unused dimensions at zero; this is the common portable action contract.
  return [clamp((surge-yaw)/150,-1,1),clamp((surge+yaw)/150,-1,1),0,0];
}
