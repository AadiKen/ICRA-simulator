export type NavigationState={north_m:number;east_m:number;heading_rad:number;surge_mps:number;yaw_rate_rad_s:number};
export type ControllerName="LOS-PID-v2"|"LOS-SPEEDCAP-v2";
export type LosGains={lookahead:number;kp:number;kd:number;speed:number};
export type WrenchLimits={surge_n:number;yaw_nm:number};
export const VEHICLE_WRENCH_LIMITS={
  "vehicle-a-otter":{surge_n:150,yaw_nm:100},
  // Vehicle C thrust-increase task: 2 * 1000 N and 2 * 1000 N * 0.81 m.
  "vehicle-c-azimuth":{surge_n:2000,yaw_nm:1620}
} as const satisfies Record<string,WrenchLimits>;
export const FROZEN_GAINS:Record<ControllerName,{lookahead:number;kp:number;kd:number;speed:number}>={
  "LOS-PID-v2":{lookahead:8,kp:100,kd:35,speed:1},
  "LOS-SPEEDCAP-v2":{lookahead:4,kp:100,kd:35,speed:1}
};
export const VEHICLE_C_CANDIDATE_GAINS:Record<string,LosGains>={
  "LOS-PID-v2-C-candidate-v1":{lookahead:4,kp:100,kd:35,speed:1}
};
const clamp=(value:number,min:number,max:number)=>Math.min(max,Math.max(min,value));
const wrap=(angle:number)=>Math.atan2(Math.sin(angle),Math.cos(angle));

/** Shared, simulator-neutral guidance and control implementation. Adapters only map its wrench. */
export function losAction(name:ControllerName,state:NavigationState,legStart:[number,number],goal:[number,number],limits:WrenchLimits=VEHICLE_WRENCH_LIMITS["vehicle-a-otter"]):[number,number]{
  return losActionWithGains(state,legStart,goal,FROZEN_GAINS[name],name==="LOS-SPEEDCAP-v2",limits);
}

export function losActionCandidate(candidateName:keyof typeof VEHICLE_C_CANDIDATE_GAINS,state:NavigationState,legStart:[number,number],goal:[number,number],limits:WrenchLimits=VEHICLE_WRENCH_LIMITS["vehicle-c-azimuth"]):[number,number]{
  return losActionWithGains(state,legStart,goal,VEHICLE_C_CANDIDATE_GAINS[candidateName],false,limits);
}

export function losDesiredHeading(state:NavigationState,legStart:[number,number],goal:[number,number],lookahead:number):{desired_rad:number;along_m:number;leg_length_m:number;mode:"line"|"point-capture"}{
  const dn=goal[0]-legStart[0],de=goal[1]-legStart[1],length=Math.hypot(dn,de),cn=dn/length,ce=de/length;
  const along=(state.north_m-legStart[0])*cn+(state.east_m-legStart[1])*ce;
  const cross=-ce*(state.north_m-legStart[0])+cn*(state.east_m-legStart[1]);
  const dNorth=goal[0]-state.north_m,dEast=goal[1]-state.east_m;
  return along>=length
    ? {desired_rad:Math.atan2(dEast,dNorth),along_m:along,leg_length_m:length,mode:"point-capture"}
    : {desired_rad:Math.atan2(ce,cn)-Math.atan2(cross,lookahead),along_m:along,leg_length_m:length,mode:"line"};
}

export function losActionWithGains(state:NavigationState,legStart:[number,number],goal:[number,number],gains:LosGains,speedCap=false,limits:WrenchLimits=VEHICLE_WRENCH_LIMITS["vehicle-a-otter"]):[number,number]{
  const {desired_rad:desired}=losDesiredHeading(state,legStart,goal,gains.lookahead);
  const dNorth=goal[0]-state.north_m,dEast=goal[1]-state.east_m,distance=Math.hypot(dNorth,dEast);
  const headingError=wrap(desired-state.heading_rad);
  let speed=gains.speed;
  if(speedCap) speed=Math.min(speed,Math.sqrt(Math.max(0,.8*distance)),speed*Math.max(.25,Math.cos(Math.min(Math.PI/2,Math.abs(headingError)))));
  // [surge force N, yaw moment Nm]. Gains are shared; the safety envelope is
  // selected for the vehicle that must realize the requested wrench.
  return [clamp(100*(speed-state.surge_mps),-limits.surge_n,limits.surge_n),clamp(gains.kp*headingError-gains.kd*state.yaw_rate_rad_s,-limits.yaw_nm,limits.yaw_nm)];
}

export function vehicleAWrenchToActuators([surge,yaw]:[number,number]):[number,number,number,number]{
  // Vehicle A's 4-vector reserves unused dimensions at zero; this is the common portable action contract.
  return [clamp((surge-yaw)/150,-1,1),clamp((surge+yaw)/150,-1,1),0,0];
}

/** Vehicle C deliberately has no guidance law here: it consumes losAction's wrench unchanged. */
export function vehicleCDesiredWrench([surge,yaw]:[number,number]):[number,number,number,number,number,number]{
  return [surge,0,0,0,0,yaw];
}
