from __future__ import annotations
import hashlib,json,math
from pathlib import Path
from typing import Any
import gymnasium as gym
import numpy as np
from .node_bridge import PersistentNodeBridge
from .common_task import classify_termination, compute_reward, cross_track_distance, passed_waypoint_plane
class Mulberry32:
 def __init__(self,seed:int):self.s=seed&0xffffffff
 def next(self):
  self.s=(self.s+0x6D2B79F5)&0xffffffff;z=self.s;z=((z^(z>>15))*(z|1))&0xffffffff;z=(z^(z+(((z^(z>>7))*(z|61))&0xffffffff)))&0xffffffff;return ((z^(z>>14))&0xffffffff)/4294967296
class CommonWaypointEnv(gym.Env):
 metadata={"render_modes":[]}
 # A sticky bridge sample is usable only through one complete nominal sample
 # period, its configured latency, and one physics tick of scheduling slack.
 # Missing, invalid, or older samples are zero-filled; GPS fix_valid remains 0.
 SENSOR_MAX_AGE_S={"imu":1/50+.01+.05,"gps":1/2+.2+.05}
 # Frozen by the approved common-waypoint-transit-v1 terminal revision.  The
 # terminal condition is intentionally pass-through; station keeping remains
 # part of the non-portable showcase task.
 EXPECTED_CONTRACT_SHA256="2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36"
 def __init__(self,repository:str|Path,base_seed:int=0,fixed_reset_seed:int|None=None,final_leg_curriculum:bool=False,shaping_enabled:bool=True,bridge=None,backend_type:str="node"):
  self.root=Path(repository);contract=json.loads((self.root/"artifacts/rl-campaign/surveyor/task-contract-frozen.json").read_text());assert contract["content_sha256"]==self.EXPECTED_CONTRACT_SHA256;common=next(task for task in contract["tasks"] if task["task_id"]=="common-waypoint-transit-v1");self.contract=common;self.final_radius_m=float(common["learnability"]["absolute_success_rate_threshold"]["terminal_definition"]["radius_m"]);self.terminal_variant=common["learnability"]["absolute_success_rate_threshold"]["terminal_definition"]["variant"];timing=common["timing"];self.timeout_steps=int(timing["episode_length_steps"]);self.physics_timestep_s=float(timing["physics_timestep_s"]);self.control_interval_s=float(timing["control_interval_s"]);ratio=self.control_interval_s/self.physics_timestep_s;self.physics_steps_per_action=round(ratio)
  if self.physics_steps_per_action<1 or not math.isclose(ratio,self.physics_steps_per_action,rel_tol=0,abs_tol=1e-12):raise ValueError("control_interval_s must be an integer multiple of physics_timestep_s")
  self.max_control_steps=math.ceil(self.timeout_steps/self.physics_steps_per_action);self.route_relative=common["reset_randomization"]["route_relative_m"];self.wind_range=common["reset_randomization"]["wind_speed_m_s"];self.current_range=common["reset_randomization"]["current_speed_m_s"];self.vehicle_preset=common["vehicle"]["preset"];shaping=common["reward"]["potential_shaping"];self.shaping_k=float(shaping["k"]);self.shaping_gamma=float(shaping["gamma"]);self.shaping_enabled=shaping_enabled;self.final_leg_curriculum=final_leg_curriculum;self.base_seed=base_seed;self.fixed_reset_seed=fixed_reset_seed;self.episode=0;self.backend_type=backend_type;self.bridge=bridge if bridge is not None else PersistentNodeBridge(self.root);self.action_space=gym.spaces.Box(-1,1,(2,),np.float32);self.observation_space=gym.spaces.Box(-np.inf,np.inf,(15,),np.float32);self.route=[];self.waypoint=0;self.prev_distance=0.;self.prev_action=np.zeros(2);self.steps=0;self.control_steps=0;self.hold=0;self.cross_track_sum=0.;self.last_truth=None;self.last_observation={"time_s":0.,"sensors":{}}
 def _randomization(self,seed):
  r=Mulberry32(seed);u=lambda a,b:a+(b-a)*r.next();angle=math.radians(u(-20,20));origin=[10000+u(-1,1),10000+u(-1,1)];rot=lambda p:[origin[0]+p[0]*math.cos(angle)-p[1]*math.sin(angle),origin[1]+p[0]*math.sin(angle)+p[1]*math.cos(angle)];speed=u(*self.current_range);direction=u(0,2*math.pi);wind=u(*self.wind_range);wd=u(0,2*math.pi);heading_error=math.radians(u(-10,10));route=[rot(p) for p in self.route_relative]
  if self.final_leg_curriculum:
   start=list(route[1]);heading=math.atan2(route[2][1]-route[1][1],route[2][0]-route[1][0])+heading_error
  else:start=origin;heading=heading_error
  return start,heading,route,[speed*math.cos(direction),speed*math.sin(direction),0],[wind*math.cos(wd),wind*math.sin(wd),0]
 def _config(self,seed):
  start,heading,self.route,current,wind=self._randomization(seed);self.start=start
  return{"schema_version":1,"experiment":{"name":f"p3-v3-surveyor-{seed}","seed":seed,"timestep_s":self.physics_timestep_s,"duration_s":120},"backend":{"type":self.backend_type},"vehicle":{"preset":self.vehicle_preset,"plant":"planar3"},"environment":{"current_mps":current,"wind_mps":wind},"initial_state":{"position_ned_m":[*start,0],"attitude_rad":[0,0,heading]},"mission":{"type":"rl-common-waypoint-v1","waypoints":[{"north_m":19000,"east_m":19000}]},"sensors":[{"plugin":"imu","enabled":True},{"plugin":"gps","enabled":True}]}
 def reset(self,*,seed=None,options=None):
  super().reset(seed=seed);actual=self.fixed_reset_seed if self.fixed_reset_seed is not None else self.base_seed+self.episode;self.episode+=1;reset_result=self.bridge.reset([self._config(actual)]);self.last_observation=reset_result["observations"][0];self.waypoint=2 if self.final_leg_curriculum else 0;self.prev_action[:]=0;self.steps=0;self.control_steps=0;self.hold=0;self.cross_track_sum=0;self.last_truth=self.bridge.ground_truth();self.prev_distance=self._distance();return self._obs(),{"seed":actual,"curriculum_phase":"final-leg-isolation" if self.final_leg_curriculum else "full-episode","control_interval_s":self.control_interval_s,"physics_steps_per_action":self.physics_steps_per_action}
 def _distance(self):return math.hypot(self.route[self.waypoint][0]-self.last_truth["position_ned_m"][0],self.route[self.waypoint][1]-self.last_truth["position_ned_m"][1])
 def _fresh_sensor(self,name):
  observation=self.last_observation if isinstance(self.last_observation,dict) else {};sample=observation.get("sensors",{}).get(name);now=observation.get("time_s",self.steps*self.physics_timestep_s)
  if not isinstance(sample,dict) or not sample.get("valid") or sample.get("payload") is None:return None
  timestamp=sample.get("timestampS")
  if not isinstance(timestamp,(int,float)) or not isinstance(now,(int,float)):return None
  age=float(now)-float(timestamp)
  return sample["payload"] if age>=-1e-12 and age<=self.SENSOR_MAX_AGE_S[name]+1e-12 else None
 def _obs(self):
  target=self.route[self.waypoint];imu_payload=self._fresh_sensor("imu");gps_payload=self._fresh_sensor("gps");imu=[0.]*7;gps=[0.]*3
  if imu_payload is not None:imu=[*imu_payload["acceleration_body_mps2"],*imu_payload["angular_rate_body_rad_s"],imu_payload["orientation_rad"][2]]
  if gps_payload is not None:
   n,e=gps_payload["position_ned_m"][:2];gps=[target[0]-n,target[1]-e,1.]
  time_remaining=max(0.,(self.timeout_steps-self.steps)/self.timeout_steps);return np.asarray([*imu,*gps,self.prev_action[0],self.prev_action[1],0,0,time_remaining],np.float32)
 def _cross_track(self):
  p=self.last_truth["position_ned_m"][:2];a=self.route[self.waypoint-1] if self.waypoint else self.start;b=self.route[self.waypoint];return cross_track_distance(p,a,b)
 def _passed_waypoint_plane(self):
  p=self.last_truth["position_ned_m"][:2];a=self.route[self.waypoint-1] if self.waypoint else self.start;b=self.route[self.waypoint];return passed_waypoint_plane(p,a,b)
 def step(self,action):
  previous_final_distance=math.hypot(self.route[-1][0]-self.last_truth["position_ned_m"][0],self.route[-1][1]-self.last_truth["position_ned_m"][1]);a=np.clip(np.asarray(action,float),-1,1);command={"active_sensors":["imu","gps"],"actuators":{"effectors":{"port":{"command":float(a[0])},"starboard":{"command":float(a[1])}}}};result=None
  for _ in range(self.physics_steps_per_action):
   result=self.bridge.step([command]);self.last_observation=result["observations"][0];self.last_truth=self.bridge.ground_truth();self.steps+=1
   if result["terminated"][0] or result["truncated"][0] or self.steps>=self.timeout_steps:break
  assert result is not None
  distance=self._distance();track=self._cross_track();self.cross_track_sum+=track;previous_distance=self.prev_distance;previous_action=self.prev_action.copy();self.prev_distance=distance;self.prev_action=a.copy();self.control_steps+=1
  if self.waypoint<len(self.route)-1 and (distance<=6 or self._passed_waypoint_plane()):self.waypoint+=1;self.prev_distance=self._distance()
  speed=math.hypot(*self.last_truth["velocity_body_mps"][:2]);final_leg_active=self.waypoint==len(self.route)-1;final_distance=math.hypot(self.route[-1][0]-self.last_truth["position_ned_m"][0],self.route[-1][1]-self.last_truth["position_ned_m"][1]);success=final_leg_active and final_distance<=self.final_radius_m;timed_out=self.steps>=self.timeout_steps or bool(result["truncated"][0]);stop_reason=str(result["infos"][0].get("stop_reason","simulator_termination")) if result["terminated"][0] else None;reason=classify_termination(success=success,collision_type=stop_reason if stop_reason in ("grounding","object_collision") else None,allocation_failed=stop_reason=="allocation_failure",unstable=stop_reason=="instability",simulator_terminated=bool(result["terminated"][0]),timed_out=timed_out);terminated=reason not in ("running","timeout");truncated=reason=="timeout";scored=compute_reward(previous_distance,distance,track,a,previous_action,reason,previous_final_distance_m=previous_final_distance,final_distance_m=final_distance,shaping_k=self.shaping_k,shaping_gamma=self.shaping_gamma,shaping_enabled=self.shaping_enabled);reward=scored.reward;info={"success":success,"termination_reason":reason,"waypoints_reached":self.waypoint+(1 if success else 0),"current_waypoint_index":self.waypoint,"final_leg_active":final_leg_active,"distance_to_current_waypoint_m":distance,"distance_to_final_waypoint_m":final_distance,"speed_mps":speed,"terminal_definition":{"radius_m":self.final_radius_m,"variant":self.terminal_variant,"timeout_s":120},"reward_components":scored.components(),"mean_cross_track_m":self.cross_track_sum/self.control_steps,"physics_steps":self.steps,"control_steps":self.control_steps,"control_interval_s":self.control_interval_s,"terminal_state":self.last_truth};return self._obs(),reward,terminated,truncated,info
 def close(self):self.bridge.close()
