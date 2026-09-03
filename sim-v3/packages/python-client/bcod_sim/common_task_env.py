from __future__ import annotations
import hashlib,json,math
from pathlib import Path
from typing import Any
import gymnasium as gym
import numpy as np
from .node_bridge import PersistentNodeBridge
class Mulberry32:
 def __init__(self,seed:int):self.s=seed&0xffffffff
 def next(self):
  self.s=(self.s+0x6D2B79F5)&0xffffffff;z=self.s;z=((z^(z>>15))*(z|1))&0xffffffff;z=(z^(z+(((z^(z>>7))*(z|61))&0xffffffff)))&0xffffffff;return ((z^(z>>14))&0xffffffff)/4294967296
class CommonWaypointEnv(gym.Env):
 metadata={"render_modes":[]}
 # Frozen by the approved common-waypoint-transit-v1 terminal revision.  The
 # terminal condition is intentionally pass-through; station keeping remains
 # part of the non-portable showcase task.
 EXPECTED_CONTRACT_SHA256="905347f3e894841aa7e6613dfc286e585a0bf284f5f21820600cc09c0e6358ba"
 def __init__(self,repository:str|Path,base_seed:int=0,fixed_reset_seed:int|None=None,final_leg_curriculum:bool=False,shaping_enabled:bool=True):
  self.root=Path(repository);contract=json.loads((self.root/"artifacts/rl-campaign/surveyor/task-contract-frozen.json").read_text());assert contract["content_sha256"]==self.EXPECTED_CONTRACT_SHA256;common=next(task for task in contract["tasks"] if task["task_id"]=="common-waypoint-transit-v1");self.contract=common;self.final_radius_m=float(common["learnability"]["absolute_success_rate_threshold"]["terminal_definition"]["radius_m"]);self.terminal_variant=common["learnability"]["absolute_success_rate_threshold"]["terminal_definition"]["variant"];timing=common["timing"];self.timeout_steps=int(timing["episode_length_steps"]);self.physics_timestep_s=float(timing["physics_timestep_s"]);self.control_interval_s=float(timing["control_interval_s"]);ratio=self.control_interval_s/self.physics_timestep_s;self.physics_steps_per_action=round(ratio)
  if self.physics_steps_per_action<1 or not math.isclose(ratio,self.physics_steps_per_action,rel_tol=0,abs_tol=1e-12):raise ValueError("control_interval_s must be an integer multiple of physics_timestep_s")
  self.max_control_steps=math.ceil(self.timeout_steps/self.physics_steps_per_action);self.route_relative=common["reset_randomization"]["route_relative_m"];self.wind_range=common["reset_randomization"]["wind_speed_m_s"];self.current_range=common["reset_randomization"]["current_speed_m_s"];self.vehicle_preset=common["vehicle"]["preset"];shaping=common["reward"]["potential_shaping"];self.shaping_k=float(shaping["k"]);self.shaping_gamma=float(shaping["gamma"]);self.shaping_enabled=shaping_enabled;self.final_leg_curriculum=final_leg_curriculum;self.base_seed=base_seed;self.fixed_reset_seed=fixed_reset_seed;self.episode=0;self.bridge=PersistentNodeBridge(self.root);self.action_space=gym.spaces.Box(-1,1,(2,),np.float32);self.observation_space=gym.spaces.Box(-np.inf,np.inf,(16,),np.float32);self.route=[];self.waypoint=0;self.prev_distance=0.;self.prev_action=np.zeros(2);self.steps=0;self.control_steps=0;self.hold=0;self.cross_track_sum=0.;self.last_truth=None
 def _randomization(self,seed):
  r=Mulberry32(seed);u=lambda a,b:a+(b-a)*r.next();angle=math.radians(u(-20,20));origin=[10000+u(-1,1),10000+u(-1,1)];rot=lambda p:[origin[0]+p[0]*math.cos(angle)-p[1]*math.sin(angle),origin[1]+p[0]*math.sin(angle)+p[1]*math.cos(angle)];speed=u(*self.current_range);direction=u(0,2*math.pi);wind=u(*self.wind_range);wd=u(0,2*math.pi);heading_error=math.radians(u(-10,10));route=[rot(p) for p in self.route_relative]
  if self.final_leg_curriculum:
   start=list(route[1]);heading=math.atan2(route[2][1]-route[1][1],route[2][0]-route[1][0])+heading_error
  else:start=origin;heading=heading_error
  return start,heading,route,[speed*math.cos(direction),speed*math.sin(direction),0],[wind*math.cos(wd),wind*math.sin(wd),0]
 def _config(self,seed):
  start,heading,self.route,current,wind=self._randomization(seed);self.start=start
  return{"schema_version":1,"experiment":{"name":f"p3-v3-surveyor-{seed}","seed":seed,"timestep_s":self.physics_timestep_s,"duration_s":120},"backend":{"type":"node"},"vehicle":{"preset":self.vehicle_preset,"plant":"planar3"},"environment":{"current_mps":current,"wind_mps":wind},"initial_state":{"position_ned_m":[*start,0],"attitude_rad":[0,0,heading]},"mission":{"type":"rl-common-waypoint-v1","waypoints":[{"north_m":19000,"east_m":19000}]},"sensors":[]}
 def reset(self,*,seed=None,options=None):
  super().reset(seed=seed);actual=self.fixed_reset_seed if self.fixed_reset_seed is not None else self.base_seed+self.episode;self.episode+=1;self.bridge.reset([self._config(actual)]);self.waypoint=2 if self.final_leg_curriculum else 0;self.prev_action[:]=0;self.steps=0;self.control_steps=0;self.hold=0;self.cross_track_sum=0;self.last_truth=self.bridge.ground_truth();self.prev_distance=self._distance();return self._obs(),{"seed":actual,"curriculum_phase":"final-leg-isolation" if self.final_leg_curriculum else "full-episode","control_interval_s":self.control_interval_s,"physics_steps_per_action":self.physics_steps_per_action}
 def _distance(self):return math.hypot(self.route[self.waypoint][0]-self.last_truth["position_ned_m"][0],self.route[self.waypoint][1]-self.last_truth["position_ned_m"][1])
 def _obs(self):
  t=self.last_truth;n,e=t["position_ned_m"][:2];target=self.route[self.waypoint];yaw=t["attitude_rad"][2];u,v=t["velocity_body_mps"][:2];vn=math.cos(yaw)*u-math.sin(yaw)*v;ve=math.sin(yaw)*u+math.cos(yaw)*v;imu=[*t["acceleration_body_mps2"],*t["angular_rate_body_rad_s"]];gps=[target[0]-n,target[1]-e,vn,ve,1.];time_remaining=max(0.,(self.timeout_steps-self.steps)/self.timeout_steps);return np.asarray([*imu,*gps,self.prev_action[0],self.prev_action[1],0,0,time_remaining],np.float32)
 def _cross_track(self):
  p=self.last_truth["position_ned_m"][:2];a=self.route[self.waypoint-1] if self.waypoint else self.start;b=self.route[self.waypoint];dx,dy=b[0]-a[0],b[1]-a[1];q=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)));return math.hypot(p[0]-(a[0]+q*dx),p[1]-(a[1]+q*dy))
 def _passed_waypoint_plane(self):
  p=self.last_truth["position_ned_m"][:2];a=self.route[self.waypoint-1] if self.waypoint else self.start;b=self.route[self.waypoint];dx,dy=b[0]-a[0],b[1]-a[1];den=dx*dx+dy*dy;cross=abs(-dy*(p[0]-a[0])+dx*(p[1]-a[1]))/math.sqrt(den);return (p[0]-a[0])*dx+(p[1]-a[1])*dy>=den and cross<=15.361124064575238
 def step(self,action):
  previous_final_distance=math.hypot(self.route[-1][0]-self.last_truth["position_ned_m"][0],self.route[-1][1]-self.last_truth["position_ned_m"][1]);a=np.clip(np.asarray(action,float),-1,1);command={"actuators":{"effectors":{"port":{"command":float(a[0])},"starboard":{"command":float(a[1])}}}};result=None
  for _ in range(self.physics_steps_per_action):
   result=self.bridge.step([command]);self.last_truth=self.bridge.ground_truth();self.steps+=1
   if result["terminated"][0] or result["truncated"][0] or self.steps>=self.timeout_steps:break
  assert result is not None
  distance=self._distance();progress=self.prev_distance-distance;track=self._cross_track();progress_reward=2*progress;cross_track_penalty=-.02*track;action_delta_penalty=-.05*float(np.sum((a-self.prev_action)**2));base_reward=progress_reward+cross_track_penalty+action_delta_penalty;self.cross_track_sum+=track;self.prev_distance=distance;self.prev_action=a.copy();self.control_steps+=1
  if self.waypoint<len(self.route)-1 and (distance<=6 or self._passed_waypoint_plane()):self.waypoint+=1;self.prev_distance=self._distance()
  speed=math.hypot(*self.last_truth["velocity_body_mps"][:2]);final_leg_active=self.waypoint==len(self.route)-1;final_distance=math.hypot(self.route[-1][0]-self.last_truth["position_ned_m"][0],self.route[-1][1]-self.last_truth["position_ned_m"][1]);success=final_leg_active and final_distance<=self.final_radius_m;terminated=success or bool(result["terminated"][0]);truncated=self.steps>=self.timeout_steps or bool(result["truncated"][0]);reason="success" if success else str(result["infos"][0].get("stop_reason","simulator_termination")) if result["terminated"][0] else "timeout" if truncated else "running";terminal_reward=100 if success else -100 if result["terminated"][0] else -10 if truncated else 0;base_reward+=terminal_reward;phi_previous=-self.shaping_k*previous_final_distance;phi_next=0. if terminated or truncated else -self.shaping_k*final_distance;shaping_reward=self.shaping_gamma*phi_next-phi_previous if self.shaping_enabled else 0.;reward=base_reward+shaping_reward;info={"success":success,"termination_reason":reason,"waypoints_reached":self.waypoint+(1 if success else 0),"current_waypoint_index":self.waypoint,"final_leg_active":final_leg_active,"distance_to_current_waypoint_m":distance,"distance_to_final_waypoint_m":final_distance,"speed_mps":speed,"terminal_definition":{"radius_m":self.final_radius_m,"variant":self.terminal_variant,"timeout_s":120},"reward_components":{"progress":progress_reward,"cross_track":cross_track_penalty,"action_delta":action_delta_penalty,"terminal":terminal_reward,"base_reward":base_reward,"potential_shaping":shaping_reward,"shaped_reward":reward},"mean_cross_track_m":self.cross_track_sum/self.control_steps,"physics_steps":self.steps,"control_steps":self.control_steps,"control_interval_s":self.control_interval_s,"terminal_state":self.last_truth};return self._obs(),reward,terminated,truncated,info
 def close(self):self.bridge.close()
