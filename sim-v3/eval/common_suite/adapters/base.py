from __future__ import annotations
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence, runtime_checkable
import numpy as np

POLICY_OBSERVATION_FIELDS=("imu.linear_accel_x","imu.linear_accel_y","imu.linear_accel_z","imu.angular_rate_x","imu.angular_rate_y","imu.angular_rate_z","imu.yaw_ned_rad","gps.relative_goal_north_m","gps.relative_goal_east_m","gps.fix_valid","previous_action.effector_0","previous_action.effector_1","previous_action.steer_0","previous_action.steer_1","normalized_time_remaining")
SHARED_ACTION_FIELDS=("effector_0","effector_1")
class AdapterConfigurationError(ValueError):pass

@runtime_checkable
class PolicyAdapter(Protocol):
 arm:str;consumes_sensors:bool;force_ceiling_n:float;actuator_tau_s:float;native_observation_keys:tuple[str,...];native_action_keys:tuple[str,...]
 def load_policy(self,checkpoint_path:str|Path)->Any:...
 def native_obs_to_shared(self,native_obs:Mapping[str,Any]|Sequence[float])->np.ndarray:...
 def shared_obs_to_native(self,shared_obs:Sequence[float])->dict[str,float]:...
 def shared_action_to_native(self,action:Sequence[float])->dict[str,float]:...
 def native_action_to_shared(self,native_action:Mapping[str,float]|Sequence[float])->np.ndarray:...
 def policy_step(self,policy:Any,shared_obs:np.ndarray)->np.ndarray:...

class PortablePPOAdapter:
 """Translation only; the judge applies actuator lag exactly once."""
 arm="unknown";consumes_sensors=True;native_observation_keys=POLICY_OBSERVATION_FIELDS;native_action_keys=SHARED_ACTION_FIELDS
 def __init__(self,*,force_ceiling_n:float=70.,actuator_tau_s:float=.35,native_observation_keys:Sequence[str]|None=None,native_action_keys:Sequence[str]|None=None):
  if force_ceiling_n<=0 or actuator_tau_s<0:raise AdapterConfigurationError("force ceiling must be positive and actuator lag non-negative")
  self.force_ceiling_n=float(force_ceiling_n);self.actuator_tau_s=float(actuator_tau_s);self.native_observation_keys=tuple(native_observation_keys or self.native_observation_keys);self.native_action_keys=tuple(native_action_keys or self.native_action_keys)
  if len(self.native_observation_keys)!=15:raise AdapterConfigurationError(f"{self.arm}: exported policy requires exactly 15 observation fields")
  if len(self.native_action_keys)!=2:raise AdapterConfigurationError(f"{self.arm}: exported policy requires exactly two fixed-thruster actions")
 def load_policy(self,checkpoint_path:str|Path)->Any:
  path=Path(checkpoint_path)
  if not path.is_file():raise FileNotFoundError(path)
  from stable_baselines3 import PPO
  policy=PPO.load(str(path),device="cpu")
  if policy.observation_space.shape!=(15,) or policy.action_space.shape!=(2,):raise AdapterConfigurationError(f"{self.arm}: checkpoint is not a 15-observation/two-action portable PPO")
  return policy
 def native_obs_to_shared(self,native_obs:Mapping[str,Any]|Sequence[float])->np.ndarray:
  if isinstance(native_obs,Mapping):
   missing=[key for key in self.native_observation_keys if key not in native_obs]
   if missing:raise AdapterConfigurationError(f"{self.arm}: missing native observation fields: {missing}")
   values=[native_obs[key] for key in self.native_observation_keys]
  else:values=native_obs
  vector=np.asarray(values,dtype=np.float32)
  if vector.shape!=(15,) or not np.isfinite(vector).all():raise AdapterConfigurationError(f"{self.arm}: observation must be 15 finite values")
  return vector
 def shared_obs_to_native(self,shared_obs:Sequence[float])->dict[str,float]:
  vector=np.asarray(shared_obs,dtype=np.float32)
  if vector.shape!=(15,) or not np.isfinite(vector).all():raise AdapterConfigurationError("shared observation must be 15 finite values")
  return dict(zip(self.native_observation_keys,map(float,vector),strict=True))
 def shared_action_to_native(self,action:Sequence[float])->dict[str,float]:
  vector=np.asarray(action,dtype=np.float32)
  if vector.shape!=(2,) or not np.isfinite(vector).all():raise AdapterConfigurationError("shared action must be two finite normalized commands")
  return dict(zip(self.native_action_keys,map(float,np.clip(vector,-1,1)),strict=True))
 def native_action_to_shared(self,native_action:Mapping[str,float]|Sequence[float])->np.ndarray:
  if isinstance(native_action,Mapping):
   missing=[key for key in self.native_action_keys if key not in native_action]
   if missing:raise AdapterConfigurationError(f"{self.arm}: missing native action fields: {missing}")
   values=[native_action[key] for key in self.native_action_keys]
  else:values=native_action
  vector=np.asarray(values,dtype=np.float32)
  if vector.shape!=(2,) or not np.isfinite(vector).all():raise AdapterConfigurationError(f"{self.arm}: action must be two finite values")
  return np.clip(vector,-1,1)
 def policy_step(self,policy:Any,shared_obs:np.ndarray)->np.ndarray:
  action,_=policy.predict(self.native_obs_to_shared(self.shared_obs_to_native(shared_obs)),deterministic=True);return self.native_action_to_shared(action)
