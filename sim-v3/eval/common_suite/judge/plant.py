"""Standalone planar Fossen plant. This module imports no simulator engine."""
from __future__ import annotations
from dataclasses import dataclass,replace
from math import cos,exp,pi,sin
from typing import Sequence
import numpy as np

@dataclass(frozen=True)
class FossenParameters:
 mass_kg:float=52.3;yaw_inertia_kg_m2:float=17.891219833333338
 added_mass_kg:tuple[float,float,float]=(0.,0.,0.)
 linear_damping:tuple[float,float,float]=(6.,18.,8.)
 quadratic_damping:tuple[float,float,float]=(18.,60.,12.)
 thruster_y_m:tuple[float,float]=(-.33,.33);max_thrust_n:float=70.;actuator_tau_s:float=.35
 wind_force_coefficient:float=.35
 def perturbed(self,mass_scale=1.,drag_scale=1.,thrust_scale=1.):
  return replace(self,mass_kg=self.mass_kg*mass_scale,yaw_inertia_kg_m2=self.yaw_inertia_kg_m2*mass_scale,linear_damping=tuple(x*drag_scale for x in self.linear_damping),quadratic_damping=tuple(x*drag_scale for x in self.quadratic_damping),max_thrust_n=self.max_thrust_n*thrust_scale)

@dataclass
class PlantState:
 north_m:float=0.;east_m:float=0.;heading_rad:float=0.;surge_mps:float=0.;sway_mps:float=0.;yaw_rate_rad_s:float=0.;port_thrust_n:float=0.;starboard_thrust_n:float=0.;time_s:float=0.
 def vector(self):return np.array([self.north_m,self.east_m,self.heading_rad,self.surge_mps,self.sway_mps,self.yaw_rate_rad_s],float)

def wrap(angle:float)->float:return (angle+pi)%(2*pi)-pi

class AnalyticJudge:
 def __init__(self,parameters:FossenParameters|None=None,dt_s:float=.05):
  self.parameters=parameters or FossenParameters();self.dt_s=float(dt_s);self.state=PlantState();self.last_acceleration=np.zeros(3)
 def reset(self,state:PlantState|None=None,parameters:FossenParameters|None=None):self.state=state or PlantState();self.parameters=parameters or self.parameters;self.last_acceleration=np.zeros(3);return self.state
 def _derivative(self,y:np.ndarray,thrust:np.ndarray,current_ned:np.ndarray,wind_ned:np.ndarray)->np.ndarray:
  p=self.parameters;n,e,psi,u,v,r=y;c,s=cos(psi),sin(psi);current_body=np.array([c*current_ned[0]+s*current_ned[1],-s*current_ned[0]+c*current_ned[1],0.]);nu=np.array([u,v,r]);relative=nu-current_body
  mass=np.array([p.mass_kg+p.added_mass_kg[0],p.mass_kg+p.added_mass_kg[1],p.yaw_inertia_kg_m2+p.added_mass_kg[2]])
  coriolis=np.array([-p.mass_kg*v*r,p.mass_kg*u*r,0.]);damping=np.asarray(p.linear_damping)*relative+np.asarray(p.quadratic_damping)*np.abs(relative)*relative
  tau=np.array([thrust.sum(),0.,-p.thruster_y_m[0]*thrust[0]-p.thruster_y_m[1]*thrust[1]])
  wind_body=np.array([c*wind_ned[0]+s*wind_ned[1],-s*wind_ned[0]+c*wind_ned[1],0.]);tau+=p.wind_force_coefficient*wind_body*np.abs(wind_body)
  accel=(tau-coriolis-damping)/mass
  return np.array([c*u-s*v,s*u+c*v,r,*accel])
 def step(self,normalized_action:Sequence[float],current_ned=(0.,0.),wind_ned=(0.,0.))->PlantState:
  action=np.clip(np.asarray(normalized_action,float),-1,1);p=self.parameters;alpha=1. if p.actuator_tau_s==0 else 1-exp(-self.dt_s/p.actuator_tau_s);applied=np.array([self.state.port_thrust_n,self.state.starboard_thrust_n]);applied+=alpha*(action*p.max_thrust_n-applied)
  y=self.state.vector();cur=np.asarray(current_ned,float);wind=np.asarray(wind_ned,float);dt=self.dt_s
  k1=self._derivative(y,applied,cur,wind);k2=self._derivative(y+.5*dt*k1,applied,cur,wind);k3=self._derivative(y+.5*dt*k2,applied,cur,wind);k4=self._derivative(y+dt*k3,applied,cur,wind);next_y=y+dt*(k1+2*k2+2*k3+k4)/6;next_y[2]=wrap(float(next_y[2]));self.last_acceleration=k4[3:];self.state=PlantState(*map(float,next_y),*map(float,applied),self.state.time_s+dt);return self.state
 def observation(self,target:Sequence[float],previous_action:Sequence[float],remaining_fraction:float,fix_valid:float=1.)->np.ndarray:
  s=self.state;return np.asarray([self.last_acceleration[0],self.last_acceleration[1],0.,0.,0.,s.yaw_rate_rad_s,s.heading_rad,target[0]-s.north_m,target[1]-s.east_m,fix_valid,previous_action[0],previous_action[1],0.,0.,np.clip(remaining_fraction,0,1)],np.float32)
