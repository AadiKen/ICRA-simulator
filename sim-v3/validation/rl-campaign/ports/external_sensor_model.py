"""Pure sensor conditioning shared by external runtime exporters."""
from collections import deque
import math
import random

EARTH_RADIUS_M = 6_378_137.0
WGS84_FLATTENING = 1 / 298.257223563

def gazebo_navsat_valid(latitude_deg, longitude_deg):
    """Gazebo NavSat has no status bit: finite geodetic coordinates are valid.

    Timestamp freshness remains a separate contract check at observation time,
    matching bcod-sim's distinction between sensor validity and staleness.
    """
    return all(isinstance(x,(int,float)) and math.isfinite(x) for x in (latitude_deg,longitude_deg))

class ExternalGpsModel:
    """Delayed 2 Hz position-only GPS; ground velocity is excluded."""
    def __init__(self, seed, initial_n_m, initial_e_m, rate_hz=2.0, latency_s=0.2, position_std_m=0.8,
                 reference_latitude_deg=None, reference_longitude_deg=None, reference_elevation_m=0.0):
        self.period_s=1.0/rate_hz; self.latency_s=latency_s; self.position_std_m=position_std_m
        self.initial_n_m=initial_n_m; self.initial_e_m=initial_e_m; self.random=random.Random(seed)
        anchored=reference_latitude_deg is not None and reference_longitude_deg is not None
        self.geodetic_anchored=anchored
        self.reference=(reference_latitude_deg,reference_longitude_deg) if anchored else None
        self.reference_elevation_m=reference_elevation_m
        self.anchor_n_m=0.0 if anchored else initial_n_m; self.anchor_e_m=0.0 if anchored else initial_e_m
        self.last_ingested_s=-math.inf; self.pending=deque(); self.held=None; self.last_diagnostic=None
    def ingest(self,timestamp_s,latitude_deg,longitude_deg,valid,altitude_m=0.0):
        if timestamp_s-self.last_ingested_s < self.period_s-1e-9:return
        self.last_ingested_s=timestamp_s
        if not valid:
            self.pending.append((timestamp_s+self.latency_s,{"timestamp_s":timestamp_s,"valid":False}));return
        if self.reference is None:self.reference=(latitude_deg,longitude_deg)
        lat0,lon0=self.reference
        if self.geodetic_anchored:
            def ecef(lat_deg,lon_deg,height):
                lat,lon=math.radians(lat_deg),math.radians(lon_deg);sin_lat=math.sin(lat)
                e2=WGS84_FLATTENING*(2-WGS84_FLATTENING);radius=EARTH_RADIUS_M/math.sqrt(1-e2*sin_lat*sin_lat)
                return ((radius+height)*math.cos(lat)*math.cos(lon),(radius+height)*math.cos(lat)*math.sin(lon),(radius*(1-e2)+height)*sin_lat)
            x,y,z=ecef(latitude_deg,longitude_deg,altitude_m);x0,y0,z0=ecef(lat0,lon0,self.reference_elevation_m)
            dx,dy,dz=x-x0,y-y0,z-z0;lat0_rad,lon0_rad=math.radians(lat0),math.radians(lon0)
            east=-math.sin(lon0_rad)*dx+math.cos(lon0_rad)*dy
            north=-math.sin(lat0_rad)*math.cos(lon0_rad)*dx-math.sin(lat0_rad)*math.sin(lon0_rad)*dy+math.cos(lat0_rad)*dz
        else:
            lat0_rad=math.radians(lat0);sin_lat=math.sin(lat0_rad);e2=WGS84_FLATTENING*(2-WGS84_FLATTENING)
            denominator=math.sqrt(1-e2*sin_lat*sin_lat);prime_vertical_radius=EARTH_RADIUS_M/denominator;meridional_radius=EARTH_RADIUS_M*(1-e2)/(denominator**3)
            north=self.initial_n_m+math.radians(latitude_deg-lat0)*meridional_radius
            east=self.initial_e_m+math.radians(longitude_deg-lon0)*prime_vertical_radius*math.cos(lat0_rad)
        base_n=north if self.geodetic_anchored else self.anchor_n_m+north-self.initial_n_m
        base_e=east if self.geodetic_anchored else self.anchor_e_m+east-self.initial_e_m
        noise_n=self.random.gauss(0,self.position_std_m);noise_e=self.random.gauss(0,self.position_std_m)
        position=(base_n+noise_n,base_e+noise_e)
        self.last_diagnostic={"timestamp_s":timestamp_s,"base_position_ned_m":[base_n,base_e],
                              "noise_ned_m":[noise_n,noise_e],"noise_magnitude_m":math.hypot(noise_n,noise_e)}
        self.pending.append((timestamp_s+self.latency_s,{"timestamp_s":timestamp_s,"valid":True,
                            "position_ned_m":list(position),"diagnostic":self.last_diagnostic}))
    def sample(self,now_s):
        while self.pending and self.pending[0][0]<=now_s+1e-12:_,self.held=self.pending.popleft()
        return self.held

def quaternion_to_ned_yaw(w,x,y,z):
    """Exact Python port of task-trace-bridge.ts enuQuaternionToNedYaw."""
    yaw_enu=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    wrapped=((math.pi/2-yaw_enu+math.pi)%(2*math.pi)+2*math.pi)%(2*math.pi)-math.pi
    return float(format(wrapped,'.15g'))

def flu_to_body_ned(x,y,z):return [x,-y,-z]

def quaternion_rotate_world_to_body(w,x,y,z,v):
    """Rotate a world-frame vector into body FLU for body-to-world quaternion q."""
    vx,vy,vz=v
    return [
        (1-2*(y*y+z*z))*vx+2*(x*y+w*z)*vy+2*(x*z-w*y)*vz,
        2*(x*y-w*z)*vx+(1-2*(x*x+z*z))*vy+2*(y*z+w*x)*vz,
        2*(x*z+w*y)*vx+2*(y*z-w*x)*vy+(1-2*(x*x+y*y))*vz]

def inertial_acceleration_from_gazebo_imu(w,x,y,z,ax,ay,az,gravity_mps2=9.81):
    """Remove the body-frame gravity reaction from Gazebo proper acceleration."""
    gx,gy,gz=quaternion_rotate_world_to_body(w,x,y,z,(0.,0.,-gravity_mps2))
    return [ax+gx,ay+gy,az+gz]
