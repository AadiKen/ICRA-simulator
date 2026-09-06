"""Pure sensor conditioning shared by external runtime exporters."""
from collections import deque
import math
import random

EARTH_RADIUS_M = 6_378_137.0

class ExternalGpsModel:
    """Delayed 2 Hz position-only GPS; ground velocity is excluded."""
    def __init__(self, seed, initial_n_m, initial_e_m, rate_hz=2.0, latency_s=0.2, position_std_m=0.8):
        self.period_s=1.0/rate_hz; self.latency_s=latency_s; self.position_std_m=position_std_m
        self.initial_n_m=initial_n_m; self.initial_e_m=initial_e_m; self.random=random.Random(seed)
        self.reference=None; self.last_ingested_s=-math.inf; self.pending=deque(); self.held=None
    def ingest(self,timestamp_s,latitude_deg,longitude_deg,valid):
        if timestamp_s-self.last_ingested_s < self.period_s-1e-9:return
        self.last_ingested_s=timestamp_s
        if not valid:
            self.pending.append((timestamp_s+self.latency_s,{"timestamp_s":timestamp_s,"valid":False}));return
        if self.reference is None:self.reference=(latitude_deg,longitude_deg)
        lat0,lon0=self.reference
        north=self.initial_n_m+math.radians(latitude_deg-lat0)*EARTH_RADIUS_M
        east=self.initial_e_m+math.radians(longitude_deg-lon0)*EARTH_RADIUS_M*math.cos(math.radians(lat0))
        position=(north+self.random.gauss(0,self.position_std_m),east+self.random.gauss(0,self.position_std_m))
        self.pending.append((timestamp_s+self.latency_s,{"timestamp_s":timestamp_s,"valid":True,"position_ned_m":list(position)}))
    def sample(self,now_s):
        while self.pending and self.pending[0][0]<=now_s+1e-12:_,self.held=self.pending.popleft()
        return self.held

def quaternion_to_ned_yaw(w,x,y,z):
    yaw_enu=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return (math.pi/2-yaw_enu+math.pi)%(2*math.pi)-math.pi

def flu_to_body_ned(x,y,z):return [x,-y,-z]
