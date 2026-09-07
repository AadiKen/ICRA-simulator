#!/usr/bin/env python3
"""Persistent Gazebo Harmonic JSONL runtime for ``GazeboGymEnv``.

Gazebo stays paused and advances one 0.05 s physics iteration per ``step``.
All synchronization uses world ``/clock`` simulation timestamps.  GPS
conditioning and frame conversions are imported from the existing shared
ports; this file contains no alternative sensor or ENU/NED implementation.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

from external_sensor_model import ExternalGpsModel, flu_to_body_ned, gazebo_navsat_valid, quaternion_to_ned_yaw

ROOT=Path(__file__).resolve().parents[3]
IMAGE=os.environ.get("BCOD_GAZEBO_IMAGE","174e8baad590")
PHYSICS_DT_S=.005
CONTRACT_TICK_S=.05
PHYSICS_STEPS_PER_TICK=round(CONTRACT_TICK_S/PHYSICS_DT_S)

def stamp(value):
    value=value or {}; return int(value.get("sec",0))+int(value.get("nsec",0))*1e-9

class JsonTopic:
    def __init__(self,container,topic):
        self.topic=topic; self.latest=None; self.items=queue.Queue(); self.stalled=False; self.accepted=[]
        self.process=subprocess.Popen(["docker","exec",container,"gz","topic","-e","-t",topic,"--json-output"],text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=1)
        self.thread=threading.Thread(target=self._read,daemon=True); self.thread.start()
    def _read(self):
        assert self.process.stdout
        for line in self.process.stdout:
            try: value=json.loads(line)
            except json.JSONDecodeError: continue
            if not self.stalled: self.latest=value; self.items.put(value); self.accepted.append(value)
    def wait_at_least(self,target,extract,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=self.latest
            if value is not None and extract(value)>=target-1e-9:return value
            try:self.items.get(timeout=min(.1,max(0,deadline-time.monotonic())))
            except queue.Empty:pass
        raise TimeoutError(f"{self.topic} did not reach simulation time {target}")
    def wait_after(self,previous,extract,timeout=20):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=self.latest
            if value is not None and extract(value)>previous+1e-9:return value
            try:self.items.get(timeout=min(.1,max(0,deadline-time.monotonic())))
            except queue.Empty:pass
        raise TimeoutError(f"{self.topic} did not publish a sample newer than {previous}")
    def close(self):
        if self.process.poll() is None:self.process.terminate()
        try:self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:self.process.kill()

class Runtime:
    def __init__(self):
        self.container=f"icra27-gazebo-gym-{os.getpid()}"; self.temp=None; self.world=None
        self.topics={}; self.gps_model=None; self.actuator=None; self.converter=None
        self.truth=None; self.sim_time=0.; self.last_gps_seq=None
        self.idle_mode=os.environ.get("BCOD_GAZEBO_IDLE_MODE")=="1"
    def run(self,args,check=True,timeout=20):
        return subprocess.run(args,text=True,capture_output=True,check=check,timeout=timeout)
    def dexec(self,*args,**kwargs):return self.run(["docker","exec",self.container,*args],**kwargs)
    def service(self,name,reqtype,request):
        result=self.dexec("gz","service","-s",name,"--reqtype",reqtype,"--reptype","gz.msgs.Boolean","--timeout","10000","--req",request)
        if "data: true" not in result.stdout:raise RuntimeError(f"service {name} failed: {result.stdout} {result.stderr}")
    def _stop(self):
        for topic in self.topics.values():topic.close()
        self.topics={}
        for proc in (self.actuator,self.converter):
            if proc and proc.poll() is None:proc.terminate()
        self.run(["docker","rm","-f",self.container],check=False)
        if self.temp:self.temp.cleanup();self.temp=None
    def _wait_service(self):
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            out=self.dexec("gz","service","-l",check=False,timeout=5)
            if self.world+"/control" in out.stdout:return
            time.sleep(.1)
        raise TimeoutError("Gazebo world control service did not appear")
    def _node_request(self,proc,payload):
        assert proc.stdin and proc.stdout
        proc.stdin.write(json.dumps(payload,separators=(",",":"))+"\n");proc.stdin.flush()
        response=json.loads(proc.stdout.readline())
        if not response.get("ok"):raise RuntimeError(response.get("error","node helper failed"))
        return response
    def reset(self,config):
        self._stop(); self.temp=tempfile.TemporaryDirectory(prefix="bcod-gazebo-runtime-")
        seed=int(config["experiment"]["seed"]); out=self.temp.name
        prepare=["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/prepare-gazebo-episode.ts"),str(seed),out]
        if self.idle_mode:prepare.append("--idle-no-sensors")
        self.run(prepare)
        self.world=f"/world/bcod_parity_gate-{seed}"
        self.run(["docker","run","-d","--rm","--platform","linux/amd64","--name",self.container,"-e","GZ_SIM_RESOURCE_PATH=/gate/models","-v",f"{out}:/gate",IMAGE,"gz","sim","-s","-r",f"/gate/worlds/gate-{seed}.sdf"])
        self._wait_service()
        # Every episode gets a fresh container and a world generated with the
        # seeded pose. Do not issue reset-all here: Harmonic removes the
        # Buoyancy system's enabled-entity components during that redundant
        # reset, leaving the reconstructed model in gravity-only free fall.
        self.service(self.world+"/control","gz.msgs.WorldControl","pause: true")
        initial=config["initial_state"]["position_ned_m"]
        wanted={"clock":self.world+"/clock","odom":"/odometry"}
        if not self.idle_mode:wanted.update({"imu":"/imu","gps":"/gps"})
        self.topics={name:JsonTopic(self.container,topic) for name,topic in wanted.items()}
        self.actuator=subprocess.Popen(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/actuator-jsonl-bridge.ts")],cwd=ROOT,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self.converter=subprocess.Popen(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/task-trace-jsonl-bridge.ts")],cwd=ROOT,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self._node_request(self.actuator,{"op":"reset"})
        self.gps_model=ExternalGpsModel(seed,initial[0],initial[1]);self.last_gps_seq=None
        # Advance one complete contract tick using the same 10 ms internal
        # resolution as the accepted B/C plants. Harmonic's multi_step N
        # advances N+1 iterations while remaining paused.
        self.service(self.world+"/control","gz.msgs.WorldControl",f"pause: true multi_step: {PHYSICS_STEPS_PER_TICK-1}")
        self.topics["clock"].wait_at_least(CONTRACT_TICK_S,self._clock_time);self._wait_clock_stable();self.sim_time=self._clock_time(self.topics["clock"].latest or {})
        self.topics["odom"].wait_at_least(self.sim_time-.01,self._header_time)
        return self.response()
    def _clock_time(self,msg):return stamp(msg.get("sim"))
    def _header_time(self,msg):return stamp(msg.get("header",{}).get("stamp"))
    def _wait_clock_stable(self,stable_s=.2,timeout=5):
        deadline=time.monotonic()+timeout;previous=None;stable_since=None
        while time.monotonic()<deadline:
            current=self._clock_time(self.topics["clock"].latest or {})
            if previous is not None and abs(current-previous)<1e-12:
                stable_since=stable_since or time.monotonic()
                if time.monotonic()-stable_since>=stable_s:return current
            else:stable_since=None
            previous=current;time.sleep(.01)
        raise TimeoutError("/clock did not stabilize while paused")
    def _advance_confirmed(self,iterations):
        """Batch internal iterations and wait for the paused clock to settle."""
        start=self._clock_time(self.topics["clock"].latest or {})
        target=start+iterations*PHYSICS_DT_S
        self.service(self.world+"/control","gz.msgs.WorldControl",f"pause: true multi_step: {iterations}")
        self.topics["clock"].wait_at_least(target,self._clock_time)
        end=self._wait_clock_stable()
        if not math.isclose(end-start,iterations*PHYSICS_DT_S,rel_tol=0,abs_tol=1e-9):
            raise RuntimeError(f"Gazebo advanced {end-start}s; expected {iterations*PHYSICS_DT_S}s")
    def _ingest_gps(self):
        if "gps" not in self.topics:return
        msg=self.topics["gps"].latest
        if not msg:return
        seq=next((v.get("value",[None])[0] for v in msg.get("header",{}).get("data",[]) if v.get("key")=="seq"),None)
        if seq==self.last_gps_seq:return
        self.last_gps_seq=seq; ts=self._header_time(msg)
        lat=msg.get("latitudeDeg");lon=msg.get("longitudeDeg")
        valid=gazebo_navsat_valid(lat,lon)
        self.gps_model.ingest(ts,float(lat or 0),float(lon or 0),valid)
    def observation(self):
        self._ingest_gps(); sensors={}
        imu=self.topics.get("imu").latest if "imu" in self.topics else None
        if imu:
            q=imu.get("orientation",{});a=imu.get("linearAcceleration",{});w=imu.get("angularVelocity",{})
            valid=all(isinstance(x,(int,float)) and math.isfinite(x) for x in [q.get("w"),q.get("x"),q.get("y"),q.get("z"),a.get("x"),a.get("y"),a.get("z"),w.get("x"),w.get("y"),w.get("z")])
            payload=None if not valid else {"acceleration_body_mps2":flu_to_body_ned(a["x"],a["y"],a["z"]),"angular_rate_body_rad_s":flu_to_body_ned(w["x"],w["y"],w["z"]),"orientation_rad":[0,0,quaternion_to_ned_yaw(q["w"],q["x"],q["y"],q["z"])]}
            sensors["imu"]={"timestampS":self._header_time(imu),"valid":valid,"payload":payload}
        gps=self.gps_model.sample(self.sim_time)
        if gps:sensors["gps"]={"timestampS":gps["timestamp_s"],"valid":bool(gps["valid"]),"payload":{"position_ned_m":gps.get("position_ned_m",[0,0,0])}}
        return {"time_s":self.sim_time,"sensors":sensors}
    def update_truth(self):
        msg=self.topics["odom"].latest
        if not msg:raise RuntimeError("missing odometry")
        p=msg.get("pose",{}).get("position",{});q=msg.get("pose",{}).get("orientation",{});tw=msg.get("twist",{});v=tw.get("linear",{});w=tw.get("angular",{})
        yaw=math.atan2(2*(q.get("w",1)*q.get("z",0)+q.get("x",0)*q.get("y",0)),1-2*(q.get("y",0)**2+q.get("z",0)**2))
        converted=self._node_request(self.converter,{"op":"gazebo_odom_to_task","time_s":self.sim_time,"enu":{"x":p.get("x",0),"y":p.get("y",0),"vx":v.get("x",0),"vy":v.get("y",0),"yaw_rad":yaw,"angular_z":w.get("z",0)}})["result"]
        self.truth={"position_ned_m":[converted["N_m"],converted["E_m"],-p.get("z",0)],"attitude_rad":[0,0,converted["yaw_rad"]],"velocity_body_mps":[converted["u_mps"],converted["v_mps"],-v.get("z",0)],"acceleration_body_mps2":[0,0,0],"angular_rate_body_rad_s":[0,0,converted["r_rad_s"]]}
    def response(self,**extra):
        self.update_truth();return {"ok":True,"observations":[self.observation()],"truth":self.truth,**extra}
    def step(self,action):
        previous_imu_stamp=self._header_time(self.topics["imu"].latest or {})
        previous_odom_stamp=self._header_time(self.topics["odom"].latest or {})
        effectors=action["actuators"]["effectors"]; normalized=[effectors["port"]["command"],effectors["starboard"]["command"],0,0]
        thrust=self._node_request(self.actuator,{"op":"step","action":normalized,"dt_s":CONTRACT_TICK_S})["thrust_newtons"]
        for side,value in zip(("port","starboard"),thrust):
            self.dexec("gz","topic","-t",f"/model/surveyor/joint/{side}_joint/cmd_thrust","-m","gz.msgs.Double","-n","1","-p",f"data: {value}")
        target=self.sim_time+CONTRACT_TICK_S
        self._advance_confirmed(PHYSICS_STEPS_PER_TICK)
        self.sim_time=self._clock_time(self.topics["clock"].wait_at_least(target,self._clock_time))
        self.topics["odom"].wait_after(previous_odom_stamp,self._header_time)
        # IMU is expected every physics iteration; blocking here prevents a
        # previous callback from masquerading as this interval's sample.
        if not self.topics["imu"].stalled:self.topics["imu"].wait_after(previous_imu_stamp,self._header_time)
        return self.response(terminated=False,truncated=False,info={"simulation_time_s":self.sim_time,"applied_thrust_newtons":thrust})
    def handle(self,request):
        op=request.get("op")
        if op=="reset":return self.reset(request["config"])
        if op=="step":return self.step(request["action"])
        if op=="truth":return self.response()
        if op=="diagnostic_stall":
            name=request["sensor"]
            if name not in ("imu","gps"):raise ValueError("only imu or gps may be stalled")
            self.topics[name].stalled=bool(request.get("enabled",True));return {"ok":True}
        if op=="diagnostic_status":
            return {"ok":True,"simulation_time_s":self.sim_time,"topic_timestamps_s":{
                name:[self._clock_time(x) if name=="clock" else self._header_time(x) for x in topic.accepted]
                for name,topic in self.topics.items()},"gps_conditioning":{
                    "rate_hz":self.gps_model.period_s**-1,"latency_s":self.gps_model.latency_s,
                    "position_std_m":self.gps_model.position_std_m,
                    "held_source_timestamp_s":None if self.gps_model.held is None else self.gps_model.held["timestamp_s"]}}
        if op=="diagnostic_idle":
            steps=int(request.get("physics_steps",2400))
            if steps<1:raise ValueError("physics_steps must be positive")
            start_time=self.sim_time;start_index=len(self.topics["odom"].accepted)
            # The public count is in contract physics ticks (50 ms); each is
            # resolved into five 10 ms DART iterations.
            self.service(self.world+"/control","gz.msgs.WorldControl",f"pause: true multi_step: {steps*PHYSICS_STEPS_PER_TICK-2}")
            # The service acknowledges scheduling before topic callbacks have
            # necessarily drained. Wait until the paused /clock is stable;
            # acceptance below judges the measured duration, not wall time or
            # a predicted terminal stamp.
            deadline=time.monotonic()+900;previous=None;stable_since=None
            while time.monotonic()<deadline:
                current=self._clock_time(self.topics["clock"].latest or {})
                if previous is not None and abs(current-previous)<1e-12:
                    stable_since=stable_since or time.monotonic()
                    if time.monotonic()-stable_since>=3:break
                else:stable_since=None
                previous=current;time.sleep(.25)
            else:raise TimeoutError("/clock did not become stable after bulk idle stepping")
            self.sim_time=self._clock_time(self.topics["clock"].latest or {})
            self.topics["odom"].wait_at_least(self.sim_time,self._header_time,timeout=600)
            messages=self.topics["odom"].accepted[start_index:]
            def rpy(message):
                q=message.get("pose",{}).get("orientation",{});w,x,y,z=(q.get(k,0) for k in ("w","x","y","z"))
                roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));s=2*(w*y-z*x);pitch=math.copysign(math.pi/2,s) if abs(s)>=1 else math.asin(s)
                return roll,pitch
            positions=[[m.get("pose",{}).get("position",{}).get(k,0) for k in ("x","y","z")] for m in messages]
            attitudes=[rpy(m) for m in messages]
            velocities=[[m.get("twist",{}).get("linear",{}).get(k,0) for k in ("x","y","z")] for m in messages]
            first=positions[0];last=positions[-1]
            return {"ok":True,"start_simulation_time_s":start_time,"end_simulation_time_s":self.sim_time,"physics_steps":steps,"odometry_samples":len(messages),
              "start_position_enu_m":first,"end_position_enu_m":last,"horizontal_displacement_m":math.hypot(last[0]-first[0],last[1]-first[1]),"vertical_displacement_m":last[2]-first[2],"vertical_range_m":max(p[2] for p in positions)-min(p[2] for p in positions),"max_speed_m_s":max(math.sqrt(sum(x*x for x in v)) for v in velocities),"max_abs_roll_rad":max(abs(x[0]) for x in attitudes),"max_abs_pitch_rad":max(abs(x[1]) for x in attitudes)}
        if op=="close":self._stop();return {"ok":True,"closed":True}
        raise ValueError(f"unknown operation {op}")

def main():
    runtime=Runtime()
    try:
        for line in sys.stdin:
            try: response=runtime.handle(json.loads(line))
            except Exception as error: response={"ok":False,"error":f"{type(error).__name__}: {error}"}
            print(json.dumps(response,separators=(",",":")),flush=True)
            if response.get("closed"):break
    finally:runtime._stop()
if __name__=="__main__":main()
