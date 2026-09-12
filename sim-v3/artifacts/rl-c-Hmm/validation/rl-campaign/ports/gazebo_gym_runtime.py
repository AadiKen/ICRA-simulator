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
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time

from external_sensor_model import ExternalGpsModel, flu_to_body_ned, gazebo_navsat_valid, inertial_acceleration_from_gazebo_imu, quaternion_to_ned_yaw
from gazebo_termination import TerminationMonitor

ROOT=Path(__file__).resolve().parents[3]
IMAGE=os.environ.get("BCOD_GAZEBO_IMAGE","174e8baad590")
PHYSICS_DT_S=.005
CONTRACT_TICK_S=.05
PHYSICS_STEPS_PER_TICK=round(CONTRACT_TICK_S/PHYSICS_DT_S)

def stamp(value):
    value=value or {}; return int(value.get("sec",0))+int(value.get("nsec",0))*1e-9

class JsonTopic:
    def __init__(self,container,topic,native=False,native_env=None):
        self.topic=topic; self.latest=None; self.items=queue.Queue(); self.sequence=0; self.stalled=False; self.accepted=[];self.parse_errors=0;self.last_unparsed=None
        command=["gz","topic","-e","-t",topic,"--json-output"] if native else ["docker","exec",container,"gz","topic","-e","-t",topic,"--json-output"]
        self.process=subprocess.Popen(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=1,env=native_env)
        self.thread=threading.Thread(target=self._read,daemon=True); self.thread.start()
    def _read(self):
        assert self.process.stdout
        for line in self.process.stdout:
            try: value=json.loads(line)
            except json.JSONDecodeError:self.parse_errors+=1;self.last_unparsed=line[:500];continue
            if not self.stalled: self.latest=value; self.sequence+=1; self.items.put(value); self.accepted.append(value)
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
        self.topics={}; self.gps_model=None; self.actuator=None; self.converter=None;self.transport=None;self.server=None
        self.truth=None; self.sim_time=0.; self.last_gps_stamp=None
        self.termination=TerminationMonitor();self.roll_rad=0.;self.pitch_rad=0.
        self.environment_requested={};self.diagnostic_termination_override=None
        self.idle_mode=os.environ.get("BCOD_GAZEBO_IDLE_MODE")=="1"
        self.native=os.environ.get("BCOD_GAZEBO_NATIVE")=="1"
        self.native_env=os.environ.copy()
        if self.native:self.native_env["GZ_PARTITION"]=f"bcod-gazebo-{os.getpid()}"
    def run(self,args,check=True,timeout=20):
        return subprocess.run(args,text=True,capture_output=True,check=check,timeout=timeout)
    def dexec(self,*args,**kwargs):
        if self.native:return subprocess.run(list(args),text=True,capture_output=True,check=kwargs.pop("check",True),timeout=kwargs.pop("timeout",20),env=self.native_env,**kwargs)
        return self.run(["docker","exec",self.container,*args],**kwargs)
    def service(self,name,reqtype,request):
        if self.transport is not None:
            match=re.search(r"multi_step:\s*(\d+)",request)
            requested=int(match.group(1)) if match else 0
            response=self._node_request(self.transport,{"op":"control","service":name,"multi_step":requested})
            if not response.get("ok"):raise RuntimeError(f"service {name} failed through persistent transport")
            return
        result=self.dexec("gz","service","-s",name,"--reqtype",reqtype,"--reptype","gz.msgs.Boolean","--timeout","10000","--req",request)
        if "data: true" not in result.stdout:raise RuntimeError(f"service {name} failed: {result.stdout} {result.stderr}")
    def _stop(self):
        for topic in self.topics.values():topic.close()
        self.topics={}
        for proc in (self.actuator,self.converter,self.transport):
            if proc and proc.poll() is None:proc.terminate()
        self.actuator=None;self.converter=None;self.transport=None
        if self.server and self.server.poll() is None:
            self.server.terminate()
            try:self.server.wait(timeout=5)
            except subprocess.TimeoutExpired:self.server.kill()
        self.server=None
        if not self.native:self.run(["docker","rm","-f",self.container],check=False)
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
        self.environment_requested=config.get("environment",{})
        prepare=["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/prepare-gazebo-episode.ts"),str(seed),out]
        if self.idle_mode:prepare.append("--idle-no-sensors")
        self.run(prepare)
        shutil.copy(ROOT/"validation/rl-campaign/ports/gazebo_transport_jsonl.py",Path(out)/"gazebo_transport_jsonl.py")
        self.world=f"/world/bcod_parity_gate-{seed}"
        if self.native:
            native_env=self.native_env.copy();native_env["GZ_SIM_RESOURCE_PATH"]=str(Path(out)/"models")
            self.server=subprocess.Popen(["gz","sim","-s","-r",str(Path(out)/"worlds"/f"gate-{seed}.sdf")],env=native_env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
        else:
            self.run(["docker","run","-d","--rm","--platform","linux/amd64","--name",self.container,"-e","GZ_SIM_RESOURCE_PATH=/gate/models","-v",f"{out}:/gate",IMAGE,"gz","sim","-s","-r",f"/gate/worlds/gate-{seed}.sdf"])
        self._wait_service()
        # Every episode gets a fresh container and a world generated with the
        # seeded pose. Do not issue reset-all here: Harmonic removes the
        # Buoyancy system's enabled-entity components during that redundant
        # reset, leaving the reconstructed model in gravity-only free fall.
        self.service(self.world+"/control","gz.msgs.WorldControl","pause: true")
        transport_command=["/usr/bin/python3","-u",str(Path(out)/"gazebo_transport_jsonl.py")] if self.native else ["docker","exec","-i",self.container,"python3","-u","/gate/gazebo_transport_jsonl.py"]
        self.transport=subprocess.Popen(transport_command,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE,env=self.native_env if self.native else None)
        initial=config["initial_state"]["position_ned_m"]
        wanted={"clock":self.world+"/clock","stats":self.world+"/stats","odom":"/odometry"}
        if not self.idle_mode:wanted.update({"imu":"/imu","gps":"/gps",**{f"contact{i}":f"/surveyor/contacts/hull_{i}" for i in range(3)}})
        self.topics={name:JsonTopic(self.container,topic,self.native,self.native_env if self.native else None) for name,topic in wanted.items()}
        # Let Gazebo Transport discovery connect the independent echo
        # subscribers before the first bounded step. This is startup plumbing,
        # not a simulation-time measurement.
        time.sleep(1.)
        self.actuator=subprocess.Popen(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/actuator-jsonl-bridge.ts")],cwd=ROOT,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self.converter=subprocess.Popen(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/task-trace-jsonl-bridge.ts")],cwd=ROOT,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self._node_request(self.actuator,{"op":"reset"})
        self.termination=TerminationMonitor();self.roll_rad=0.;self.pitch_rad=0.
        self.diagnostic_termination_override=None
        self.gps_model=ExternalGpsModel(seed,initial[0],initial[1],reference_latitude_deg=-33.72276876888639,reference_longitude_deg=150.67399110174387);self.last_gps_stamp=None
        # Advance one complete contract tick at the world's 5 ms physics
        # resolution.  The persistent transport sends only ``multi_step``;
        # combining ``step`` and ``multi_step`` would request an extra tick.
        self.service(self.world+"/control","gz.msgs.WorldControl",f"pause: true step: true multi_step: {PHYSICS_STEPS_PER_TICK}")
        self.topics["clock"].wait_at_least(CONTRACT_TICK_S,self._clock_time);self._wait_clock_stable();self.sim_time=self._clock_time(self.topics["clock"].latest or {})
        # Transport discovery can complete after the first odometry
        # publication, especially when two AMD64-emulated worlds start at
        # once.  Waiting alone cannot produce another sample while the world
        # is paused, so retry with bounded 50 ms simulation-time advances.
        # This still requires a genuinely fresh timestamp and never substitutes
        # wall-clock time or a cached message for sensor readiness.
        for attempt in range(6):
            try:
                self.topics["odom"].wait_after(-1.,self._header_time,timeout=2.)
                break
            except TimeoutError:
                if attempt==5:raise
                self._advance_confirmed(PHYSICS_STEPS_PER_TICK)
                self.sim_time=self._clock_time(self.topics["clock"].latest or {})
        return self.response()
    def _clock_time(self,msg):return stamp(msg.get("sim"))
    def _stats_iteration(self,msg):return int(msg.get("iterations",-1))
    def _wait_paused_stats(self,minimum_iteration=-1,after_sequence=-1,timeout=20):
        """Wait for Gazebo's authoritative paused state and iteration count."""
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            value=self.topics["stats"].latest or {}
            iteration=self._stats_iteration(value)
            if self.topics["stats"].sequence>after_sequence and value.get("paused") is True and iteration>=minimum_iteration:return value
            try:self.topics["stats"].items.get(timeout=min(.1,max(0,deadline-time.monotonic())))
            except queue.Empty:pass
        raise TimeoutError(f"{self.world}/stats did not report paused at iteration >= {minimum_iteration}")
    def _stats_time(self,msg):
        value=msg.get("simTime",msg.get("sim_time",msg.get("sim")))
        if not isinstance(value,dict):raise RuntimeError(f"Gazebo stats omitted simulation time: {msg}")
        return stamp(value)
    def _header_time(self,msg):return stamp(msg.get("header",{}).get("stamp"))
    def _imu_roll_pitch(self):
        msg=self.topics.get("imu").latest if "imu" in self.topics else None
        q=(msg or {}).get("orientation",{});values=[q.get(k) for k in ("w","x","y","z")]
        if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in values):return math.nan,math.nan
        qw,qx,qy,qz=values;roll=math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));s=2*(qw*qy-qz*qx);pitch=math.copysign(math.pi/2,s) if abs(s)>=1 else math.asin(s)
        return roll,-pitch
    def _wait_clock_stable(self,stable_s=.02,timeout=5):
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
        """Advance to an absolute time so delayed or duplicate requests are idempotent."""
        stats_sequence=self.topics["stats"].sequence
        before=self._wait_paused_stats(after_sequence=stats_sequence)
        start=self._stats_time(before)
        target=round(start+iterations*PHYSICS_DT_S,9)
        response=self._node_request(self.transport,{"op":"control","service":self.world+"/control","run_to_sim_time_s":target})
        if not response.get("ok"):raise RuntimeError(f"service {self.world}/control failed through persistent transport")
        end=self._clock_time(self.topics["clock"].wait_at_least(target,self._clock_time))
        if not math.isclose(end,target,rel_tol=0,abs_tol=1e-9):
            raise RuntimeError(f"Gazebo advanced to {end}s; requested absolute time {target}s")
    def _ingest_gps(self):
        if "gps" not in self.topics:return
        msg=self.topics["gps"].latest
        if not msg:return
        # Gazebo NavSat does not guarantee an incrementing `seq` header.  Its
        # simulation timestamp is the freshness/deduplication identity.
        ts=self._header_time(msg)
        if self.last_gps_stamp is not None and ts<=self.last_gps_stamp+1e-12:return
        self.last_gps_stamp=ts
        lat=msg.get("latitudeDeg");lon=msg.get("longitudeDeg");alt=msg.get("altitude",0)
        valid=gazebo_navsat_valid(lat,lon)
        self.gps_model.ingest(ts,float(lat or 0),float(lon or 0),valid,float(alt or 0))
    def _condition_imu_acceleration(self,_timestamp_s,values):return values
    def observation(self):
        self._ingest_gps(); sensors={}
        imu=self.topics.get("imu").latest if "imu" in self.topics else None
        if imu:
            q=imu.get("orientation",{});a=imu.get("linearAcceleration",{});w=imu.get("angularVelocity",{})
            valid=all(isinstance(x,(int,float)) and math.isfinite(x) for x in [q.get("w"),q.get("x"),q.get("y"),q.get("z"),a.get("x"),a.get("y"),a.get("z"),w.get("x"),w.get("y"),w.get("z")])
            inertial_accel=None if not valid else self._condition_imu_acceleration(self._header_time(imu),inertial_acceleration_from_gazebo_imu(q["w"],q["x"],q["y"],q["z"],a["x"],a["y"],a["z"]))
            payload=None if not valid else {"acceleration_body_mps2":flu_to_body_ned(*inertial_accel),"angular_rate_body_rad_s":flu_to_body_ned(w["x"],w["y"],w["z"]),"orientation_rad":[0,0,quaternion_to_ned_yaw(q["w"],q["x"],q["y"],q["z"])]}
            sensors["imu"]={"timestampS":self._header_time(imu),"valid":valid,"payload":payload}
        gps=self.gps_model.sample(self.sim_time)
        if gps:sensors["gps"]={"timestampS":gps["timestamp_s"],"valid":bool(gps["valid"]),"payload":{"position_ned_m":gps.get("position_ned_m",[0,0,0])}}
        return {"time_s":self.sim_time,"sensors":sensors}
    def update_truth(self):
        msg=self.topics["odom"].latest
        if not msg:raise RuntimeError("missing odometry")
        p=msg.get("pose",{}).get("position",{});q=msg.get("pose",{}).get("orientation",{});tw=msg.get("twist",{});v=tw.get("linear",{});w=tw.get("angular",{})
        qw,qx,qy,qz=(q.get("w",1),q.get("x",0),q.get("y",0),q.get("z",0))
        yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));self.roll_rad=math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));s=2*(qw*qy-qz*qx);self.pitch_rad=math.copysign(math.pi/2,s) if abs(s)>=1 else math.asin(s)
        converted=self._node_request(self.converter,{"op":"gazebo_odom_to_task","time_s":self.sim_time,"enu":{"x":p.get("x",0),"y":p.get("y",0),"vx":v.get("x",0),"vy":v.get("y",0),"yaw_rad":yaw,"angular_z":w.get("z",0)}})["result"]
        self.truth={"position_ned_m":[converted["N_m"],converted["E_m"],-p.get("z",0)],"attitude_rad":[self.roll_rad,-self.pitch_rad,converted["yaw_rad"]],"velocity_body_mps":[converted["u_mps"],converted["v_mps"],-v.get("z",0)],"acceleration_body_mps2":[0,0,0],"angular_rate_body_rad_s":[0,0,converted["r_rad_s"]]}
    def response(self,**extra):
        self.update_truth();return {"ok":True,"observations":[self.observation()],"truth":self.truth,**extra}
    def _thrust_topic(self,side):return f"/model/surveyor/joint/{side}_joint/cmd_thrust"
    def step(self,action):
        previous_odom_stamp=self._header_time(self.topics["odom"].latest or {})
        effectors=action["actuators"]["effectors"]; normalized=[effectors["port"]["command"],effectors["starboard"]["command"],0,0]
        thrust=self._node_request(self.actuator,{"op":"step","action":normalized,"dt_s":CONTRACT_TICK_S})["thrust_newtons"]
        commanded=[70*max(-1,min(1,float(value))) for value in normalized[:2]]
        for side,value in zip(("port","starboard"),thrust):
            self._node_request(self.transport,{"op":"publish","topic":self._thrust_topic(side),"value":value})
        target=self.sim_time+CONTRACT_TICK_S
        self._advance_confirmed(PHYSICS_STEPS_PER_TICK)
        self.sim_time=self._clock_time(self.topics["clock"].wait_at_least(target,self._clock_time))
        self.topics["odom"].wait_after(previous_odom_stamp,self._header_time)
        # Gazebo may legitimately omit a scheduled IMU publication. Keep the
        # latest sample and let CommonWaypointEnv's frozen age limit decide
        # whether it remains usable or must be zero-filled.
        response=self.response()
        contacts=[self.topics[name].latest for name in self.topics if "contact" in name and self.topics[name].latest]
        imu_roll,imu_pitch=self._imu_roll_pitch()
        achieved_for_monitor=thrust
        if self.diagnostic_termination_override=="instability":
            imu_roll=math.radians(61)
        elif self.diagnostic_termination_override=="precedence":
            imu_roll=math.nan
        if self.diagnostic_termination_override in ("grounding","precedence"):
            contacts.append({"collision1":"surveyor::base_link::hull_collision_0","collision2":"bathymetry_seabed::collision"})
        elif self.diagnostic_termination_override=="object_collision":
            contacts.append({"collision1":"surveyor::base_link::hull_collision_0","collision2":"test_buoy::collision"})
        if self.diagnostic_termination_override in ("allocation_failure","precedence"):
            achieved_for_monitor=[0.,0.]
        stop_reason=self.termination.update(dt_s=CONTRACT_TICK_S,roll_rad=imu_roll,pitch_rad=imu_pitch,contact_messages=contacts,commanded_thrust_n=commanded,achieved_thrust_n=achieved_for_monitor)
        return {**response,"terminated":stop_reason is not None,"truncated":False,"info":{"simulation_time_s":self.sim_time,"commanded_thrust_newtons":commanded,"applied_thrust_newtons":thrust,**({"stop_reason":stop_reason} if stop_reason else {})}}
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
            imu=self.topics.get("imu").latest if self.topics.get("imu") else None
            odom=self.topics.get("odom").latest if self.topics.get("odom") else None
            iq=(imu or {}).get("orientation",{});oq=(odom or {}).get("pose",{}).get("orientation",{})
            ia=(imu or {}).get("linearAcceleration",{})
            return {"ok":True,"simulation_time_s":self.sim_time,"topic_timestamps_s":{
                name:[self._clock_time(x) if name=="clock" else self._header_time(x) for x in topic.accepted]
                for name,topic in self.topics.items()},"topic_process_exit_codes":{name:topic.process.poll() for name,topic in self.topics.items()},"topic_parse_diagnostics":{name:{"parse_errors":topic.parse_errors,"last_unparsed":topic.last_unparsed} for name,topic in self.topics.items()},"yaw_diagnostic":{
                    "imu_timestamp_s":None if imu is None else self._header_time(imu),
                    "imu_quaternion_wxyz":None if imu is None else [iq.get("w"),iq.get("x"),iq.get("y"),iq.get("z")],
                    "imu_raw_linear_acceleration_flu_mps2":None if imu is None else [ia.get("x"),ia.get("y"),ia.get("z")],
                    "imu_compensated_linear_acceleration_body_ned_mps2":None if imu is None else flu_to_body_ned(*inertial_acceleration_from_gazebo_imu(iq["w"],iq["x"],iq["y"],iq["z"],ia["x"],ia["y"],ia["z"])),
                    "imu_converted_ned_yaw_rad":None if imu is None else quaternion_to_ned_yaw(iq["w"],iq["x"],iq["y"],iq["z"]),
                    "odom_timestamp_s":None if odom is None else self._header_time(odom),
                    "odom_quaternion_wxyz":None if odom is None else [oq.get("w"),oq.get("x"),oq.get("y"),oq.get("z")]},"gps_conditioning":{
                    "rate_hz":self.gps_model.period_s**-1,"latency_s":self.gps_model.latency_s,
                    "position_std_m":self.gps_model.position_std_m,
                    "geodetic_anchored":self.gps_model.geodetic_anchored,
                    "reference_latitude_longitude_deg":self.gps_model.reference,
                    "held_source_timestamp_s":None if self.gps_model.held is None else self.gps_model.held["timestamp_s"],
                    "held_valid":None if self.gps_model.held is None else self.gps_model.held["valid"],
                    "last_ingest_diagnostic":self.gps_model.last_diagnostic,
                    "held_diagnostic":None if self.gps_model.held is None else self.gps_model.held.get("diagnostic")},"environment":{"requested":self.environment_requested,"applied":{"current_mps":[0,0,0],"wind_mps":[0,0,0]},"mechanism":"No Gazebo wind/current plugin is loaded; the running world is still water."}}
        if op=="diagnostic_termination_override":
            kind=request.get("kind")
            if kind not in (None,"instability","grounding","object_collision","allocation_failure","precedence"):raise ValueError("invalid diagnostic termination override")
            self.diagnostic_termination_override=kind;self.termination=TerminationMonitor();return {"ok":True,"kind":kind}
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
