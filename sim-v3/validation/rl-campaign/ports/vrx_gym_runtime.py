#!/usr/bin/env python3
"""Live VRX v3.0.1 JSONL runtime for ``VrxGymEnv``.

The lifecycle and synchronization are inherited from the validated Gazebo
adapter.  This adapter changes only episode generation, the pinned VRX image,
plugin search paths, world name, and the live Surveyor topic namespace.
"""
from __future__ import annotations

import shutil
import subprocess
import tempfile
import time
import math
import json
import os
from pathlib import Path

from gazebo_gym_runtime import (
    CONTRACT_TICK_S, PHYSICS_STEPS_PER_TICK, ExternalGpsModel, JsonTopic,
    ROOT, Runtime as GazeboRuntime, TerminationMonitor,
)

VRX_IMAGE="leadcat/vrx:surveyor-patched-v3.0.1"
VRX_PHYSICS_DT_S=.05
ODOMETRY_YAW_RATE_WARMUP_S=.5


class Runtime(GazeboRuntime):
    def update_truth(self):
        super().update_truth()
        valid_after=getattr(self,"odometry_yaw_rate_valid_after_s",math.inf)
        self.odometry_yaw_rate_valid=self.sim_time>=valid_after-1e-12
        if not self.odometry_yaw_rate_valid:self.truth["angular_rate_body_rad_s"][2]=0.

    def _condition_imu_acceleration(self,timestamp_s,values):
        if self.imu_filter_timestamp is not None and timestamp_s<=self.imu_filter_timestamp+1e-12:return self.imu_filter_output
        dt=VRX_PHYSICS_DT_S if self.imu_filter_timestamp is None else timestamp_s-self.imu_filter_timestamp
        alpha=1-math.exp(-2*math.pi*1.0*dt);stage=list(values)
        for index in range(4):
            self.imu_filter_stages[index]=[old+alpha*(new-old) for old,new in zip(self.imu_filter_stages[index],stage)]
            stage=self.imu_filter_stages[index]
        self.imu_filter_timestamp=timestamp_s;self.imu_filter_output=list(stage);return self.imu_filter_output
    def _advance_confirmed(self,_iterations):
        """Advance exactly one VRX 50 ms physics iteration."""
        start=self._wait_clock_stable();target=start+VRX_PHYSICS_DT_S
        self.service(self.world+"/control","gz.msgs.WorldControl","pause: true multi_step: 1")
        self.topics["clock"].wait_at_least(target,self._clock_time);end=self._wait_clock_stable()
        if not math.isclose(end-start,VRX_PHYSICS_DT_S,rel_tol=0,abs_tol=1e-9):
            raise RuntimeError(f"VRX advanced {end-start}s; expected {VRX_PHYSICS_DT_S}s")

    def reset(self,config):
        self._stop();self.temp=tempfile.TemporaryDirectory(prefix="bcod-vrx-runtime-")
        seed=int(config["experiment"]["seed"]);out=self.temp.name
        self.environment_requested=config.get("environment",{})
        environment_scale=0 if all(float(x)==0 for key in ("current_mps","wind_mps") for x in self.environment_requested.get(key,[0,0,0])) else 1
        collision_scenario=os.environ.get("BCOD_VRX_COLLISION_SCENARIO","none")
        self.surface_mode=os.environ.get("BCOD_VRX_SURFACE_MODE","none" if os.environ.get("BCOD_VRX_DISABLE_SURFACE")=="1" else "both")
        if self.surface_mode not in {"both","port","starboard","none"}:raise ValueError(f"invalid BCOD_VRX_SURFACE_MODE={self.surface_mode}")
        self.surface_enabled=self.surface_mode!="none"
        self.run(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/prepare-vrx-episode.ts"),str(seed),out,str(environment_scale),"1",str(environment_scale),str(environment_scale),collision_scenario,self.surface_mode])
        schedule=json.loads((Path(out)/"transport.json").read_text())
        disturbance=schedule["reset"]["disturbance"]
        current_angle=math.radians(disturbance["current_direction_deg"]);wind_angle=math.radians(disturbance["wind_direction_deg"])
        self.environment_applied={
            "current_mps":[environment_scale*disturbance["current_speed_m_s"]*math.cos(current_angle),environment_scale*disturbance["current_speed_m_s"]*math.sin(current_angle),0],
            "wind_mps":[environment_scale*disturbance["wind_speed_m_s"]*math.cos(wind_angle),environment_scale*disturbance["wind_speed_m_s"]*math.sin(wind_angle),0],
            "wave":{"gain":0,"mechanism":"No wave-parameter publisher in the common-task world; flat water is explicit."}}
        shutil.copy(ROOT/"validation/rl-campaign/ports/gazebo_transport_jsonl.py",Path(out)/"gazebo_transport_jsonl.py")
        self.world="/world/surveyor_vrx"
        self.run(["docker","run","-d","--rm","--platform","linux/amd64","--name",self.container,
            "-e","GZ_SIM_RESOURCE_PATH=/gate/models",
            "-e","GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib",
            "-e","LD_LIBRARY_PATH=/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib:/opt/ros/jazzy/lib",
            "-v",f"{out}:/gate",VRX_IMAGE,"gz","sim","-s","/gate/world.sdf"])
        self._wait_service();self.service(self.world+"/control","gz.msgs.WorldControl","pause: true")
        self.transport=subprocess.Popen(["docker","exec","-i",self.container,"python3","-u","/gate/gazebo_transport_jsonl.py"],text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        initial=config["initial_state"]["position_ned_m"]
        wanted={"clock":self.world+"/clock","odom":"/odometry","imu":"/imu","gps":"/gps","contact0":"/surveyor/contacts/hull_0","contact1":"/surveyor/contacts/hull_1","world_contacts":self.world+"/physics/contacts"}
        self.topics={name:JsonTopic(self.container,topic) for name,topic in wanted.items()}
        time.sleep(1.)
        self.actuator=subprocess.Popen(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/actuator-jsonl-bridge.ts")],cwd=ROOT,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self.converter=subprocess.Popen(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/task-trace-jsonl-bridge.ts")],cwd=ROOT,text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        self._node_request(self.actuator,{"op":"reset"});self.termination=TerminationMonitor();self.roll_rad=0.;self.pitch_rad=0.;self.diagnostic_termination_override=None
        self.imu_filter_stages=[[0.,0.,0.] for _ in range(4)];self.imu_filter_timestamp=None;self.imu_filter_output=[0.,0.,0.]
        self.gps_model=ExternalGpsModel(seed,initial[0],initial[1],reference_latitude_deg=-33.72276876888639,reference_longitude_deg=150.67399110174387);self.last_gps_stamp=None
        self.service(self.world+"/control","gz.msgs.WorldControl","pause: true multi_step: 1")
        self.topics["clock"].wait_at_least(CONTRACT_TICK_S,self._clock_time);self._wait_clock_stable();self.sim_time=self._clock_time(self.topics["clock"].latest or {})
        for attempt in range(6):
            try:self.topics["odom"].wait_after(-1.,self._header_time,timeout=2.);break
            except TimeoutError:
                if attempt==5:raise
                self._advance_confirmed(PHYSICS_STEPS_PER_TICK);self.sim_time=self._clock_time(self.topics["clock"].latest or {})
        # Drain the final startup clock callback so the public reset epoch is
        # the exact origin used by the first controlled step.
        self.sim_time=self._wait_clock_stable()
        # OdometryPublisher estimates twist from an internal pose history that
        # is not reset with the model.  Across multiple seeds its yaw-rate
        # transient persists for four 0.1 s control steps and settles at the
        # fifth (0.6 s simulation time for this reset sequence).
        self.odometry_yaw_rate_valid_after_s=self.sim_time+ODOMETRY_YAW_RATE_WARMUP_S
        return self.response()

    def _thrust_topic(self,side):return f"/surveyor/thrusters/{side}/thrust"

    def handle(self,request):
        response=super().handle(request)
        if request.get("op")=="diagnostic_status":
            listed=self.dexec("gz","topic","-l",check=False).stdout.splitlines()
            response["vrx"]={"image":VRX_IMAGE,"gazebo_sim_version":"8.10.0","topic_namespace":"/surveyor/thrusters/{port,starboard}/thrust","ros2_namespace_verified_live":True,
                "surface_enabled":self.surface_enabled,"surface_mode":self.surface_mode,"published_contact_topics":[topic for topic in listed if "contact" in topic.lower()]}
            response["environment"]={"requested":self.environment_requested,"applied":self.environment_applied,
                "mechanism":{"current":"episode-local libVrxCurrentRelativeVelocity.so SDF current_enu",
                             "wind":"episode-local libVrxSurveyorRelativeWind.so SDF wind_enu",
                             "wave":"VRX Surface subscribes to /vrx/wavefield/parameters; common-task protocol has no seeded wave distribution and holds gain at zero"},
                "locked_to_competition_preset":False}
            response["odometry_yaw_rate_warmup"]={"valid":self.odometry_yaw_rate_valid,"valid_after_simulation_time_s":self.odometry_yaw_rate_valid_after_s,"duration_s":ODOMETRY_YAW_RATE_WARMUP_S,"invalid_control_steps_after_reset":4,"policy":"zero-fill odometry-derived truth yaw rate; IMU gyro is unaffected"}
        return response


def main():
    import json,sys
    runtime=Runtime()
    try:
        for line in sys.stdin:
            try:response=runtime.handle(json.loads(line))
            except Exception as error:response={"ok":False,"error":f"{type(error).__name__}: {error}"}
            print(json.dumps(response,separators=(",",":")),flush=True)
            if response.get("closed"):break
    finally:runtime._stop()


if __name__=="__main__":main()
