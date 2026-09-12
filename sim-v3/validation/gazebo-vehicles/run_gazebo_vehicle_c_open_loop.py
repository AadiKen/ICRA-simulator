#!/usr/bin/env python3
"""Replay Vehicle C schedules with full-resolution odometry and delivered-actuator logging."""
from __future__ import annotations
import argparse,json,math,os,subprocess,tempfile,time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics

MIN_THRUST=-670.2530375080305;MAX_THRUST=1340.506075016061
def qrel_yaw(base,pod):
    a=(-base.x,-base.y,-base.z,base.w);b=(pod.x,pod.y,pod.z,pod.w)
    x=a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1];y=a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0];z=a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3];w=a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def seconds(value):
    return value.sec + value.nsec * 1e-9
def main():
    p=argparse.ArgumentParser();p.add_argument("--schedule",required=True);p.add_argument("--output",required=True);p.add_argument("--world",required=True);a=p.parse_args();schedule_doc=json.loads(Path(a.schedule).read_text());samples=schedule_doc["samples"];sample_dt=float(schedule_doc["dt_s"]);physics_dt=.01;iteration_delta=round(sample_dt/physics_dt)
    if iteration_delta<1 or abs(iteration_delta*physics_dt-sample_dt)>1e-12:raise ValueError("Schedule dt_s must be a positive integer multiple of the 0.01 s Gazebo physics step")
    current=schedule_doc.get("current_enu_mps",{"x":0.0,"y":0.0,"z":0.0});root=Path(__file__).resolve().parents[2]
    source=Path(a.world).read_text().replace("</world>",'<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/></world>')
    with tempfile.TemporaryDirectory(prefix="vehicle-c-open-loop-") as td:
      world=Path(td)/"world.sdf";world.write_text(source);env=dict(os.environ);env["GZ_SIM_RESOURCE_PATH"]=str(root/"gazebo/models")+(os.pathsep+env["GZ_SIM_RESOURCE_PATH"] if env.get("GZ_SIM_RESOURCE_PATH") else "");server=subprocess.Popen(["gz","sim","--force-version","8","-s","-v","1",str(world)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env)
      try:
        topic="/model/vehicle-c-azimuth/odometry";deadline=time.monotonic()+20
        while time.monotonic()<deadline:
          if topic in subprocess.run(["gz","topic","-l"],text=True,capture_output=True).stdout:break
          time.sleep(.2)
        else:raise RuntimeError("Gazebo odometry topic did not appear")
        odom=[];poses=[];stats=[];node=Node();node.subscribe(Odometry,topic,lambda m,*_:odom.append(m));node.subscribe(Pose_V,"/world/vehicle_c_check/dynamic_pose/info",lambda m,*_:poses.append(m));node.subscribe(WorldStatistics,"/world/vehicle_c_check/stats",lambda m,*_:stats.append(m));pub={}
        for side in ("port","starboard"):
          pub[side+"_thrust"]=node.advertise(f"/model/vehicle-c-azimuth/joint/{side}_propeller_joint/cmd_thrust",Double);pub[side+"_azimuth"]=node.advertise(f"/model/vehicle-c-azimuth/joint/{side}_azimuth_joint/0/cmd_pos",Double)
        current_pub=node.advertise("/ocean_current",Vector3d);current_pub.publish(Vector3d(x=float(current["x"]),y=float(current["y"]),z=float(current["z"])))
        deadline=time.monotonic()+5
        while not stats and time.monotonic()<deadline:time.sleep(.002)
        if not stats:raise RuntimeError("Gazebo world statistics did not appear")
        paused=False
        for _ in range(3):
          before_pause_messages=len(stats);ok,response=node.request("/world/vehicle_c_check/control",WorldControl(pause=True),WorldControl,Boolean,5000)
          deadline=time.monotonic()+5
          while len(stats)<=before_pause_messages and time.monotonic()<deadline:time.sleep(.002)
          if len(stats)>before_pause_messages and stats[-1].paused:
            paused=True;break
        if not paused:raise RuntimeError("Gazebo world did not enter the required paused stepping mode")
        # Prime one paused iteration so the stats stream publishes an authoritative
        # post-startup counter instead of leaving its initial zero-valued sample last.
        node.request("/world/vehicle_c_check/control",WorldControl(pause=True,multi_step=1),WorldControl,Boolean,5000)
        # Drain startup publications before defining the paused-world baseline.
        stable_since=time.monotonic();last_startup_iteration=stats[-1].iterations
        while time.monotonic()-stable_since<.2:
          time.sleep(.002)
          if stats[-1].iterations!=last_startup_iteration:
            last_startup_iteration=stats[-1].iterations;stable_since=time.monotonic()
        initial_iteration=stats[-1].iterations;initial_sim_time=seconds(stats[-1].sim_time);captured=[]
        for i,row in enumerate(samples):
          for side in ("port","starboard"):
            pub[side+"_thrust"].publish(Double(data=row[side+"_thrust_n"]));pub[side+"_azimuth"].publish(Double(data=-row[side+"_azimuth_rad"]))
          before=len(odom);target=initial_iteration+(i+1)*iteration_delta;current=stats[-1].iterations
          if current!=target-iteration_delta:raise RuntimeError(f"Unexpected pre-step iteration {current}; expected {target-iteration_delta}")
          node.request("/world/vehicle_c_check/control",WorldControl(pause=True,multi_step=iteration_delta),WorldControl,Boolean,100)
          # Ignore transport success and intermediate stats alike. The request is
          # never repeated; only the absolute postcondition can release this sample.
          deadline=time.monotonic()+10
          while stats[-1].iterations<target and time.monotonic()<deadline:time.sleep(.002)
          if stats[-1].iterations<target:raise RuntimeError(f"Gazebo did not reach target iteration {target}; stopped at {stats[-1].iterations}")
          if stats[-1].iterations>target:raise RuntimeError(f"Gazebo overshot target iteration {target}: {stats[-1].iterations}")
          deadline=time.monotonic()+2
          while len(odom)<=before and time.monotonic()<deadline:time.sleep(.002)
          if len(odom)<=before:raise RuntimeError("Gazebo odometry did not advance")
          observed=stats[-1];expected_time=(i+1)*sample_dt;elapsed=seconds(observed.sim_time)-initial_sim_time
          if observed.iterations!=target or abs(elapsed-expected_time)>1e-9:raise RuntimeError(f"Synchronization failure at sample {i}: iteration={observed.iterations}, elapsed={elapsed}")
          captured.append((odom[-1],poses[-1] if poses else None,row,observed.iterations,elapsed))
        rows=[]
        for i,(m,pv,command,iteration,sim_time_s) in enumerate(captured):
          q=m.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));angles={"port":None,"starboard":None}
          if pv:
            named={p.name.split("::")[-1]:p for p in pv.pose};base=named.get("base_link")
            if base:
              for side in angles:
                pod=named.get(side+"_pod");angles[side]=-qrel_yaw(base.orientation,pod.orientation) if pod else None
          rows.append({"step":i,"time_s":sim_time_s,"gazebo_iteration":iteration,"gazebo_sim_time_s":sim_time_s,"iteration_delta":iteration_delta,"sim_time_delta_s":sample_dt,"enu":{"x":m.pose.position.x,"y":m.pose.position.y,"yaw_rad":yaw,"vx":m.twist.linear.x,"vy":m.twist.linear.y,"angular_z":m.twist.angular.z},"command":command,"delivered":{"port_thrust_n":max(MIN_THRUST,min(MAX_THRUST,command["port_thrust_n"])),"port_azimuth_rad":angles["port"],"starboard_thrust_n":max(MIN_THRUST,min(MAX_THRUST,command["starboard_thrust_n"])),"starboard_azimuth_rad":angles["starboard"]}})
        out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps({"schema_version":3,"synchronization":{"source":"/world/vehicle_c_check/stats","initial_iteration":initial_iteration,"initial_sim_time_s":initial_sim_time,"target_policy":f"absolute initial_iteration + {iteration_delta}*(sample_index+1)","blind_retries":False,"asserted_iteration_delta":iteration_delta,"asserted_sim_time_delta_s":sample_dt},"current_enu_mps":current,"rows":rows,"delivery_basis":{"thrust":"direct cmd_thrust force after corrected SDF clipping","azimuth":"measured base-to-pod relative pose from temporary SceneBroadcaster instrumentation","current":"/ocean_current Hydrodynamics system input"}},indent=2)+"\n")
      finally:
        server.send_signal(2)
        try:server.wait(timeout=10)
        except subprocess.TimeoutExpired:server.kill()
    return 0
if __name__=="__main__":raise SystemExit(main())
