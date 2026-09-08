#!/usr/bin/env python3
"""Drive Gazebo Vehicle C from the persistent TypeScript controller bridge."""
from __future__ import annotations
import argparse,json,math,os,subprocess,tempfile,time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.world_control_pb2 import WorldControl

def main()->int:
    p=argparse.ArgumentParser();p.add_argument("--world",required=True);p.add_argument("--output",required=True);a=p.parse_args();root=Path(__file__).resolve().parents[2]
    bridge=subprocess.Popen(["node","--experimental-strip-types",str(Path(__file__).with_name("vehicle_c_controller_bridge.ts"))],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    bridge.stdin.write("{}\n");bridge.stdin.flush();command=json.loads(bridge.stdout.readline());heading=command["heading_rad"]
    source=Path(a.world).read_text();source=source.replace("<pose>0 0 0 0 0 1.5707963267948966</pose>",f"<pose>0 0 0 0 0 {math.pi/2-heading:.17g}</pose>")
    with tempfile.TemporaryDirectory(prefix="vehicle-c-closed-loop-") as td:
      world=Path(td)/"world.sdf";world.write_text(source);env=dict(os.environ);models=str(root/"gazebo/models");env["GZ_SIM_RESOURCE_PATH"]=models+(os.pathsep+env["GZ_SIM_RESOURCE_PATH"] if env.get("GZ_SIM_RESOURCE_PATH") else "");server=subprocess.Popen(["gz","sim","--force-version","8","-s","-v","1",str(world)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env)
      try:
        topic="/model/vehicle-c-azimuth/odometry";deadline=time.monotonic()+20
        while time.monotonic()<deadline:
          if topic in subprocess.run(["gz","topic","-l"],text=True,capture_output=True).stdout:break
          time.sleep(.2)
        else:raise RuntimeError("Gazebo odometry topic did not appear")
        messages=[];node=Node();node.subscribe(Odometry,topic,lambda msg,*_:messages.append(msg));pub={}
        for side in ("port","starboard"):
          pub[side+"_thrust"]=node.advertise(f"/model/vehicle-c-azimuth/joint/{side}_propeller_joint/cmd_thrust",Double);pub[side+"_azimuth"]=node.advertise(f"/model/vehicle-c-azimuth/joint/{side}_azimuth_joint/0/cmd_pos",Double)
        time.sleep(.3);rows=[]
        while not command["done"]:
          for side in ("port","starboard"):
            pub[side+"_thrust"].publish(Double(data=command[side+"_thrust_n"]));pub[side+"_azimuth"].publish(Double(data=-command[side+"_azimuth_rad"]))
          before=len(messages);ok=False;response=None
          for _ in range(3):
            ok,response=node.request("/world/vehicle_c_check/control",WorldControl(multi_step=5),WorldControl,Boolean,5000)
            if ok and response.data:break
            time.sleep(.05)
          if not ok or not response or not response.data:raise RuntimeError("Gazebo fixed-step request failed after three transport attempts")
          deadline=time.monotonic()+2
          while len(messages)<=before and time.monotonic()<deadline:time.sleep(.002)
          if len(messages)<=before:raise RuntimeError("Gazebo odometry did not advance")
          m=messages[-1];q=m.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));enu={"x":m.pose.position.x,"y":m.pose.position.y,"yaw_rad":yaw,"vx":m.twist.linear.x,"vy":m.twist.linear.y,"angular_z":m.twist.angular.z}
          bridge.stdin.write(json.dumps({"enu":enu})+"\n");bridge.stdin.flush();command=json.loads(bridge.stdout.readline());rows.append({k:v for k,v in command.items() if k not in ("done","success","closest_approach_m")})
        Path(a.output).parent.mkdir(parents=True,exist_ok=True);Path(a.output).write_text(json.dumps({"schema_version":1,"rows":rows,"outcome":{"success":command["success"],"closest_approach_m":command["closest_approach_m"]}},indent=2)+"\n")
      finally:
        server.send_signal(2)
        try:server.wait(timeout=10)
        except subprocess.TimeoutExpired:server.kill()
    bridge.terminate();return 0
if __name__=="__main__":raise SystemExit(main())
