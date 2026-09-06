#!/usr/bin/env python3
"""Clock VRX commands from 20 Hz odometry and emit exactly one row per physics sample."""
import json, math, pathlib, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from external_sensor_model import ExternalGpsModel, flu_to_body_ned, quaternion_to_ned_yaw

class Runtime(Node):
    def __init__(self,schedule,output):
        super().__init__('vrx_gate7_runtime'); self.schedule=schedule; self.output=pathlib.Path(output)
        self.i=0; self.imu=None; self.gps=None
        initial=schedule['reset']['initial_state']
        self.gps_model=ExternalGpsModel(schedule['reset']['seed'],initial[0],initial[1])
        self.port=self.create_publisher(Float64,'/surveyor/thrusters/port/thrust',10)
        self.starboard=self.create_publisher(Float64,'/surveyor/thrusters/starboard/thrust',10)
        self.create_subscription(Imu,'/imu',self.on_imu,20); self.create_subscription(NavSatFix,'/gps',self.on_gps,20)
        self.create_subscription(Odometry,'/odometry',self.on_odom,20)
    def on_imu(self,msg): self.imu=msg
    def on_gps(self,msg):
        timestamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.gps_model.ingest(timestamp,msg.latitude,msg.longitude,msg.status.status>=0); self.gps=msg
    def on_odom(self,msg):
        if self.i>=self.schedule['physics_samples']: return
        commands=self.schedule['transport'][self.i]
        vals={c['topic']:c['value'] for c in commands}
        left=float(vals['/surveyor/thrusters/port/thrust']); right=float(vals['/surveyor/thrusters/starboard/thrust'])
        self.port.publish(Float64(data=left)); self.starboard.publish(Float64(data=right))
        q=msg.pose.pose.orientation; yaw_enu=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); yaw=math.pi/2-yaw_enu
        v=msg.twist.twist.linear; imu=self.imu; now=self.i*self.schedule['physics_dt_s']; imu_sample=None
        if imu is not None:
            iq=imu.orientation; timestamp=imu.header.stamp.sec+imu.header.stamp.nanosec*1e-9
            imu_sample={'timestamp_s':timestamp,'valid':True,'linear_accel_body':flu_to_body_ned(imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z),'angular_rate_body':flu_to_body_ned(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z),'yaw_ned_rad':quaternion_to_ned_yaw(iq.w,iq.x,iq.y,iq.z)}
        row={'time_s':now,'state':[msg.pose.pose.position.y,msg.pose.pose.position.x,yaw,math.cos(yaw)*v.y+math.sin(yaw)*v.x,-math.sin(yaw)*v.y+math.cos(yaw)*v.x,-msg.twist.twist.angular.z],'imu':imu_sample,'gps':self.gps_model.sample(now),'thruster_newtons':[left,right]}
        with self.output.open('a') as f: f.write(json.dumps(row,separators=(',',':'))+'\n')
        self.i+=1
        if self.i==self.schedule['physics_samples']: rclpy.shutdown()

def main():
    schedule=json.loads(pathlib.Path(sys.argv[1]).read_text()); output=sys.argv[2]; pathlib.Path(output).unlink(missing_ok=True)
    rclpy.init(); node=Runtime(schedule,output)
    try: rclpy.spin(node)
    finally: node.destroy_node()
if __name__=='__main__': main()
