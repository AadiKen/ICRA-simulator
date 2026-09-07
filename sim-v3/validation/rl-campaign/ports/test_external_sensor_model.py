import math
import json
from pathlib import Path
import statistics
import struct
import subprocess
import unittest
from external_sensor_model import ExternalGpsModel, flu_to_body_ned, gazebo_navsat_valid, inertial_acceleration_from_gazebo_imu, quaternion_to_ned_yaw

class ExternalSensorModelTest(unittest.TestCase):
    def test_rate_latency_and_position_noise(self):
        model=ExternalGpsModel(7319,10000,10000)
        positions=[]
        for i in range(10001):
            t=i*.05; model.ingest(t,0,0,True); sample=model.sample(t)
            if sample and sample['timestamp_s'] == t-.2:
                positions.append(sample['position_ned_m'][0]-10000)
                self.assertNotIn('velocity_ned_mps',sample)
        self.assertAlmostEqual(statistics.pstdev(positions),.8,delta=.04)
    def test_frame_conversion(self):
        self.assertEqual(flu_to_body_ned(1,2,3),[1,-2,-3])
        self.assertAlmostEqual(quaternion_to_ned_yaw(1,0,0,0),math.pi/2)
        self.assertAlmostEqual(quaternion_to_ned_yaw(math.cos(-math.pi/4),0,0,math.sin(-math.pi/4)),-math.pi)
        self.assertAlmostEqual(quaternion_to_ned_yaw(math.cos((math.pi/2+1e-6)/2),0,0,math.sin((math.pi/2+1e-6)/2)),-1e-6)
    def test_python_and_canonical_typescript_yaw_are_bit_identical_for_quaternion_sweep(self):
        bridge=Path(__file__).with_name('task-trace-jsonl-bridge.ts')
        process=subprocess.Popen(['node','--experimental-strip-types',str(bridge)],text=True,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        try:
            for roll in (-1.1,-.2,0.,.4,1.2):
                for pitch in (-.7,0.,.6):
                    for yaw in (-math.pi,-2.4,-math.pi/2,-.3,0.,.8,math.pi-1e-9,math.pi):
                        cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
                        w=cr*cp*cy+sr*sp*sy;x=sr*cp*cy-cr*sp*sy;y=cr*sp*cy+sr*cp*sy;z=cr*cp*sy-sr*sp*cy
                        request={'op':'quaternion_to_ned_yaw','quaternion':{'w':w,'x':x,'y':y,'z':z}}
                        process.stdin.write(json.dumps(request,separators=(',',':'))+'\n');process.stdin.flush()
                        typescript=json.loads(process.stdout.readline())['result'];python=quaternion_to_ned_yaw(w,x,y,z)
                        self.assertEqual(struct.pack('>d',python),struct.pack('>d',typescript),(roll,pitch,yaw,python,typescript))
        finally:
            if process.poll() is None:
                process.stdin.write('{"op":"close"}\n');process.stdin.flush();process.stdin.close();process.wait(timeout=5)
            process.stdout.close()
    def test_gravity_is_rotated_before_removal(self):
        # Level: Gazebo reports +g along body Z.  Pitched 90 degrees: the same
        # reaction is along body -X; both map back to zero inertial acceleration.
        self.assertTrue(all(abs(x)<1e-12 for x in inertial_acceleration_from_gazebo_imu(1,0,0,0,0,0,9.81)))
        q=math.sqrt(.5)
        self.assertTrue(all(abs(x)<1e-12 for x in inertial_acceleration_from_gazebo_imu(q,0,q,0,-9.81,0,0)))
    def test_configured_geodetic_origin_is_not_first_fix(self):
        model=ExternalGpsModel(1,10000,10000,position_std_m=0,reference_latitude_deg=0,reference_longitude_deg=0)
        # One kilometre east on the equator, expressed as an ECEF-consistent
        # geodetic point on the local tangent plane.
        angle=math.atan2(1000,6_378_137);altitude=math.hypot(6_378_137,1000)-6_378_137
        model.ingest(0,0,math.degrees(angle),True,altitude)
        sample=model.sample(.2)['position_ned_m']
        self.assertAlmostEqual(sample[0],0,places=7);self.assertAlmostEqual(sample[1],1000,places=7)
    def test_deliberately_stalled_gps_does_not_create_new_samples(self):
        model=ExternalGpsModel(7319,10000,10000)
        model.ingest(5.0,-33.7,150.6,True)
        self.assertIsNone(model.sample(5.19))
        released=model.sample(5.2)
        self.assertEqual(released['timestamp_s'],5.0)
        # Simulated /clock advances but the topic callback is deliberately not
        # invoked. The conditioner must not refresh the held sample timestamp.
        self.assertIs(model.sample(5.75),released)
        self.assertEqual(model.sample(5.75)['timestamp_s'],5.0)
    def test_gazebo_navsat_validity_is_finite_geodetic_output(self):
        self.assertTrue(gazebo_navsat_valid(-33.7,150.6))
        self.assertFalse(gazebo_navsat_valid(float('nan'),150.6))
        self.assertFalse(gazebo_navsat_valid(-33.7,float('inf')))

if __name__ == '__main__': unittest.main()
