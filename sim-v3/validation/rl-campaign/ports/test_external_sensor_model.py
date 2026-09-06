import math
import statistics
import unittest
from external_sensor_model import ExternalGpsModel, flu_to_body_ned, quaternion_to_ned_yaw

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

if __name__ == '__main__': unittest.main()
