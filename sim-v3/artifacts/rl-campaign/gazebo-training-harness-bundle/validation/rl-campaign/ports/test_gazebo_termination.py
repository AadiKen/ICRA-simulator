import math,unittest
from gazebo_termination import TerminationMonitor,classify_contacts

class GazeboTerminationTests(unittest.TestCase):
 def test_grounding_precedes_object(self):
  messages=[{"contact":[{"collision1":{"name":"surveyor::base_link::hull_collision_0"},"collision2":{"name":"dock::collision"}},{"collision2":{"name":"bathymetry_seabed::collision"}}]}]
  self.assertEqual(classify_contacts(messages),"grounding")
 def test_object_collision(self):
  self.assertEqual(classify_contacts([{"collision1":"surveyor::base_link::hull_collision_1","collision2":"buoy::collision"}]),"object_collision")
 def test_instability_requires_one_simulated_second(self):
  m=TerminationMonitor();result=None
  for _ in range(19):result=m.update(dt_s=.05,roll_rad=math.radians(61),pitch_rad=0,contact_messages=[],commanded_thrust_n=[0,0],achieved_thrust_n=[0,0])
  self.assertIsNone(result);self.assertEqual(m.update(dt_s=.05,roll_rad=math.radians(61),pitch_rad=0,contact_messages=[],commanded_thrust_n=[0,0],achieved_thrust_n=[0,0]),"instability")
 def test_nonfinite_state_is_immediate(self):
  self.assertEqual(TerminationMonitor().update(dt_s=.05,roll_rad=math.nan,pitch_rad=0,contact_messages=[],commanded_thrust_n=[0,0],achieved_thrust_n=[0,0]),"instability")
 def test_allocation_failure_requires_ten_control_intervals(self):
  m=TerminationMonitor();result=None
  for _ in range(19):result=m.update(dt_s=.05,roll_rad=0,pitch_rad=0,contact_messages=[],commanded_thrust_n=[70,70],achieved_thrust_n=[0,0])
  self.assertIsNone(result);self.assertEqual(m.update(dt_s=.05,roll_rad=0,pitch_rad=0,contact_messages=[],commanded_thrust_n=[70,70],achieved_thrust_n=[0,0]),"allocation_failure")
 def test_post_lag_command_does_not_report_expected_motor_dynamics_as_failure(self):
  m=TerminationMonitor()
  lagged=[0.,0.]
  for index in range(40):
   target=70. if index%2==0 else -70.
   alpha=1-math.exp(-.05/.35)
   lagged=[value+(target-value)*alpha for value in lagged]
   self.assertIsNone(m.update(dt_s=.05,roll_rad=0,pitch_rad=0,contact_messages=[],commanded_thrust_n=lagged,achieved_thrust_n=lagged))
 def test_precedence(self):
  m=TerminationMonitor();m.unstable_s=.95;m.allocation_bad_s=.95
  self.assertEqual(m.update(dt_s=.05,roll_rad=math.radians(61),pitch_rad=0,contact_messages=[{"collision2":"ground_plane"}],commanded_thrust_n=[70,70],achieved_thrust_n=[0,0]),"instability")

if __name__=="__main__":unittest.main()
