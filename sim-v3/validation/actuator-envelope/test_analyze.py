import importlib.util,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location("actuator_envelope_analyze",Path(__file__).with_name("analyze.py"));analyze=importlib.util.module_from_spec(spec);spec.loader.exec_module(analyze)

def capture(arm,speed=2.):
 return {"schema_version":1,"artifact_kind":"native-actuator-envelope-capture","arm":arm,"disturbance":{"current_mps":0.0,"wind_mps":0.0,"waves":"off"},"speed_curve":[{"command_level":q,"steady_speed_mps":speed*q,"steady_window_speed_slope_mps2":0.001,"commanded_force_n_per_thruster":95*q,"delivered_force_n_per_thruster":95*q,"time_to_speed_s":{"0.5":1.,"1.0":2. if q>=.5 else None,"1.5":3. if q>=.75 else None}} for q in (.25,.5,.75,1.)],"turning":{"heading_step":{"settling_time_s":4.,"overshoot_deg":2.},"waypoint_90deg":{"turn_time_s":5.,"distance_cost_m":1.}},"policy_operating_point":{"command_p05":.1,"command_p50":.4,"command_p95":.8,"mean_speed_mps":1.}}
class AnalyzeTest(unittest.TestCase):
 def test_complete_matrix_recommends_option_3_without_mutation(self):
  report=analyze.analyze({arm:capture(arm) for arm in analyze.ARMS});self.assertEqual(report["recommendation"]["option"],3);self.assertFalse(report["caps_changed"]);self.assertFalse(any(x["flagged"] for x in report["pairwise"]))
 def test_nonsteady_capture_is_rejected(self):
  rows={arm:capture(arm) for arm in analyze.ARMS};rows["gazebo"]["speed_curve"][-1]["steady_window_speed_slope_mps2"]=.02
  with self.assertRaisesRegex(ValueError,"did not reach steady state"):analyze.analyze(rows)
if __name__=="__main__":unittest.main()
