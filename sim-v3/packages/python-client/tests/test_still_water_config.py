from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv

class NoopBridge:
 def close(self):pass

class StillWaterConfigTests(unittest.TestCase):
 def test_primary_arm_is_exactly_zero_for_all_sampled_seeds(self):
  env=CommonWaypointEnv(ROOT,bridge=NoopBridge(),disturbance_mode="zero")
  try:
   for seed in (0,1,7319,30000,2**32-1):
    config=env._config(seed)
    self.assertEqual(config["environment"]["current_mps"],[0.,0.,0.])
    self.assertEqual(config["environment"]["wind_mps"],[0.,0.,0.])
  finally:env.close()

 def test_disturbed_arm_remains_seeded_and_separate(self):
  env=CommonWaypointEnv(ROOT,bridge=NoopBridge(),disturbance_mode="seeded")
  try:
   environments=[env._config(seed)["environment"] for seed in (1,2,3)]
   self.assertTrue(any(item["current_mps"]!=[0.,0.,0.] or item["wind_mps"]!=[0.,0.,0.] for item in environments))
  finally:env.close()

if __name__=="__main__":unittest.main()
