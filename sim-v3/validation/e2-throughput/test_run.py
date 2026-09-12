import pathlib,sys,unittest
ROOT=pathlib.Path(__file__).resolve().parents[2];sys.path.insert(0,str(pathlib.Path(__file__).parent))
from run import measure

class PilotTests(unittest.TestCase):
 def test_estimate_is_derived_from_sample(self):
  row=measure('cpu',2,3,1,30,13)
  self.assertEqual(row['sample_aggregate_environment_steps'],6)
  self.assertAlmostEqual(row['estimated_wall_clock_per_rollout_s'],row['measured_wall_clock_s']*10)
  self.assertEqual(row['measurement_class'],'heuristic-pilot-extrapolation')
  self.assertIsNone(row['bridge_overhead_ms'])

if __name__=='__main__': unittest.main()
