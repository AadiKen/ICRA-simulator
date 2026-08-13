import json,pathlib,sys,unittest
sys.path.insert(0,str(pathlib.Path(__file__).parent));from run import run
class BaselineTests(unittest.TestCase):
 def test_deterministic_baselines(self):
  root=pathlib.Path(__file__).parents[2]/"artifacts/baselines";first=run(root);second=run(root);self.assertEqual(first,second);self.assertFalse(first["training"]["algorithmic_novelty_claim"]);self.assertEqual(first["training"]["algorithm"],"PPO");self.assertTrue((root/"checkpoints/ppo-cpu.zip").exists());self.assertTrue((root/"checkpoints/ppo-actor.json").exists());self.assertTrue(all(r["finite"] for values in first["policies"].values() for r in values))
if __name__=="__main__":unittest.main()
