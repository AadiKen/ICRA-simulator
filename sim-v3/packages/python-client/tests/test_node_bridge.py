import sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import PersistentNodeBridge,ShardedNodeBridge
def config(seed:int,plant:str,vehicle:str):return{"schema_version":1,"experiment":{"name":f"bridge-{plant}-{seed}","seed":seed,"timestep_s":.05,"duration_s":2},"backend":{"type":"node"},"vehicle":{"preset":vehicle,"plant":plant},"mission":{"type":"hold"}}
class BridgeTest(unittest.TestCase):
 def test_planar_and_coupled_checkpoint(self):
  with PersistentNodeBridge(ROOT) as bridge:
   bridge.reset([config(1,"planar3","vehicle-a-otter"),config(2,"coupled6","vehicle-c-azimuth")]);actions=[{"portCommand":.2,"starboardCommand":.3},{"surgeForce":10,"yawMoment":1}];first=bridge.step(actions);checkpoint=bridge.checkpoint();expected=bridge.step(actions);bridge.restore(checkpoint);actual=bridge.step(actions);self.assertEqual(expected,actual);self.assertEqual(len(first["observations"]),2);self.assertAlmostEqual(bridge.ground_truth(0)["time_s"],.1,places=11)
 def test_sharded_bridge_checkpoint(self):
  configs=[config(i,"planar3","vehicle-a-otter") for i in range(4)];actions=[{"portCommand":.2,"starboardCommand":.3} for _ in configs]
  with ShardedNodeBridge(ROOT,workers=2) as bridge:
   self.assertEqual(len(bridge.reset(configs)),4);bridge.step(actions);checkpoint=bridge.checkpoints();expected=bridge.step(actions);bridge.restore(checkpoint);self.assertEqual(bridge.step(actions),expected)
if __name__=="__main__":unittest.main()
