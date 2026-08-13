import json
import pathlib
import subprocess
import unittest

ROOT=pathlib.Path(__file__).parents[2]

class IkedaTests(unittest.TestCase):
    def test_lower_bound(self):
        subprocess.run([str(ROOT/".venv/bin/python"),str(ROOT/"validation/ikeda/run.py")],check=True,cwd=ROOT,stdout=subprocess.DEVNULL)
        artifact=json.loads((ROOT/"artifacts/ikeda/roll-damping-lower-bound.json").read_text())
        self.assertEqual(artifact["status"],"software-estimate-not-physical-validation")
        self.assertFalse(artifact["assumptions"]["radiation_double_counted"])
        for vehicle in artifact["vehicles"].values():
            components=vehicle["components_Nms_per_rad"]
            self.assertGreater(components["friction_equivalent_linear"],0)
            self.assertGreater(components["eddy_equivalent_linear"],0)
            self.assertGreater(components["potential_radiation"],0)
            self.assertEqual(components["bilge_keel"],0)
            self.assertEqual(components["lift"],0)
            self.assertGreater(vehicle["viscous_quadratic_roll_Nms2_per_rad2"],0)

if __name__=="__main__": unittest.main()
