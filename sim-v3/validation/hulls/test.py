import json
import subprocess
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]

class HullGeneratorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run([str(ROOT/".venv/bin/python"),str(ROOT/"validation/hulls/generate.py")],check=True,cwd=ROOT)
        cls.report=json.loads((ROOT/"validation/hulls/mesh-verification.json").read_text())

    def test_geometry_and_frames(self):
        self.assertIn("midship/centreline/DWL",self.report["frame_assertion"])
        for vehicle in self.report["vehicles"].values():
            for density in vehicle["densities"].values():
                self.assertTrue(density["quality"]["watertight"])
                self.assertTrue(density["quality"]["outward_normals"])
                self.assertGreater(density["quality"]["minimum_panel_area_m2"],0)
            self.assertTrue(all(abs(value)<0.03 for value in vehicle["finest_relative_error"].values()))

    def test_catamaran_gm_is_intentionally_large(self):
        cat=self.report["vehicles"]["vehicle-c-azimuth"]
        self.assertAlmostEqual(cat["analytic"]["GM_T_m"],3.7232,delta=0.02)

if __name__=="__main__": unittest.main()
