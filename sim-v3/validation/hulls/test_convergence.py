import json
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]

class ConvergenceArtifactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.artifact=json.loads((ROOT/"validation/hulls/capytaine-convergence.json").read_text())

    def test_full_density_sweep_and_residual(self):
        self.assertEqual(self.artifact["chosen_density"],"p6400")
        self.assertFalse(self.artifact["is_validation_evidence"])
        self.assertTrue(self.artifact["validation_status_unchanged"])
        for vehicle in self.artifact["vehicles"].values():
            self.assertEqual([row["density"] for row in vehicle["densities"]],["p0800","p1600","p3200","p6400"])
            for residual in vehicle["chosen_density_residual"]:
                self.assertLess(residual["added_mass_relative_change"],0.01)
                self.assertLess(residual["radiation_damping_relative_change"],0.01)

    def test_capytaine_gm_is_comparison_not_overwrite(self):
        for vehicle in self.artifact["vehicles"].values():
            comparison=vehicle["hydrostatics_no_overwrite"]
            self.assertEqual(comparison["resolution_policy"],"report-both-no-automatic-overwrite")
            for key in ("GM_T_m","GM_L_m"):
                self.assertLess(abs(comparison["signed_delta"][key])/comparison["analytic"][key],0.03)
        self.assertAlmostEqual(self.artifact["vehicles"]["vehicle-c-azimuth"]["hydrostatics_no_overwrite"]["analytic"]["GM_T_m"],3.7232)

if __name__=="__main__": unittest.main()
