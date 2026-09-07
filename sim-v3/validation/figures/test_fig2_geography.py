from __future__ import annotations
import copy,json,unittest
from validation.figures.common import ROOT
from validation.figures.fig2_geography import validate_inputs
class GeographyValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        base=ROOT/"artifacts/environment-coverage";cls.values=[json.loads((base/name).read_text()) for name in ("coverage-matrix.json","live-confirmatory-pass.json","gebco-live-confirmatory-pass.json","real-vs-idealized-trajectory.json")]
    def test_shapes_are_valid(self): validate_inputs(*self.values)
    def test_missing_coordinates_fail(self):
        values=copy.deepcopy(self.values);del values[0]["methodology"]["sites"][0]["latitude_deg"]
        with self.assertRaisesRegex(ValueError,"latitude/longitude"): validate_inputs(*values)
    def test_uncontrolled_pair_fails(self):
        values=copy.deepcopy(self.values);values[3]["scenario"]["invariant_between_runs"]="unknown"
        with self.assertRaisesRegex(ValueError,"controlled comparison"): validate_inputs(*values)
if __name__=="__main__": unittest.main()
