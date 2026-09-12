import json,tempfile,unittest
from pathlib import Path
from figures.production import heterogeneous_vehicle_validation as figure

class HeterogeneousVehicleValidationTest(unittest.TestCase):
 def test_authoritative_sources_and_maneuvers(self):
  a=figure.load(figure.VEHICLE_A);c=figure.load(figure.VEHICLE_C)
  self.assertEqual([s["id"] for s in a["scenarios"]],["constant-thrust","turning-circle","yaw-turn","zig-zag","coast-down","current-drift"])
  self.assertEqual([s["id"] for s in c["scenarios"]],["straight-ahead","pure-lateral","rotation-in-place","allocation-chirp"])
 def test_figure_and_provenance_build(self):
  with tempfile.TemporaryDirectory() as d:
   out=figure.build(output=Path(d)/"figure.png");self.assertGreater(out.stat().st_size,50_000);self.assertTrue(out.with_suffix(".svg").is_file())
   prov=json.loads(out.with_suffix(".provenance.json").read_text());self.assertEqual(len(prov["sources"]),4);self.assertTrue(prov["render_decisions"]["expanded_battery_worst_callout"]["enabled"]);self.assertEqual(len(prov["render_decisions"]["aligned_maneuvers"]),5);self.assertEqual(prov["rotation_in_place_diagnostic"]["classification"],"real cross-model dynamics difference; root cause not isolated");self.assertFalse(prov["rotation_in_place_diagnostic"]["shared_actuator_pairing_root_cause_ruled_out"]);self.assertIn("zig-zag",prov["vehicle_c_mixed_provenance"]["physically_rerun"])
if __name__=="__main__":unittest.main()
