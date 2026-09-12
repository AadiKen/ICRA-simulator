import json,unittest
from figures.production import environment_validation as figure
class EnvironmentValidationTest(unittest.TestCase):
 def test_all_four_masks_are_spatial(self):
  for _,key,*_ in figure.SITES:
   cells=figure.mask_cells(figure.mask_path(key));self.assertGreater(len(cells),20);self.assertTrue({c["rtofs_mask"] for c in cells}.issubset({"water","land"}))
 def test_current_metrics_resolve_discrepancies(self):
  rows=json.loads(figure.REPORT.read_text())["currents"]["matches"];s=[r for r in rows if r["zone"]=="shelf"];n=[r for r in rows if r["zone"]=="nearshore"]
  self.assertAlmostEqual(figure.speed_rmse(s),.10462501406947745);self.assertAlmostEqual(figure.speed_rmse(n),.19776117526431267);self.assertEqual(sum(r["reference_speed_mps"]>=.75 for r in rows),0)
if __name__=="__main__":unittest.main()
