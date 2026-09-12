import tempfile,unittest
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from figures.production import fig3_environmental_sensing as figure

class Figure3SensingTest(unittest.TestCase):
 def test_production_payloads_and_domain_contrast(self):
  data=figure.load(figure.DEFAULT);rows={x["condition"]:x for x in data["weather"]}
  self.assertNotIn("config",data);self.assertEqual(data["seed"],7319)
  self.assertEqual(len(rows["clear"]["camera"]["payload"]["rgba"]),8*6*4)
  self.assertEqual(len(rows["clear"]["camera"]["diagnostic_payload"]["rgba"]),80*60*4)
  self.assertEqual(len(rows["clear"]["lidar"]["payload"]["ranges_m"]),16)
  self.assertEqual(len(rows["clear"]["radar"]["beams"]),12)
  self.assertEqual(rows["clear"]["radar"]["detected_count"],rows["fog"]["radar"]["detected_count"])
  self.assertLess(rows["fog"]["camera"]["transmission_at_target"],rows["clear"]["camera"]["transmission_at_target"]*.5)
  self.assertLess(rows["fog"]["lidar"]["effective_range_scale"],.5)
  self.assertGreater(rows["rain"]["radar"]["clutter_db"],rows["clear"]["radar"]["clutter_db"])
 def test_rain_radar_seed_audit_is_recorded(self):
  audit=figure.load(figure.DEFAULT)["radar_seed_audit"]
  self.assertEqual(audit["scan_count"],1000);self.assertEqual(sum(audit["histogram"].values()),1000)
  self.assertGreater(audit["zero_hit_fraction"],.5);self.assertLess(audit["zero_hit_fraction"],.8)
  self.assertGreater(audit["max_hits"],0);self.assertLess(audit["mean_hits"],1)
 def test_figure_builds(self):
  with tempfile.TemporaryDirectory() as d:
   out=figure.build(figure.DEFAULT,Path(d)/"fig.png");self.assertGreater(out.stat().st_size,50_000);self.assertTrue(out.with_suffix(".svg").is_file());self.assertTrue(out.with_suffix(".provenance.json").is_file());self.assertTrue(out.with_suffix(".stats.json").is_file())
 def test_display_exposure_is_uniform_and_raw_data_unchanged(self):
  before=figure.DEFAULT.read_bytes()
  with tempfile.TemporaryDirectory() as d:figure.build(figure.DEFAULT,Path(d)/"fig.png")
  self.assertEqual(before,figure.DEFAULT.read_bytes());self.assertEqual(figure.EXPOSURE_STOPS,2.0)
 def test_raw_camera_frames_have_scene_structure(self):
  rows=figure.load(figure.DEFAULT)["weather"]
  variances=[];unique_colors=[]
  for row in rows:
   p=row["camera"]["payload"];rgb=np.asarray(p["rgba"]).reshape(p["height"],p["width"],4)[:,:,:3];variances.append(float(rgb.var()));unique_colors.append(len(np.unique(rgb.reshape(-1,3),axis=0)))
  self.assertTrue(all(v>1 for v in variances),variances)
  self.assertTrue(all(n>=8 for n in unique_colors),unique_colors)
 def test_camera_statistics_remain_tied_to_policy_observation(self):
  for row in figure.load(figure.DEFAULT)["weather"]:
   camera=row["camera"];p=camera["payload"];red=np.asarray(p["rgba"]).reshape(6,8,4)[:,:,0]
   self.assertAlmostEqual(camera["mean_intensity_0_255"],float(red.mean()))
   self.assertEqual((p["width"],p["height"]),(8,6));self.assertEqual((camera["diagnostic_payload"]["width"],camera["diagnostic_payload"]["height"]),(80,60))
 def test_each_sensor_panel_draws_its_own_reported_marker_count(self):
  data=figure.load(figure.DEFAULT);audit={**data["radar_seed_audit"],"display_seed":data["seed"]}
  fig=plt.figure();grid=fig.add_gridspec(3,2)
  try:
   for i,row in enumerate(data["weather"]):
    lidar=figure.lidar_cell(fig,grid[i,0],row);radar=figure.radar_cell(fig,grid[i,1],row,audit)
    lidar_marks=[c for c in lidar.collections if c.get_gid()==f"lidar-markers-{row['condition']}"]
    radar_marks=[c for c in radar.collections if c.get_gid()==f"radar-markers-{row['condition']}"]
    self.assertEqual(sum(len(c.get_offsets()) for c in lidar_marks),row["lidar"]["detected_count"])
    self.assertEqual(sum(len(c.get_offsets()) for c in radar_marks),row["radar"]["detected_count"])
  finally:plt.close(fig)

if __name__=="__main__":unittest.main()
