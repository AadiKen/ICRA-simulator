from __future__ import annotations
import math
import unittest
import numpy as np

from sensordemo.ablations import blind_observation
from sensordemo.config import SensorDemoConfig
from sensordemo.obs_contract import SENSOR_DEMO_OBS_CONTRACT_HASH, ObservationStack, assemble_frame
from sensordemo.reward import compute_reward
from sensordemo.sensors import AISReceiver, Detection, EgoPose, RadarSensor, SceneObject, polygon_line_of_sight
from sensordemo.tracker import TrackManager
from sensordemo.train_sensor_demo_policy import _duration
from sensordemo.waypoint_sources import VesselState, relative_waypoint_to_ned


class SensorDemoTests(unittest.TestCase):
    def setUp(self):
        self.config = SensorDemoConfig(radar_dropout_probability=0, radar_false_positive_rate_per_scan=0)
        self.ego = EgoPose(0, 0, 0)

    def test_radar_range_and_occlusion(self):
        radar = RadarSensor(self.config, np.random.default_rng(4))
        objects = [SceneObject("near", 100, 0), SceneObject("far", 400, 0)]
        self.assertEqual([d.object_id for d in radar.scan(self.ego, objects, 0)], ["near"])
        blocked = polygon_line_of_sight([[(50, -10), (50, 10), (60, 10), (60, -10)]])
        self.assertEqual(radar.scan(self.ego, objects, 0, blocked), [])

    def test_ais_only_reports_broadcasting_vessels_and_is_stale(self):
        ais = AISReceiver(self.config, np.random.default_rng(5))
        objects = [SceneObject("yes", 1000, 0, 2, 0, ais_broadcasting=True), SceneObject("no", 100, 0, ais_broadcasting=False), SceneObject("rock", 50, 0, kind="static", ais_broadcasting=True)]
        result = ais.scan(self.ego, objects, 20)
        self.assertEqual([d.object_id for d in result], ["yes"])
        self.assertLessEqual(result[0].timestamp_s, 20)
        self.assertEqual(ais.scan(self.ego, objects, 20.1), [])

    def test_tracker_fuses_sources_and_coasts(self):
        manager = TrackManager(self.config)
        manager.update([Detection("radar", 0, 10, 0, "ship")], 0)
        tracks = manager.update([Detection("ais", 1, 11, 0, "ship", 1, 0)], 1)
        self.assertEqual(len(tracks), 1); self.assertTrue(tracks[0].source_radar); self.assertTrue(tracks[0].source_ais)
        self.assertEqual(len(manager.update([], 2)), 1)
        self.assertEqual(manager.update([], 32), [])

    def test_observation_contract_and_padding(self):
        frame = assemble_frame(config=self.config, ego=self.ego, velocity_body_mps=(0, 0), yaw_rate_rad_s=0, speed_over_ground_mps=0, goal_ne_m=(100, 0), current_ne_mps=(0, 0), wave_height_m=0, wave_period_s=0, wave_direction_rad=0, tracks=[], now_s=0, depth_under_keel_m=10)
        self.assertEqual(frame.shape, (80,)); self.assertEqual(frame[15], 1)
        stack = ObservationStack(self.config); observation = stack.reset(frame); self.assertEqual(observation.shape, (320,)); self.assertEqual(len(SENSOR_DEMO_OBS_CONTRACT_HASH), 64)
        blind = blind_observation(observation); self.assertTrue(np.all(blind[9:79] == 0)); self.assertEqual(blind[6], observation[6])

    def test_relative_waypoint_uses_ned_heading(self):
        self.assertTrue(np.allclose(relative_waypoint_to_ned(VesselState(2, 3, math.pi/2), 10, 0), (2, 13)))

    def test_reward_uses_truth_inputs(self):
        terms = compute_reward(self.config, previous_goal_distance_m=100, goal_distance_m=90, collision=False, true_cpa_m=15, bearing_action=.5, previous_bearing_action=0, integrated_thrust_ns=100, thrust_normalizer_ns=1000, reached_goal=False)
        self.assertGreater(terms.proximity, 0); self.assertGreater(terms.progress, 0); self.assertLess(terms.total, terms.progress)

    def test_eta_duration_format(self):
        self.assertEqual(_duration(3661.9), "01:01:01")
        self.assertEqual(_duration(float("inf")), "--:--:--")


if __name__ == "__main__": unittest.main()
