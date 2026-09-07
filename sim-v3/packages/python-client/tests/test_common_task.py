import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim.common_task import (
    classify_termination,
    compute_reward,
    cross_track_distance,
    passed_waypoint_plane,
)


class CommonTaskTest(unittest.TestCase):
    def test_reward_matches_frozen_formula(self):
        result = compute_reward(10, 8.5, 2, np.array([.5, -.5]), np.zeros(2))
        self.assertAlmostEqual(result.progress_reward, 3)
        self.assertAlmostEqual(result.cross_track_penalty, -.04)
        self.assertAlmostEqual(result.action_delta_penalty, -.025)
        self.assertAlmostEqual(result.reward, 2.935)

    def test_terminal_precedence_and_rewards(self):
        self.assertEqual(classify_termination(success=True, collision_type="grounding", timed_out=True), "success")
        self.assertEqual(classify_termination(success=False, collision_type="grounding", allocation_failed=True), "grounding")
        self.assertEqual(classify_termination(success=False, allocation_failed=True, timed_out=True), "allocation_failure")
        self.assertEqual(compute_reward(0, 0, 0, np.zeros(2), np.zeros(2), "allocation_failure").terminal_reward, -25)

    def test_geometry_helpers(self):
        self.assertAlmostEqual(cross_track_distance([5, 2], [0, 0], [10, 0]), 2)
        self.assertTrue(passed_waypoint_plane([11, 2], [0, 0], [10, 0]))
        self.assertFalse(passed_waypoint_plane([11, 20], [0, 0], [10, 0]))

    def test_potential_shaping_zeroes_terminal_potential(self):
        result = compute_reward(4, 3, 0, np.zeros(2), np.zeros(2), "success",
                                previous_final_distance_m=4, final_distance_m=3,
                                shaping_k=.2, shaping_gamma=.99, shaping_enabled=True)
        self.assertAlmostEqual(result.potential_shaping, .8)

    def test_undiscounted_shaping_is_pure_progress(self):
        stationary = compute_reward(4, 4, 0, np.zeros(2), np.zeros(2),
                                    previous_final_distance_m=4, final_distance_m=4,
                                    shaping_k=.2, shaping_gamma=1, shaping_enabled=True)
        retreat = compute_reward(4, 5, 0, np.zeros(2), np.zeros(2),
                                 previous_final_distance_m=4, final_distance_m=5,
                                 shaping_k=.2, shaping_gamma=1, shaping_enabled=True)
        self.assertAlmostEqual(stationary.potential_shaping, 0)
        self.assertAlmostEqual(retreat.potential_shaping, -.2)

    def test_timeout_keeps_real_next_potential(self):
        result = compute_reward(4, 5, 0, np.zeros(2), np.zeros(2), "timeout",
                                previous_final_distance_m=4, final_distance_m=5,
                                shaping_k=.2, shaping_gamma=1, shaping_enabled=True)
        self.assertAlmostEqual(result.potential_shaping, -.2)


if __name__ == "__main__":
    unittest.main()
