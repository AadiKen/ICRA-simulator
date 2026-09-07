from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "bcod_sim"))
from common_task import CompletionTracker, compute_reward, full_route_length_m  # noqa: E402


BACKENDS = ("bcod-sim", "gazebo-harmonic", "vrx", "holoocean", "stonefish")


class CompletionFractionTest(unittest.TestCase):
    def test_one_of_two_scripted_legs_is_half_complete_in_every_harness(self):
        start = [0.0, 0.0]
        route = [[10.0, 0.0], [20.0, 0.0]]
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                tracker = CompletionTracker.for_route(start, route)
                scored = compute_reward(10.0, 0.0, 0.0, np.zeros(2), np.zeros(2))
                self.assertEqual(tracker.update(scored.progress_reward), 0.5)

    def test_full_route_includes_start_to_first_waypoint(self):
        self.assertEqual(
            full_route_length_m([0.0, 0.0], [[3.0, 4.0], [9.0, 12.0]]),
            15.0,
        )

    def test_reverse_motion_reduces_signed_progress_and_fraction_is_clamped(self):
        tracker = CompletionTracker.for_route([0.0, 0.0], [[10.0, 0.0]])
        forward = compute_reward(10.0, 5.0, 0.0, np.zeros(2), np.zeros(2))
        reverse = compute_reward(5.0, 8.0, 0.0, np.zeros(2), np.zeros(2))
        self.assertEqual(tracker.update(forward.progress_reward), 0.5)
        self.assertEqual(tracker.update(reverse.progress_reward), 0.2)
        self.assertEqual(tracker.update(-1000.0), 0.0)


if __name__ == "__main__":
    unittest.main()
