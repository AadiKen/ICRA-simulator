from pathlib import Path
import json
import math
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
sys.path.insert(0, str(ROOT / "stonefish/python"))

from bcod_sim.common_task_env import CommonWaypointEnv
from stonefish_task import draw_reset_randomization


class StonefishResetRandomizationTest(unittest.TestCase):
    def test_matches_bcod_mulberry_sequence_and_field_order(self):
        document = json.loads(
            (ROOT / "artifacts/rl-campaign/surveyor/task-contract-frozen.json").read_text()
        )
        contract = next(
            task for task in document["tasks"]
            if task["task_id"] == "common-waypoint-transit-v1"
        )
        reference = CommonWaypointEnv(ROOT, bridge=object())
        for seed in range(10000, 10005):
            start, heading, _, current, wind = reference._randomization(seed)
            draw = draw_reset_randomization(contract, seed)
            np.testing.assert_allclose(
                draw["start_ned_m"], [start[0] - 10000.0, start[1] - 10000.0], atol=1e-12
            )
            self.assertTrue(math.isclose(draw["heading_ned_rad"], heading, abs_tol=1e-12))
            np.testing.assert_allclose(draw["current_ned_mps"], current, atol=1e-12)
            np.testing.assert_allclose(draw["wind_ned_mps"], wind, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
