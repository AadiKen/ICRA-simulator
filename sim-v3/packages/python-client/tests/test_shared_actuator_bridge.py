from pathlib import Path
import unittest

import numpy as np

from bcod_sim.shared_actuator_bridge import SharedActuatorBridge


ROOT = Path(__file__).resolve().parents[3]


class SharedActuatorBridgeTest(unittest.TestCase):
    def test_lag_is_owned_by_shared_typescript_bank(self):
        bridge = SharedActuatorBridge(ROOT)
        try:
            first = bridge.step(np.ones(2), .05)
            second = bridge.step(np.ones(2), .05)
            self.assertTrue(np.all(first > 0))
            self.assertTrue(np.all(second > first))
            self.assertTrue(np.all(second < 70))
            np.testing.assert_allclose(bridge.reset(), np.zeros(2))
        finally:
            bridge.close()


if __name__ == "__main__":
    unittest.main()
