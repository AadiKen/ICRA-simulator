import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim.holoocean_termination import HoloOceanTerminationMonitor


class HoloOceanTerminationMonitorTest(unittest.TestCase):
    def update(self, monitor, **overrides):
        values = {
            "dt_s": 0.1,
            "acceleration_body_frd_mps2": [0.0, 0.0, -9.81],
            "colliding": False,
            "commanded_thrust_n": [100.0, 100.0],
            "applied_thrust_n": [100.0, 100.0],
        }
        values.update(overrides)
        return monitor.update(**values)

    def test_native_collision_is_generic_because_counterpart_is_unavailable(self):
        self.assertEqual(
            self.update(HoloOceanTerminationMonitor(), colliding=True), "collision"
        )

    def test_instability_requires_one_simulated_second(self):
        monitor = HoloOceanTerminationMonitor()
        tilted = [9.81 * math.sin(math.radians(61)), 0.0, -9.81 * math.cos(math.radians(61))]
        for _ in range(9):
            self.assertIsNone(
                self.update(monitor, acceleration_body_frd_mps2=tilted)
            )
        self.assertEqual(
            self.update(monitor, acceleration_body_frd_mps2=tilted), "instability"
        )

    def test_nonfinite_imu_is_immediate(self):
        self.assertEqual(
            self.update(
                HoloOceanTerminationMonitor(),
                acceleration_body_frd_mps2=[math.nan, 0.0, -9.81],
            ),
            "instability",
        )

    def test_allocation_failure_requires_one_simulated_second(self):
        monitor = HoloOceanTerminationMonitor()
        for _ in range(9):
            self.assertIsNone(
                self.update(monitor, applied_thrust_n=[0.0, 0.0])
            )
        self.assertEqual(
            self.update(monitor, applied_thrust_n=[0.0, 0.0]),
            "allocation_failure",
        )

    def test_instability_precedes_collision_and_allocation(self):
        monitor = HoloOceanTerminationMonitor()
        self.assertEqual(
            self.update(
                monitor,
                acceleration_body_frd_mps2=[math.nan, 0.0, 0.0],
                colliding=True,
                applied_thrust_n=[0.0, 0.0],
            ),
            "instability",
        )


if __name__ == "__main__":
    unittest.main()
