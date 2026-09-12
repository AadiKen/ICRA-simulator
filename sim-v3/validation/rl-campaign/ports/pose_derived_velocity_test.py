#!/usr/bin/env python3
import math
import unittest

from pose_derived_velocity import PlanarPose, PoseDerivedPlanarVelocity


class PoseDerivedVelocityTest(unittest.TestCase):
    def test_body_velocity_uses_endpoint_heading(self):
        estimator = PoseDerivedPlanarVelocity()
        self.assertEqual(estimator.update(PlanarPose(0, 0, 0, math.pi / 2)), (0, 0, 0))
        u, v, r = estimator.update(PlanarPose(.05, 0, .1, math.pi / 2))
        self.assertAlmostEqual(u, 2)
        self.assertAlmostEqual(v, 0)
        self.assertAlmostEqual(r, 0)

    def test_yaw_delta_wraps(self):
        estimator = PoseDerivedPlanarVelocity()
        estimator.update(PlanarPose(0, 0, 0, math.pi - .01))
        _, _, r = estimator.update(PlanarPose(.05, 0, 0, -math.pi + .01))
        self.assertAlmostEqual(r, .4)


if __name__ == "__main__":
    unittest.main()
