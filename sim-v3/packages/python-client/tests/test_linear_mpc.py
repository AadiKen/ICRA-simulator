import math
import unittest

import numpy as np

from bcod_sim.linear_mpc import DELIVERED_THRUST_EFFORT_WEIGHT, LinearPlanarMPC


def truth() -> dict[str, list[float]]:
    return {
        "position_ned_m": [0., 0., 0.],
        "attitude_rad": [0., 0., 0.],
        "velocity_body_mps": [0., 0., 0.],
        "angular_rate_body_rad_s": [0., 0., 0.],
    }


class LinearMpcTest(unittest.TestCase):
    def test_first_order_lag_uses_delivered_thrust_state(self):
        controller = LinearPlanarMPC(prediction_horizon=4, position_effort_ratio=4)
        state = np.array([0., 0., 0., 0., 0., 0., 14., -7.])
        next_state = controller._step(state, np.array([70., -70.]))
        alpha = math.exp(-controller.dt_s / controller.actuator_tau_s)
        np.testing.assert_allclose(next_state[6:], alpha * state[6:] + (1 - alpha) * np.array([70., -70.]), rtol=0, atol=1e-12)
        self.assertGreater(next_state[6], state[6])
        self.assertLess(next_state[7], state[7])

    def test_solver_uses_reference_trajectory_and_respects_bounds(self):
        delivered = np.array([14., -7.])
        north = [(20. + step, 0., 0., 1.) for step in range(10)]
        east = [(0., 20. + step, math.pi / 2, 1.) for step in range(10)]
        north_controller = LinearPlanarMPC(prediction_horizon=10, position_effort_ratio=4, control_horizon=2)
        east_controller = LinearPlanarMPC(prediction_horizon=10, position_effort_ratio=4, control_horizon=2)
        north_result = north_controller.solve(truth(), north, delivered)
        east_result = east_controller.solve(truth(), east, delivered)

        self.assertFalse(north_result.fallback, north_result.status)
        self.assertFalse(east_result.fallback, east_result.status)
        self.assertTrue(np.all(np.isfinite(north_result.command)))
        self.assertTrue(np.all(np.abs(north_result.command) <= north_controller.command_limit_n))
        self.assertEqual(north_controller.previous_solution.shape, (4,))
        np.testing.assert_allclose(north_controller.delivered_thrust, delivered, rtol=0, atol=0)
        # A solver that ignores the supplied trajectory would choose the same
        # command for these orthogonal desired motions.
        self.assertGreater(np.linalg.norm(north_result.command - east_result.command), 1e-3)

    def test_solver_relinearizes_at_each_reference_horizon_point(self):
        controller = LinearPlanarMPC(prediction_horizon=4, position_effort_ratio=4)
        trajectory = [
            (10., 20., 0., .5),
            (11., 21., .1, .6),
            (12., 23., .3, .7),
            (13., 26., .6, .8),
        ]
        linearization_states: list[np.ndarray] = []
        original = controller._linearize

        def capture(state: np.ndarray):
            linearization_states.append(state.copy())
            return original(state)

        controller._linearize = capture  # type: ignore[method-assign]
        result = controller.solve(truth(), trajectory)

        self.assertFalse(result.fallback, result.status)
        self.assertEqual(len(linearization_states), controller.prediction_horizon)
        np.testing.assert_allclose(
            np.asarray([state[:4] for state in linearization_states]),
            np.asarray(trajectory), rtol=0, atol=1e-12,
        )
        self.assertGreater(linearization_states[1][5], 0.)
        self.assertGreater(linearization_states[2][5], linearization_states[1][5])

    def test_delivered_thrust_effort_weight_is_rebalanced(self):
        self.assertEqual(DELIVERED_THRUST_EFFORT_WEIGHT, 2e-5)
        self.assertLess(DELIVERED_THRUST_EFFORT_WEIGHT, .02 / 100)
