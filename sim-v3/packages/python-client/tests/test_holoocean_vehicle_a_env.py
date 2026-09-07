import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim.holoocean_vehicle_a_env import HoloOceanVehicleAEnv
from bcod_sim.common_task_env import CommonWaypointEnv


class _FakeHoloOcean:
    def __init__(self, scenario_cfg, show_viewport):
        self.scenario = scenario_cfg
        self.show_viewport = show_viewport
        self.current = None
        self.exited = False
        self.actions = []
        self.agents = {"vehicle_a": self}
        self.physics_states = []

    @staticmethod
    def _state():
        return {
            "GPSSensor": np.asarray([1.0, -2.0, 0.1], dtype=np.float32),
            "IMUSensor": np.asarray([[1.0, 2.0, 3.0], [0.1, 0.2, 0.3]], dtype=np.float32),
            "MagnetometerSensor": np.asarray([0.0, 1.0, 0.0], dtype=np.float32),
            "CollisionSensor": np.asarray([False]),
        }

    def reset(self):
        return self._state()

    def step(self, action, ticks):
        self.action = np.asarray(action)
        self.actions.append(self.action.copy())
        self.ticks = ticks
        return self._state()

    def set_physics_state(self, location, rotation, velocity, angular_velocity):
        self.physics_states.append(
            tuple(np.asarray(value).copy() for value in (location, rotation, velocity, angular_velocity))
        )

    def tick(self, publish=True, tick_clock=True):
        self.tick_options = (publish, tick_clock)
        return self._state()

    def set_ocean_currents(self, agent_name, current):
        self.current = (agent_name, current)

    def __exit__(self, *_args):
        self.exited = True


class HoloOceanVehicleAEnvTest(unittest.TestCase):
    def make_env(self, **kwargs):
        made = []

        def factory(**factory_kwargs):
            instance = _FakeHoloOcean(**factory_kwargs)
            made.append(instance)
            return instance

        return HoloOceanVehicleAEnv(ROOT, environment_factory=factory, **kwargs), made

    def test_policy_path_configures_only_allowed_sensor_derived_inputs_and_forces_headless(self):
        env, made = self.make_env(fixed_reset_seed=7319)
        observation, info = env.reset()
        sensor_types = [
            sensor["sensor_type"] for sensor in made[0].scenario["agents"][0]["sensors"]
        ]
        self.assertEqual(
            sensor_types,
            ["GPSSensor", "IMUSensor", "MagnetometerSensor", "CollisionSensor"],
        )
        self.assertFalse(made[0].show_viewport)
        self.assertEqual(observation.shape, (15,))
        self.assertEqual(observation.dtype, np.float32)
        self.assertEqual(info["oracle_safe_policy_sensors"], sensor_types[:3])
        self.assertEqual(info["termination_sensors"], ["CollisionSensor", "IMUSensor"])

    def test_flu_sensor_payload_maps_to_contract_frd_and_previous_action(self):
        env, _ = self.make_env(fixed_reset_seed=7319)
        initial, _ = env.reset()
        np.testing.assert_allclose(initial[:6], [1, -2, -3, .1, -.2, -.3])
        self.assertAlmostEqual(float(initial[6]), np.pi / 2)
        observation, reward, terminated, truncated, info = env.step(np.asarray([0.25, -0.5]))
        np.testing.assert_allclose(observation[10:14], [.25, -.5, 0, 0])
        self.assertAlmostEqual(reward, info["reward_components"]["shaped_reward"])
        self.assertAlmostEqual(
            reward,
            info["reward_components"]["base_reward"]
            + info["reward_components"]["potential_shaping"],
        )
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info["diagnostic_only"])

    def test_reward_port_uses_frozen_unit_gamma_and_timeout_keeps_potential(self):
        env, _ = self.make_env(fixed_reset_seed=7319)
        env.reset()
        self.assertEqual(env.shaping_gamma, 1.0)
        env.timeout_steps = env.physics_steps_per_action
        _, reward, terminated, truncated, info = env.step(np.zeros(2))
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["termination_reason"], "timeout")
        self.assertEqual(info["reward_components"]["terminal"], -10.0)
        self.assertAlmostEqual(
            reward,
            info["reward_components"]["base_reward"]
            + info["reward_components"]["potential_shaping"],
        )

    def test_seed_controls_spawn_route_current_wind_and_action_sampling(self):
        first, _ = self.make_env(fixed_reset_seed=99)
        second, _ = self.make_env(fixed_reset_seed=99)
        first.reset()
        second.reset()
        self.assertEqual(first._last_randomization, second._last_randomization)
        np.testing.assert_array_equal(first.action_space.sample(), second.action_space.sample())

    def test_reset_draws_match_bcod_mulberry_sequence_and_field_order(self):
        holo, _ = self.make_env()
        reference = CommonWaypointEnv(ROOT, bridge=object())
        for seed in range(10000, 10005):
            start, heading, route, current, wind = reference._randomization(seed)
            draw = holo._draw_randomization(seed)
            np.testing.assert_allclose(
                draw["spawn_nwu_m"][:2],
                [start[0] - 10000.0, -(start[1] - 10000.0)],
                atol=1e-12,
            )
            np.testing.assert_allclose(
                draw["route_nwu_m"],
                [[point[0] - 10000.0, -(point[1] - 10000.0)] for point in route],
                atol=1e-12,
            )
            self.assertAlmostEqual(draw["heading_ned_rad"], heading)
            np.testing.assert_allclose(
                draw["current_nwu_mps"], [current[0], -current[1], current[2]], atol=1e-12
            )
            np.testing.assert_allclose(draw["wind_ned_mps"], wind, atol=1e-12)

    def test_gps_validity_expires_when_no_new_payload_arrives(self):
        env, _ = self.make_env(fixed_reset_seed=99)
        env.reset()
        state = {"t": 0.05, "GPSSensor": np.asarray([1.0, -2.0, 0.1])}
        self.assertEqual(float(env._observation(state)[9]), 1.0)
        self.assertEqual(float(env._observation({"t": 0.20})[9]), 0.0)

    def test_yaw_is_magnetometer_derived_and_not_gyro_integrated(self):
        env, _ = self.make_env(fixed_reset_seed=99)
        env.reset()
        state = {
            "IMUSensor": np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, -9.0]]),
            "MagnetometerSensor": np.asarray([1.0, 0.0, 0.0]),
        }
        first = env._observation(state)
        second = env._observation(state)
        self.assertEqual(float(first[6]), 0.0)
        self.assertEqual(float(second[6]), 0.0)

    def test_wind_is_off_by_default_and_optional_mapping_changes_native_forces(self):
        calm, calm_made = self.make_env(fixed_reset_seed=99)
        calm.reset()
        calm.step(np.zeros(2))
        np.testing.assert_array_equal(calm_made[0].action, np.zeros(2))

        windy, windy_made = self.make_env(fixed_reset_seed=99, wind_mode="surge_equivalent")
        windy.reset()
        _, _, _, _, info = windy.step(np.zeros(2))
        self.assertNotEqual(info["wind_surge_force_n"], 0.0)
        np.testing.assert_allclose(
            windy_made[0].action,
            np.full(2, info["wind_surge_force_n"] / 2.0),
        )
        self.assertFalse(info["wind_lateral_force_supported"])

    def test_actuator_uses_exact_bcod_exponential_lag_each_physics_tick(self):
        env, made = self.make_env(fixed_reset_seed=99)
        env.reset()
        _, _, _, _, info = env.step(np.ones(2))
        alpha = 1.0 - np.exp(-env.physics_timestep_s / env.MOTOR_TIME_CONSTANT_S)
        expected_first = env.MAX_THRUST_N * alpha
        expected_second = expected_first + (env.MAX_THRUST_N - expected_first) * alpha
        np.testing.assert_allclose(made[0].actions[-2], [expected_first, expected_first])
        np.testing.assert_allclose(made[0].actions[-1], [expected_second, expected_second])
        np.testing.assert_allclose(info["actuator_lagged_force_n"], [expected_second] * 2)
        self.assertEqual(info["actuator_target_force_n"], [env.MAX_THRUST_N, env.MAX_THRUST_N])

    def test_actuator_state_resets_to_zero(self):
        env, made = self.make_env(fixed_reset_seed=99)
        env.reset()
        env.step(np.ones(2))
        self.assertGreater(float(env._thruster_state_n[0]), 0.0)
        _, info = env.reset()
        np.testing.assert_array_equal(env._thruster_state_n, np.zeros(2))
        self.assertEqual(len(made), 1)
        self.assertFalse(made[0].exited)
        self.assertEqual(info["simulator_reset_mode"], "native-reset-and-set-physics-state")
        self.assertEqual(made[0].tick_options, (False, False))
        np.testing.assert_array_equal(made[0].physics_states[-1][2], np.zeros(3))
        np.testing.assert_array_equal(made[0].physics_states[-1][3], np.zeros(3))

    def test_expected_motor_lag_is_not_an_allocation_failure(self):
        env, _ = self.make_env(fixed_reset_seed=99)
        env.reset()
        for index in range(15):
            action = np.asarray([1.0, -1.0]) if index % 2 == 0 else np.asarray([-1.0, 1.0])
            _, _, terminated, truncated, info = env.step(action)
            self.assertFalse(terminated)
            self.assertFalse(truncated)
            self.assertEqual(info["termination_reason"], "running")

    def test_settle_steps_are_outside_episode_and_precede_current(self):
        env, made = self.make_env(fixed_reset_seed=99, settle_physics_steps=3)
        observation, info = env.reset()
        self.assertEqual(info["settle_physics_steps"], 3)
        self.assertEqual(env._physics_steps, 0)
        self.assertEqual(float(observation[-1]), 1.0)
        self.assertEqual(made[0].ticks, 1)
        self.assertIsNotNone(made[0].current)


if __name__ == "__main__":
    unittest.main()
