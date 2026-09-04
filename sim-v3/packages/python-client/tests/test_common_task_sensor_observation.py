import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim import CommonWaypointEnv


class _NoTruthAccess:
    def __getitem__(self, _key):
        raise AssertionError("the policy observation path read privileged ground truth")


def _bare_env(sensor_time=1.0):
    env = object.__new__(CommonWaypointEnv)
    env.last_truth = _NoTruthAccess()
    env.last_observation = {"time_s": sensor_time, "sensors": {}}
    env.route = [[10.0, 20.0]]
    env.waypoint = 0
    env.prev_action = np.array([0.25, -0.5])
    env.timeout_steps = 100
    env.steps = 25
    env.physics_timestep_s = 0.05
    return env


class CommonTaskSensorObservationTest(unittest.TestCase):
    def test_observation_path_cannot_read_ground_truth(self):
        env = _bare_env()
        observation = env._obs()
        self.assertEqual(observation.shape, (17,))
        self.assertEqual(observation.dtype, np.float32)
        np.testing.assert_array_equal(observation[:12], np.zeros(12, dtype=np.float32))

    def test_fresh_samples_preserve_field_order_shape_and_dtype(self):
        env = _bare_env()
        env.last_observation["sensors"] = {
            "imu": {"valid": True, "timestampS": 0.95, "payload": {
                "acceleration_body_mps2": [1, 2, 3],
                "angular_rate_body_rad_s": [4, 5, 6],
                "orientation_rad": [.1, -.2, .3]}},
            "gps": {"valid": True, "timestampS": 0.5, "payload": {
                "position_ned_m": [7, 8, 0],
                "velocity_ned_mps": [9, 10, 0]}},
        }
        observation = env._obs()
        expected = np.asarray([1, 2, 3, 4, 5, 6, .3, 3, 12, 9, 10, 1,
                               .25, -.5, 0, 0, .75], dtype=np.float32)
        self.assertEqual(observation.shape, (17,))
        self.assertEqual(observation.dtype, np.float32)
        np.testing.assert_array_equal(observation, expected)

    def test_stale_and_invalid_samples_are_zero_filled(self):
        env = _bare_env()
        env.last_observation["sensors"] = {
            "imu": {"valid": True, "timestampS": 0.8, "payload": {
                "acceleration_body_mps2": [1, 2, 3],
                "angular_rate_body_rad_s": [4, 5, 6],
                "orientation_rad": [.1, -.2, .3]}},
            "gps": {"valid": False, "timestampS": 1.0, "payload": None},
        }
        observation = env._obs()
        np.testing.assert_array_equal(observation[:12], np.zeros(12, dtype=np.float32))

    def test_closed_loop_sensor_smoke_and_no_gps_dropout(self):
        env = CommonWaypointEnv(ROOT, fixed_reset_seed=30000, final_leg_curriculum=True)
        try:
            config = env._config(30000)
            self.assertEqual(config["sensors"], [
                {"plugin": "imu", "enabled": True},
                {"plugin": "gps", "enabled": True},
            ])
            observation, _ = env.reset()
            self.assertEqual(observation.shape, (17,))
            self.assertEqual(observation.dtype, np.float32)
            self.assertEqual(float(observation[11]), 0.0)
            seen_imu = seen_gps = False
            gps_timestamps = set()
            imu_acceleration_errors = []
            imu_rate_errors = []
            gps_position_errors = []
            gps_velocity_errors = []
            for _ in range(20):
                observation, reward, terminated, truncated, _ = env.step(np.array([.15, .2]))
                self.assertTrue(np.all(np.isfinite(observation)))
                self.assertTrue(np.isfinite(reward))
                sensors = env.last_observation.get("sensors", {})
                imu, gps = sensors.get("imu"), sensors.get("gps")
                seen_imu = seen_imu or bool(imu and imu.get("valid"))
                if env._fresh_sensor("imu") is not None:
                    imu_acceleration_errors.extend(np.abs(
                        observation[:3] - np.asarray(env.last_truth["acceleration_body_mps2"][:3])))
                    imu_rate_errors.extend(np.abs(
                        observation[3:6] - np.asarray(env.last_truth["angular_rate_body_rad_s"][:3])))
                if gps and gps.get("timestampS") not in gps_timestamps:
                    gps_timestamps.add(gps.get("timestampS"))
                    self.assertTrue(gps.get("valid"))
                    self.assertIsNotNone(gps.get("payload"))
                    seen_gps = True
                if env._fresh_sensor("gps") is not None:
                    target = env.route[env.waypoint]
                    truth_position = env.last_truth["position_ned_m"]
                    truth_relative_goal = np.asarray([
                        target[0] - truth_position[0], target[1] - truth_position[1]])
                    gps_position_errors.extend(np.abs(observation[7:9] - truth_relative_goal))
                    gps_velocity_errors.extend(np.abs(
                        observation[9:11] - np.asarray(env.last_truth["velocity_ned_mps"][:2])))
                if terminated or truncated:
                    break
            self.assertTrue(seen_imu)
            self.assertTrue(seen_gps)
            # Bounds include sensor noise plus the declared delivery latency.
            self.assertLess(max(imu_acceleration_errors), 1.0)
            self.assertLess(max(imu_rate_errors), .2)
            self.assertLess(max(gps_position_errors), 5.0)
            self.assertLess(max(gps_velocity_errors), 1.0)
            metrics = env.bridge.metrics()
            self.assertGreater(metrics["total_sensor_cost"], 0)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
