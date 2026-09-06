import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/python-client"))
SCRIPT = ROOT / "validation/rl-campaign/run_task5_conditions.py"
SPEC = importlib.util.spec_from_file_location("train_portable_ppo_task5", SCRIPT)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class Task5ConditionTest(unittest.TestCase):
    def test_condition_configuration_is_isolated_and_outputs_are_separate(self):
        environments = [
            RUNNER.Task5ConditionEnv(ROOT, condition=name, fixed_reset_seed=30000,
                                     final_leg_curriculum=True, bridge=object())
            for name in RUNNER.CONDITIONS
        ]
        configs = [environment._config(30000) for environment in environments]
        noise_imu = configs[1]["sensors"][0]["config"]
        self.assertTrue(all(value == 0.0 for value in noise_imu.values()))
        self.assertNotIn("config", configs[0]["sensors"][0])
        self.assertNotIn("config", configs[2]["sensors"][0])
        noise_imu["gyro_noise_std_rad_s"] = 123.0
        self.assertNotIn("config", configs[2]["sensors"][0])
        outputs = {RUNNER.output_directory(name, 7319) for name in RUNNER.CONDITIONS}
        self.assertEqual(len(outputs), 3)
        replicated = {RUNNER.output_directory(name, seed)
                      for name in RUNNER.CONDITIONS for seed in (7320, 7321)}
        self.assertEqual(len(replicated), 6)
        self.assertTrue(outputs.isdisjoint(replicated))

    def test_control_masks_only_yaw_across_steps_and_seeds(self):
        observations = []
        for seed in (30000, 30001):
            env = RUNNER.Task5ConditionEnv(
                ROOT, condition="control", fixed_reset_seed=seed,
                final_leg_curriculum=True)
            try:
                observation, _ = env.reset()
                observations.append(observation.copy())
                for _ in range(12):
                    observation, _, terminated, truncated, _ = env.step(
                        np.asarray([.15, .2], dtype=np.float32))
                    observations.append(observation.copy())
                    if terminated or truncated:
                        break
            finally:
                env.close()
        matrix = np.asarray(observations)
        self.assertEqual(matrix.shape[1], 15)
        np.testing.assert_array_equal(matrix[:, 6], np.zeros(len(matrix)))
        self.assertGreater(np.ptp(matrix[:, :6], axis=0).max(), 0.0)
        self.assertGreater(np.ptp(matrix[:, 7:], axis=0).max(), 0.0)

    def test_noise_zero_samples_match_same_timestamp_truth(self):
        env = RUNNER.Task5ConditionEnv(
            ROOT, condition="noise-zero", fixed_reset_seed=30000,
            final_leg_curriculum=True)
        truth_by_time = {}
        checked = {"imu": 0, "gps": 0}
        seen_timestamps = {"imu": set(), "gps": set()}
        try:
            reset = env.bridge.reset([env._config(30000)])
            truth = env.bridge.ground_truth()
            truth_by_time[round(float(truth["time_s"]), 9)] = truth
            observation = reset["observations"][0]
            command = {"active_sensors": ["imu", "gps"], "actuators": {
                "effectors": {"port": {"command": .15},
                              "starboard": {"command": .2}}}}
            for _ in range(30):
                result = env.bridge.step([command])
                observation = result["observations"][0]
                truth = env.bridge.ground_truth()
                truth_by_time[round(float(truth["time_s"]), 9)] = truth
                for name in ("imu", "gps"):
                    sample = observation.get("sensors", {}).get(name)
                    if not sample or not sample.get("valid") or sample.get("payload") is None:
                        continue
                    timestamp = round(float(sample["timestampS"]), 9)
                    if timestamp in seen_timestamps[name] or timestamp not in truth_by_time:
                        continue
                    seen_timestamps[name].add(timestamp)
                    sampled_truth = truth_by_time[timestamp]
                    payload = sample["payload"]
                    if name == "imu":
                        np.testing.assert_allclose(payload["acceleration_body_mps2"],
                                                   sampled_truth["acceleration_body_mps2"], atol=1e-6, rtol=0)
                        np.testing.assert_allclose(payload["angular_rate_body_rad_s"],
                                                   sampled_truth["angular_rate_body_rad_s"], atol=1e-6, rtol=0)
                        np.testing.assert_allclose(payload["orientation_rad"],
                                                   sampled_truth["attitude_rad"], atol=1e-6, rtol=0)
                        np.testing.assert_allclose(payload["accel_bias_mps2"], [0, 0, 0], atol=1e-12, rtol=0)
                        np.testing.assert_allclose(payload["gyro_bias_rad_s"], [0, 0, 0], atol=1e-12, rtol=0)
                    else:
                        np.testing.assert_allclose(payload["position_ned_m"],
                                                   sampled_truth["position_ned_m"], atol=1e-6, rtol=0)
                        np.testing.assert_allclose(payload["velocity_ned_mps"],
                                                   sampled_truth["velocity_ned_mps"], atol=1e-6, rtol=0)
                    checked[name] += 1
            self.assertGreater(checked["imu"], 0)
            self.assertGreater(checked["gps"], 0)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
