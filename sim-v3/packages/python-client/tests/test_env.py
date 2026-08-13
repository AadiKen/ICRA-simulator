import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))
from bcod_sim.env import BCODGymEnv, BCODVectorEnv, EnvConfig


class EnvironmentTests(unittest.TestCase):
    def test_checkpoint_independent_observation_and_determinism(self):
        config = EnvConfig(timestep_s=0.1, duration_s=0.3, seed=42)
        first, second = BCODGymEnv(config), BCODGymEnv(config)
        obs_a, _ = first.reset()
        obs_b, _ = second.reset()
        obs_a[0] = 99
        self.assertEqual(first._state[0], 0)
        for _ in range(3):
            obs_a, _, _, truncated_a, _ = first.step(np.ones(6, dtype=np.float32))
            obs_b, _, _, truncated_b, _ = second.step(np.ones(6, dtype=np.float32))
            np.testing.assert_array_equal(obs_a, obs_b)
        self.assertTrue(truncated_a and truncated_b)

    def test_vector_shape(self):
        env = BCODVectorEnv(4)
        observations, _ = env.reset(seed=3)
        self.assertEqual(observations.shape, (4, 12))

    def test_checkpoint_and_oracle_separation(self):
        env = BCODGymEnv(EnvConfig(timestep_s=.1, duration_s=1, seed=7))
        env.reset(); _, _, _, _, info = env.step(np.ones(6, dtype=np.float32))
        self.assertNotIn("ground_truth", info)
        checkpoint = env.save_checkpoint(); expected = env.step(np.ones(6, dtype=np.float32))
        env.load_checkpoint(checkpoint); actual = env.step(np.ones(6, dtype=np.float32))
        np.testing.assert_array_equal(expected[0], actual[0])
        oracle = BCODGymEnv(EnvConfig(oracle_access=True)); oracle.reset(); _, _, _, _, oracle_info = oracle.step(np.zeros(6, dtype=np.float32)); self.assertIn("ground_truth", oracle_info)


if __name__ == "__main__":
    unittest.main()
