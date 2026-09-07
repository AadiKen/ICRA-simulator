from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

import stonefish_gym_env as module


class FakeBridge:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def close(self):
        self.closed = True


class FakeTask:
    def __init__(self, bridge, contract):
        self.bridge = bridge
        self.actions = []

    def reset(self, seed):
        return np.zeros(15), {"seed": seed}

    def step(self, action):
        self.actions.append(action)
        return np.ones(15), 2.5, True, False, {
            "success": True, "termination_reason": "success", "physics_steps": 2,
        }


class StonefishGymEnvTest(unittest.TestCase):
    @patch.object(module, "StonefishCommonTask", FakeTask)
    @patch.object(module, "StonefishBridge", FakeBridge)
    def test_maps_two_policy_actions_and_seeds_episodes(self):
        env = module.StonefishGymEnv(
            Path("."), executable="bridge", data_dir="data", base_seed=20)
        observation, info = env.reset()
        self.assertEqual(observation.shape, (15,))
        self.assertEqual(info["seed"], 20)
        _, reward, terminated, truncated, _ = env.step(np.asarray([.2, -.3]))
        self.assertTrue(np.allclose(env.task.actions[0], [.2, -.3, 0.0, 0.0]))
        self.assertEqual(reward, 2.5)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        env.close()
        self.assertTrue(env.bridge.closed)


if __name__ == "__main__":
    unittest.main()
