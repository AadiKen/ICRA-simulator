import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[3] / "validation/rl-campaign/train_portable_ppo.py"
SPEC = importlib.util.spec_from_file_location("train_portable_ppo", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeEnv:
    action_space = None
    def __init__(self, seed): self.seed, self.i, self.closed = seed, 0, False
    def reset(self): return np.zeros(15), {}
    def step(self, action):
        self.i += 1
        done = self.i == 3
        return np.zeros(15), 1.0, done, False, {
            "success": done, "termination_reason": "success" if done else "running"}
    def close(self): self.closed = True


class FakeRecurrentModel:
    def __init__(self): self.calls = []
    def predict(self, observation, *, state, episode_start, deterministic):
        self.calls.append((state, bool(episode_start[0]), deterministic))
        return np.zeros(2), 1 if state is None else state + 1


class PortableRecurrentPPOTest(unittest.TestCase):
    def test_evaluation_threads_lstm_state_and_resets_each_episode(self):
        model = FakeRecurrentModel()
        result = MODULE.evaluate_recurrent(model, FakeEnv, 10, 2)
        self.assertEqual(result["success_rate"], 1.0)
        self.assertEqual(result["median_return"], 3.0)
        self.assertEqual(model.calls, [
            (None, True, True), (1, False, True), (2, False, True),
            (None, True, True), (1, False, True), (2, False, True),
        ])

    def test_v7_protocol_contains_frozen_gate_and_budget(self):
        source = SCRIPT.read_text()
        self.assertIn("range(250_000, 1_500_001, 250_000)", source)
        self.assertIn('result["success_rate"] >= .9', source)
        self.assertIn("if consecutive >= 2", source)
        self.assertIn('"full_episode_training_started": False', source)


if __name__ == "__main__": unittest.main()
