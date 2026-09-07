import importlib.util
import csv
from pathlib import Path
import tempfile
import unittest

import numpy as np
import json


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


class FakeComponentEnv(MODULE.gym.Env):
    observation_space = None
    action_space = None
    metadata = {}
    render_mode = None
    spec = None
    def __init__(self): self.i = 0
    def reset(self, **_kwargs): self.i = 0; return np.zeros(15), {"seed": 42}
    def step(self, _action):
        self.i += 1
        done = self.i == 2
        components = {
            "progress": 1.0, "cross_track": -0.2, "action_delta": -0.1,
            "terminal": -10.0 if done else 0.0,
            "base_reward": -9.3 if done else 0.7,
            "potential_shaping": 0.3,
            "shaped_reward": -9.0 if done else 1.0,
        }
        return np.zeros(15), components["shaped_reward"], False, done, {
            "physics_steps": self.i * 2,
            "completion_fraction": self.i / 2,
            "success": False,
            "waypoints_reached": self.i - 1,
            "termination_reason": "timeout" if done else "running",
            "reward_components": components,
        }
    def close(self): pass


class PortableRecurrentPPOTest(unittest.TestCase):
    def test_gazebo_algorithm_config_is_byte_identical_to_bcod(self):
        bcod = MODULE.algorithm_config_bytes("bcod-sim")
        gazebo = MODULE.algorithm_config_bytes("gazebo-harmonic")
        self.assertEqual(gazebo, bcod)
        config = json.loads(gazebo)
        self.assertEqual(config["algorithm"], "RecurrentPPO")
        self.assertEqual(config["policy"], "MlpLstmPolicy")
        self.assertEqual(config["ent_coef"], 0.005)
        self.assertEqual(config["policy_kwargs"], {
            "enable_critic_lstm": True,
            "lstm_hidden_size": 128,
            "net_arch": [128, 128],
        })
        self.assertEqual(MODULE.algorithm_config_bytes("holoocean"), bcod)
        self.assertEqual(MODULE.algorithm_config_bytes("stonefish"), bcod)

    def test_default_output_is_timestamped_and_backend_scoped(self):
        output = MODULE.default_output("bcod-sim")
        self.assertEqual(output.parent.name, "training-runs")
        self.assertTrue(output.name.startswith("bcod-sim-"))

    def test_entropy_coefficient_is_read_from_bcod_protocol_artifact(self):
        provenance = MODULE.algorithm_provenance("gazebo-harmonic")
        self.assertEqual(
            provenance["ent_coef_source"],
            "artifacts/rl-campaign/surveyor/15-field-rerun-preregistration.json",
        )
        self.assertEqual(provenance["contract_sha256"], MODULE.FROZEN_CONTRACT_SHA256)

    def test_evaluation_threads_lstm_state_and_resets_each_episode(self):
        model = FakeRecurrentModel()
        result = MODULE.evaluate_recurrent(model, FakeEnv, 10, 2)
        self.assertEqual(result["success_rate"], 1.0)
        self.assertEqual(result["median_return"], 3.0)
        self.assertEqual(model.calls, [
            (None, True, True), (1, False, True), (2, False, True),
            (None, True, True), (1, False, True), (2, False, True),
        ])

    def test_reward_component_trace_writes_every_step_and_flushes_terminal_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "components.csv"
            env = MODULE.RewardComponentTrace(FakeComponentEnv(), path)
            env.reset()
            env.step(np.zeros(2))
            env.step(np.zeros(2))
            env.close()
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["episode_seed"], "42")
        self.assertEqual(rows[0]["control_step"], "1")
        self.assertEqual(rows[0]["action_delta"], "-0.1")
        self.assertEqual(rows[1]["terminal"], "-10.0")
        self.assertEqual(rows[1]["termination_reason"], "timeout")
        self.assertEqual(rows[1]["completion_fraction"], "1.0")
        self.assertEqual(rows[1]["waypoints_reached"], "1")
        self.assertEqual(rows[1]["truncated"], "True")

    def test_v7_protocol_contains_frozen_gate_and_budget(self):
        source = SCRIPT.read_text()
        self.assertIn("range(250_000, 1_500_001, 250_000)", source)
        self.assertIn('result["success_rate"] >= .9', source)
        self.assertIn("if consecutive >= 2", source)
        self.assertIn('"full_episode_training_started": False', source)


if __name__ == "__main__": unittest.main()
