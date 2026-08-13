import json
import pathlib
import subprocess
import sys
import unittest

import torch

sys.path.insert(0, str(pathlib.Path(__file__).parents[1]))
from bcod_tensor import VehicleAPlanar3Config, VehicleAPlanar3TensorBackend, fnv1a_environment_seed


ROOT = pathlib.Path(__file__).parents[3]


def node_trace(steps: int, dt: float = 0.02) -> dict:
    return json.loads(
        subprocess.check_output(
            ["node", "--experimental-strip-types", "validation/tensor/node_reference.mjs", str(steps), str(dt)],
            cwd=ROOT,
            text=True,
        )
    )


def run_tensor_trace(reference: dict, environments: int = 1, slot: int = 0) -> list[dict]:
    backend = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=environments, timestep_s=reference["dt"]))
    output = []
    for item in reference["trace"]:
        commands = torch.zeros((environments, 2), dtype=torch.float64)
        commands[slot] = torch.tensor(item["command"], dtype=torch.float64)
        state = backend.step(commands)
        output.append(
            {
                "state": state[slot].clone(),
                "acceleration": backend.body_acceleration[slot].clone(),
                "thruster_state": backend.thruster_state[slot].clone(),
                "energy_j": backend.energy_j[slot].clone(),
                "power_w": backend.power_w[slot].clone(),
                "step": backend.step_counters[slot].clone(),
            }
        )
    return output


class VehicleAPlanar3TensorTests(unittest.TestCase):
    def assert_trace_close(self, reference: dict, actual: list[dict], atol: float) -> float:
        maximum = 0.0
        for expected, observed in zip(reference["trace"], actual, strict=True):
            for field in ("state", "acceleration", "thruster_state"):
                target = torch.tensor(expected[field], dtype=torch.float64)
                maximum = max(maximum, torch.max(torch.abs(observed[field] - target)).item())
                torch.testing.assert_close(observed[field], target, rtol=1e-10, atol=atol)
            self.assertEqual(observed["energy_j"].item(), expected["energy_j"])
            self.assertEqual(observed["power_w"].item(), expected["power_w"])
            self.assertEqual(observed["step"].item(), expected["step"])
        return maximum

    def test_short_production_trace_equivalence(self):
        reference = node_trace(200)
        error = self.assert_trace_close(reference, run_tensor_trace(reference), atol=1e-10)
        print(f"short_trace_max_abs_error={error:.17g}")

    def test_long_production_trace_equivalence(self):
        reference = node_trace(1000)
        error = self.assert_trace_close(reference, run_tensor_trace(reference), atol=1e-9)
        print(f"long_trace_max_abs_error={error:.17g}")

    def test_embedded_environment_matches_batch_size_one(self):
        reference = node_trace(200)
        singleton = run_tensor_trace(reference)
        embedded = run_tensor_trace(reference, environments=8, slot=5)
        maximum = 0.0
        for first, second in zip(singleton, embedded, strict=True):
            for field in ("state", "acceleration", "thruster_state"):
                maximum = max(maximum, torch.max(torch.abs(first[field] - second[field])).item())
                torch.testing.assert_close(first[field], second[field], rtol=0, atol=0)
        print(f"embedded_batch_max_abs_error={maximum:.17g}")

    def test_masked_reset_and_active_mask(self):
        backend = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=3))
        backend.step(torch.ones((3, 2), dtype=torch.float64))
        before = backend.state.clone()
        backend.reset(torch.tensor([False, True, False]))
        self.assertTrue(torch.equal(backend.state[0], before[0]))
        self.assertTrue(torch.equal(backend.state[1], torch.zeros(6, dtype=torch.float64)))
        backend.set_active(torch.tensor([True, False, True]))
        frozen = backend.state[1].clone()
        backend.step(torch.ones((3, 2), dtype=torch.float64))
        self.assertTrue(torch.equal(backend.state[1], frozen))

    def test_unavailable_accelerators_fail_explicitly(self):
        if not torch.backends.mps.is_available():
            with self.assertRaisesRegex(RuntimeError, "MPS"):
                VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=1, device="mps"))
        if not torch.cuda.is_available():
            with self.assertRaisesRegex(RuntimeError, "CUDA"):
                VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=1, device="cuda"))


class PerEnvironmentRngTests(unittest.TestCase):
    def draw(self, backend: VehicleAPlanar3TensorBackend, count: int = 8) -> torch.Tensor:
        return torch.stack([backend.random_uniform() for _ in range(count)])

    def test_same_seed_and_layout_reproduce_exactly(self):
        config = VehicleAPlanar3Config(environments=6, experiment_seed="repeatable-π")
        first = VehicleAPlanar3TensorBackend(config)
        second = VehicleAPlanar3TensorBackend(config)
        self.assertTrue(torch.equal(first.rng_state, second.rng_state))
        self.assertTrue(torch.equal(self.draw(first), self.draw(second)))

    def test_environment_indices_have_nonduplicated_streams(self):
        backend = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=16, experiment_seed=413))
        streams = self.draw(backend, 16).T
        self.assertEqual(len(set(backend.initial_rng_state.tolist())), 16)
        self.assertEqual(len({tuple(stream.tolist()) for stream in streams}), 16)
        self.assertEqual(backend.initial_rng_state[7].item(), fnv1a_environment_seed(413, 7))

    def test_masked_reset_only_resets_selected_rng_streams(self):
        backend = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=4, experiment_seed="masked"))
        self.draw(backend, 5)
        before = backend.rng_state.clone()
        backend.reset(torch.tensor([False, True, False, True]))
        self.assertEqual(backend.rng_state[0].item(), before[0].item())
        self.assertEqual(backend.rng_state[2].item(), before[2].item())
        self.assertEqual(backend.rng_state[1].item(), backend.initial_rng_state[1].item())
        self.assertEqual(backend.rng_state[3].item(), backend.initial_rng_state[3].item())

    def test_singleton_and_embedded_environment_streams_agree(self):
        singleton = VehicleAPlanar3TensorBackend(
            VehicleAPlanar3Config(environments=1, experiment_seed="layout", environment_indices=(5,))
        )
        batch = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=8, experiment_seed="layout"))
        self.assertTrue(torch.equal(self.draw(singleton, 32)[:, 0], self.draw(batch, 32)[:, 5]))

    def test_rng_plumbing_does_not_perturb_deterministic_physics(self):
        config = VehicleAPlanar3Config(environments=3, experiment_seed="physics-independent")
        unused = VehicleAPlanar3TensorBackend(config)
        consumed = VehicleAPlanar3TensorBackend(config)
        commands = torch.tensor([[0.7, 0.2], [-0.4, 0.9], [1.0, 1.0]], dtype=torch.float64)
        for _ in range(100):
            consumed.random_uniform()
            self.assertTrue(torch.equal(unused.step(commands), consumed.step(commands)))
            self.assertTrue(torch.equal(unused.thruster_state, consumed.thruster_state))

    def test_checkpoint_restores_rng_and_physics_bit_identically(self):
        backend = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(environments=3, experiment_seed="checkpoint"))
        commands = torch.tensor([[0.2, 0.8], [0.5, -0.5], [-0.3, -0.7]], dtype=torch.float64)
        for _ in range(7):
            backend.step(commands)
            backend.random_uniform()
        checkpoint = backend.save_checkpoint()
        expected_random = self.draw(backend, 12)
        expected_state = torch.stack([backend.step(commands) for _ in range(12)])
        backend.load_checkpoint(checkpoint)
        self.assertTrue(torch.equal(self.draw(backend, 12), expected_random))
        self.assertTrue(torch.equal(torch.stack([backend.step(commands) for _ in range(12)]), expected_state))


if __name__ == "__main__":
    unittest.main()
