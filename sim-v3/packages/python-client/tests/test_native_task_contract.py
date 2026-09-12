from __future__ import annotations

import unittest
from pathlib import Path

from bcod_sim.native_task_contract import load_native_task_contract


ROOT = Path(__file__).resolve().parents[3]
NOMINAL = ROOT / "eval/common_suite/contracts/nominal.json"


class NativeTaskContractTest(unittest.TestCase):
    def test_nominal_condition_drives_native_task_fields(self):
        task, binding = load_native_task_contract(ROOT, NOMINAL)
        self.assertEqual(binding["condition_id"], "nominal")
        self.assertEqual(
            binding["content_sha256"],
            "e998e32f5ee00f6296fce04e01066fa3e9a40d3737dec66b6aae79ef212f119b",
        )
        self.assertEqual(
            task["reset_randomization"]["route_relative_m"],
            [[45.0, 8.0], [90.0, -6.0], [140.0, 12.0]],
        )
        self.assertEqual(task["reset_randomization"]["current_speed_m_s"], [0.0, 0.45])
        self.assertEqual(task["reset_randomization"]["wind_speed_m_s"], [0.0, 5.0])
        self.assertEqual(task["timing"]["episode_length_steps"], 3000)
        self.assertEqual(task["timing"]["episode_length_s"], 150.0)
        self.assertEqual(task["evaluation_termination"]["energy_cap_ns"], 50000.0)

    def test_default_callers_keep_the_training_contract(self):
        task, binding = load_native_task_contract(ROOT)
        self.assertIsNone(binding["condition_id"])
        self.assertEqual(task["timing"]["episode_length_steps"], 2400)
        self.assertEqual(task["timing"]["episode_length_s"], 120)


if __name__ == "__main__":
    unittest.main()
