import importlib.util
import os
from pathlib import Path
import unittest

MODULE = Path(__file__).resolve().parents[1] / "train_portable_ppo.py"
spec = importlib.util.spec_from_file_location("train_portable_ppo", MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Env:
    backend_type = "bcod-sim"
    vehicle_preset = "vehicle-a-otter"


class EpisodeWriterTest(unittest.TestCase):
    def test_populates_all_declared_columns(self):
        row = module.episode_metric_row(
            seed=17, env=Env(),
            info={"termination_reason": "timeout", "success": False, "physics_steps": 2400},
            total_return=-3.5, wall_clock_s=.25,
            policy_id="ppo-test", algorithm="RecurrentPPO",
        )
        self.assertEqual(tuple(row), module.EPISODE_COLUMNS)
        self.assertEqual(row["host_class"], "local")
        self.assertEqual(row["collision_type"], "none")
        self.assertEqual(row["episode_length"], 2400)

    def test_cluster_detection_and_explicit_override(self):
        old_slurm, old_class = os.environ.get("SLURM_JOB_ID"), os.environ.get("BCOD_HOST_CLASS")
        try:
            os.environ.pop("BCOD_HOST_CLASS", None)
            os.environ["SLURM_JOB_ID"] = "12345"
            self.assertEqual(module.detect_host_class(), "cluster")
            os.environ["BCOD_HOST_CLASS"] = "synthetic"
            self.assertEqual(module.detect_host_class(), "synthetic")
            os.environ["BCOD_HOST_CLASS"] = "invalid"
            with self.assertRaises(RuntimeError):
                module.detect_host_class()
        finally:
            if old_slurm is None: os.environ.pop("SLURM_JOB_ID", None)
            else: os.environ["SLURM_JOB_ID"] = old_slurm
            if old_class is None: os.environ.pop("BCOD_HOST_CLASS", None)
            else: os.environ["BCOD_HOST_CLASS"] = old_class


if __name__ == "__main__":
    unittest.main()
