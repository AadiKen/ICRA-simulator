#!/usr/bin/env python3
"""Verify intentional Surveyor motor lag does not trigger allocation failure."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))

from bcod_sim import GazeboGymEnv  # noqa: E402


def main() -> int:
    runtime = [sys.executable, str(
        ROOT / "validation/rl-campaign/ports/gazebo_gym_runtime.py")]
    env = GazeboGymEnv(
        ROOT, runtime, allow_unconformant_diagnostic=True,
        fixed_reset_seed=7319, disturbance_mode="zero",
    )
    rows = []
    try:
        env.reset()
        for index in range(40):
            action = np.asarray(
                [1.0, -1.0] if index % 2 == 0 else [-1.0, 1.0],
                dtype=np.float32,
            )
            _, _, terminated, truncated, info = env.step(action)
            rows.append({
                "step": index + 1,
                "termination_reason": info["termination_reason"],
                "policy_target_thrust_newtons": info.get("policy_target_thrust_newtons"),
                "commanded_thrust_newtons": info.get("commanded_thrust_newtons"),
                "applied_thrust_newtons": info.get("applied_thrust_newtons"),
            })
            if terminated or truncated:
                raise RuntimeError(
                    f"unexpected termination at step {index + 1}: "
                    f"{info['termination_reason']}"
                )
    finally:
        env.close()
    print(json.dumps({"status": "PASS", "steps": len(rows), "last": rows[-1]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
