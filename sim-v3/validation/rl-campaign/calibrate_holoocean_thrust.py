"""Empirically calibrate HoloOcean SurfaceVessel thrust for a 1 m/s cruise."""
from __future__ import annotations

import json
from pathlib import Path
import statistics
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client/bcod_sim"))
from holoocean_vehicle_a_env import HoloOceanVehicleAEnv


TARGET_SPEED_MPS = 1.0
SEED = 7319
PROBE_SECONDS = 40.0
STEADY_WINDOW_SECONDS = 10.0


def probe(ceiling_n: float) -> dict:
    env = HoloOceanVehicleAEnv(ROOT, fixed_reset_seed=SEED, settle_physics_steps=20)
    env.max_thrust_n = float(ceiling_n)
    original = env._draw_randomization

    def calm(seed: int):
        result = original(seed)
        result["current_nwu_mps"] = [0.0, 0.0, 0.0]
        result["wind_ned_mps"] = [0.0, 0.0, 0.0]
        return result

    env._draw_randomization = calm
    speeds = []
    try:
        env.reset()
        steps = round(PROBE_SECONDS / env.control_interval_s)
        for _ in range(steps):
            env.step(np.ones(2, dtype=np.float32))
            speeds.append(float(np.linalg.norm(env._ground_velocity_ned_mps)))
    finally:
        env.close()
    window = round(STEADY_WINDOW_SECONDS / env.control_interval_s)
    return {
        "ceiling_n_per_thruster": ceiling_n,
        "mean_last_10_s_mps": statistics.fmean(speeds[-window:]),
        "final_speed_mps": speeds[-1],
        "maximum_speed_mps": max(speeds),
    }


def main() -> None:
    rows = [probe(95.0), probe(1500.0)]
    low, high = 95.0, 1500.0
    for _ in range(9):
        midpoint = (low + high) / 2.0
        row = probe(midpoint)
        rows.append(row)
        if row["mean_last_10_s_mps"] < TARGET_SPEED_MPS:
            low = midpoint
        else:
            high = midpoint
    best = min(rows, key=lambda row: abs(row["mean_last_10_s_mps"] - TARGET_SPEED_MPS))
    artifact = {
        "schema_version": 1,
        "artifact_kind": "holoocean-vehicle-a-thrust-performance-calibration",
        "method": "bisection over per-thruster ceiling; sustained equal full command; calm water; 40 s probe; final 10 s averaged",
        "target_cruise_speed_mps": TARGET_SPEED_MPS,
        "seed": SEED,
        "literal_force_match_baseline_n_per_thruster": 95.0,
        "literal_force_match_abandoned_reason": "HoloOcean SurfaceVessel mass and damping are binary-hardcoded and cannot be matched to bcod-sim Vehicle A through scenario configuration.",
        "probes": rows,
        "selected": best,
        "lag_time_constant_s": 0.25,
    }
    output = ROOT / "artifacts/rl-campaign/holoocean-vehicle-a-thrust-calibration.json"
    output.write_text(json.dumps(artifact, indent=2) + "\n")
    print(json.dumps(artifact, indent=2))


if __name__ == "__main__":
    main()
