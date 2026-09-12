"""Capture auditable native-policy paths for the comparative training figure."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import random
from pathlib import Path

from stable_baselines3 import PPO

from train_portable_ppo_feedforward import make_env, ROOT

RUNS = {
    "bcod-sim": "feedforward-1m-bcod-seed7319",
    "stonefish": "feedforward-1m-stonefish-seed7319-part2",
    "holoocean": "feedforward-1m-holoocean-seed7319-retry-part4",
    "gazebo-harmonic": "feedforward-1m-gazebo-seed7319-part61",
}
EVAL = {
    "bcod-sim": "bcod-sim",
    "stonefish": "stonefish",
    "holoocean": "holoocean",
    "gazebo-harmonic": "gazebo",
}


def position(env, backend):
    if backend in {"bcod-sim", "gazebo-harmonic"}:
        return list(map(float, env.last_truth["position_ned_m"][:2])), "ground_truth"
    if backend == "stonefish":
        return list(map(float, env.task.position[:2])), "task_gps"
    return list(map(float, env._last_gps_position_ned_m[:2])), "task_gps"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True, choices=RUNS)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    backend = args.backend
    source = ROOT / "artifacts/rl-campaign/training-runs" / RUNS[backend]
    checkpoint = source / "model-final.zip"
    manifest = json.loads((source / "run-manifest.json").read_text())
    if manifest["status"] != "completed" or not checkpoint.is_file():
        raise RuntimeError("Final training completion or checkpoint is unconfirmed")
    order = list(range(10000, 10050))
    random.Random(20260912).shuffle(order)
    eval_file = ROOT / "artifacts/common-suite/native-three-policy-20260909" / EVAL[backend] / "evaluation-episodes.csv"
    with eval_file.open() as stream:
        outcomes = {int(row["seed"]): str(row["success"]).lower() in {"true", "1"}
                    for row in csv.DictReader(stream)}
    seeds = order[:3]
    if not all(outcomes.values()):
        first_failure = next(seed for seed in order if not outcomes[seed])
        if first_failure not in seeds:
            seeds.append(first_failure)
    model = PPO.load(str(checkpoint), device="cpu")
    kwargs = dict(backend=backend, base_seed=7319, fixed_reset_seed=None,
                  final_leg_curriculum=False, condition_contract_path=None,
                  disturbance_mode="zero", holoocean_wind_mode="off",
                  runtime_command=None, diagnostic_only=False,
                  stonefish_executable=Path("/mnt/shared/gpfs/home/aadik3/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge"),
                  stonefish_data_dir=Path("/mnt/shared/gpfs/home/aadik3/stonefish-src/Tests/Data"),
                  stonefish_lib=Path("/mnt/shared/gpfs/home/aadik3/stonefish-install/lib"),
                  stonefish_deps_lib=Path("/mnt/shared/gpfs/home/aadik3/stonefish-deps/lib"),
                  stonefish_physics_threads=1, stonefish_sensor_noise=False)
    args.output.mkdir(parents=True, exist_ok=True)
    summary = []
    for seed in seeds:
        kwargs["fixed_reset_seed"] = seed
        env = make_env(argparse.Namespace(**kwargs))
        rows = []
        try:
            obs, _ = env.reset()
            p, position_source = position(env, backend)
            rows.append({"control_step": 0, "time_s": 0.0, "north_m": p[0], "east_m": p[1], "reward": 0.0})
            total = 0.0
            for step in range(1, 10001):
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                total += float(reward)
                p, _ = position(env, backend)
                rows.append({"control_step": step, "time_s": step * float(env.control_interval_s),
                             "north_m": p[0], "east_m": p[1], "reward": float(reward)})
                if terminated or truncated:
                    break
            else:
                raise RuntimeError(f"Episode {seed} exceeded control-step guard")
            with (args.output / f"{backend}-seed-{seed}.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            summary.append({"seed": seed, "success": bool(info["success"]),
                            "termination_reason": info["termination_reason"],
                            "return": total, "control_steps": len(rows)-1,
                            "position_source": position_source})
        finally:
            env.close()
    record = {"backend": backend, "training_run": RUNS[backend],
              "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
              "selection": "Shuffle held-out seeds 10000-10049 with Python random.Random(20260912); take first three, then append first failure in the shuffled order if final 50-seed success rate is below 100% and the first three contain no failure.",
              "shuffled_seed_order": order, "episodes": summary}
    (args.output / f"{backend}-manifest.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record))


if __name__ == "__main__":
    main()
