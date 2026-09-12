"""Plot native-step deterministic-replay logs without simulator-specific parsing."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np


SIMULATORS = ("bcod-sim", "gazebo", "vrx", "holoocean", "stonefish")
ROUTE = "determinism-replay-route-v1"
ROOT = Path(__file__).resolve().parents[2]


def load_arm(root: Path, simulator: str, expected_n: int) -> list[dict]:
    directory = root / simulator / ROUTE
    files = sorted(directory.glob("run_*.json"))
    if len(files) != expected_n:
        raise ValueError(f"{simulator}: expected {expected_n} logs, found {len(files)} in {directory}")
    logs = [json.loads(path.read_text()) for path in files]
    for index, log in enumerate(logs):
        if (log.get("simulator"), log.get("route_id"), log.get("vehicle"), log.get("replay_index")) != (simulator, ROUTE, "vehicle-a-otter", index):
            raise ValueError(f"{files[index]}: provenance mismatch")
        if "crashed_at_t" in log:
            raise ValueError(f"{files[index]}: partial run crashed at {log['crashed_at_t']}s; retained for diagnosis, excluded from completed figure")
        samples = log.get("samples", [])
        if len(samples) < 2 or any(samples[i]["t"] <= samples[i - 1]["t"] for i in range(1, len(samples))):
            raise ValueError(f"{files[index]}: missing or nonmonotone native-step samples")
        if not np.isclose(samples[-1]["t"], 90, atol=1e-5):
            raise ValueError(f"{files[index]}: trajectory ends before 90s")
    return logs


def xy(log: dict) -> np.ndarray:
    points = np.asarray([(s["x"], s["y"]) for s in log["samples"]], dtype=float)
    if not np.isfinite(points).all():
        raise ValueError("Non-finite trajectory position")
    return points


def analyze(logs: list[dict], checkpoints: int = 200) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    reference = xy(logs[0])
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
    unique = np.r_[True, np.diff(distance) > 0]
    if distance[-1] <= 0 or np.count_nonzero(unique) < 2:
        raise ValueError("Reference trajectory has zero arc length")
    arclength = np.linspace(0, distance[-1], checkpoints)
    center = np.column_stack([np.interp(arclength, distance[unique], reference[unique, axis]) for axis in (0, 1)])
    tangent = np.gradient(center, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), np.finfo(float).eps)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    # Subtract the reference's own nearest-sample offset. Otherwise an identical
    # replay appears nonzero because checkpoints interpolate between native steps.
    reference_nearest = np.argmin(np.sum((center[:, None, :] - reference[None, :, :]) ** 2, axis=2), axis=1)
    reference_offset = np.sum((reference[reference_nearest] - center) * normal, axis=1)
    offsets = []
    for log in logs[1:]:
        replay = xy(log)
        nearest = np.argmin(np.sum((center[:, None, :] - replay[None, :, :]) ** 2, axis=2), axis=1)
        offsets.append(np.sum((replay[nearest] - center) * normal, axis=1) - reference_offset)
    values = np.asarray(offsets)
    return center, normal, np.stack([values.min(axis=0), values.max(axis=0)]), float(np.sqrt(np.mean(values**2)))


def plot(root: Path, output: Path, n: int) -> dict[str, float]:
    arms = {sim: analyze(load_arm(root, sim, n)) for sim in SIMULATORS}
    boundaries = [center + envelope[side, :, None] * normal for center, normal, envelope, _ in arms.values() for side in (0, 1)]
    all_points = np.concatenate(boundaries)
    low = all_points.min(axis=0)
    high = all_points.max(axis=0)
    pad = max(float(np.max(high - low)) * 0.07, 0.5)
    fig, axes = plt.subplots(1, len(SIMULATORS), figsize=(19, 4.7), sharex=True, sharey=True, constrained_layout=True)
    rmse = {}
    for ax, (sim, (center, normal, envelope, error)) in zip(axes, arms.items()):
        lower = center + envelope[0, :, None] * normal
        upper = center + envelope[1, :, None] * normal
        ax.add_patch(Polygon(np.vstack([upper, lower[::-1]]), closed=True, color="#2271b2", alpha=0.25, linewidth=0))
        ax.plot(center[:, 0], center[:, 1], color="#185a90", lw=1.7)
        ax.set(title=sim, xlim=(low[0] - pad, high[0] + pad), ylim=(low[1] - pad, high[1] + pad), aspect="equal", xlabel="N (m)")
        ax.text(0.97, 0.05, f"RMSE = {error:.4g} m", transform=ax.transAxes, ha="right", va="bottom", fontsize=9, bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8})
        rmse[sim] = error
    axes[0].set_ylabel("E (m)")
    fig.suptitle(f"Vehicle A Otter · 90 s open-loop replay · {n} independent runs\nReference trajectory and signed lateral min/max envelope", fontsize=11)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output.with_suffix(".pdf"))
    fig.savefig(output.with_suffix(".png"), dpi=220)
    plt.close(fig)
    return rmse


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--logs", type=Path, default=ROOT / "artifacts/determinism-replay")
    parser.add_argument("--out", type=Path, default=ROOT / "figures/out/publication/determinism_replay")
    parser.add_argument("--n", type=int, default=30)
    options = parser.parse_args()
    if options.n < 2:
        parser.error("--n must be >= 2")
    print(json.dumps(plot(options.logs, options.out, options.n), indent=2))


if __name__ == "__main__":
    main()
