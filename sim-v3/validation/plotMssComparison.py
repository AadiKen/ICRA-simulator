#!/usr/bin/env python3
"""Plot official MSS Otter traces against the JavaScript planar simulator."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "bcod-matplotlib"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
VALIDATION = ROOT / "validation"
MANEUVERS = [
    "constant-thrust",
    "coast-down",
    "turning-circle",
    "zig-zag",
    "current-drift",
]


def load_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(line for line in stream if not line.startswith("#")))
    if not rows:
        raise RuntimeError(f"No samples found in {path}")
    return {
        key: np.asarray([float(row[key]) for row in rows], dtype=float)
        for key in rows[0]
    }


def angle_error(actual: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(actual - reference), np.cos(actual - reference))


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def metrics(actual: dict[str, np.ndarray], reference: dict[str, np.ndarray]) -> dict[str, float]:
    position_error = np.hypot(actual["N"] - reference["N"], actual["E"] - reference["E"])
    heading_error = angle_error(actual["yaw"], reference["yaw"])
    speed_error = np.hypot(actual["u"] - reference["u"], actual["v"] - reference["v"])
    return {
        "positionRmseMeters": rms(position_error),
        "headingRmseDegrees": math.degrees(rms(heading_error)),
        "bodySpeedRmseMetersPerSecond": rms(speed_error),
        "maxPositionErrorMeters": float(np.max(position_error)),
        "maxHeadingErrorDegrees": math.degrees(float(np.max(np.abs(heading_error)))),
        "maxBodySpeedErrorMetersPerSecond": float(np.max(speed_error)),
    }


def plot_maneuver(
    name: str,
    actual: dict[str, np.ndarray],
    reference: dict[str, np.ndarray],
    result: dict[str, float],
    output: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"MSS Otter vs simulator — {name}", fontsize=16, fontweight="bold")
    fig.subplots_adjust(top=0.90, bottom=0.08, hspace=0.32, wspace=0.22)

    axes[0, 0].plot(reference["E"], reference["N"], color="#172554", linewidth=2.5, label="MSS Otter")
    axes[0, 0].plot(actual["E"], actual["N"], "--", color="#f97316", linewidth=1.8, label="Simulator")
    axes[0, 0].set(title="Horizontal track", xlabel="East (m)", ylabel="North (m)")
    axes[0, 0].axis("equal")

    axes[0, 1].plot(reference["t"], reference["u"], color="#172554", linewidth=2.2, label="MSS u")
    axes[0, 1].plot(actual["t"], actual["u"], "--", color="#f97316", linewidth=1.6, label="Simulator u")
    axes[0, 1].plot(reference["t"], reference["v"], color="#2563eb", linewidth=1.8, label="MSS v")
    axes[0, 1].plot(actual["t"], actual["v"], "--", color="#fb923c", linewidth=1.4, label="Simulator v")
    axes[0, 1].set(title="Body velocities", xlabel="Time (s)", ylabel="Velocity (m/s)")

    axes[1, 0].plot(reference["t"], np.degrees(np.unwrap(reference["yaw"])), color="#172554", linewidth=2.2, label="MSS")
    axes[1, 0].plot(actual["t"], np.degrees(np.unwrap(actual["yaw"])), "--", color="#f97316", linewidth=1.6, label="Simulator")
    axes[1, 0].set(title="Heading", xlabel="Time (s)", ylabel="Yaw (deg)")

    position_error = np.hypot(actual["N"] - reference["N"], actual["E"] - reference["E"])
    heading_error = np.degrees(angle_error(actual["yaw"], reference["yaw"]))
    axes[1, 1].plot(actual["t"], position_error, color="#dc2626", label="Position error (m)")
    axes[1, 1].plot(actual["t"], np.abs(heading_error), color="#7c3aed", label="|Heading error| (deg)")
    axes[1, 1].set(title="Absolute errors", xlabel="Time (s)", ylabel="Error")

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=8)

    summary = (
        f"Position RMSE: {result['positionRmseMeters']:.6f} m\n"
        f"Heading RMSE: {result['headingRmseDegrees']:.6f}°\n"
        f"Speed RMSE: {result['bodySpeedRmseMetersPerSecond']:.6f} m/s"
    )
    axes[1, 1].text(
        0.98, 0.04, summary,
        transform=axes[1, 1].transAxes,
        ha="right",
        va="bottom",
        fontsize=9,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85, "edgecolor": "#d1d5db"},
    )
    fig.savefig(output, dpi=180)
    plt.close(fig)


def plot_summary(results: dict[str, dict[str, float]], limits: dict[str, float], output: Path) -> None:
    names = list(results)
    labels = [name.replace("-", "\n") for name in names]
    series = [
        ("Position RMSE", "positionRmseMeters", limits["positionRmseMeters"]),
        ("Heading RMSE", "headingRmseDegrees", limits["headingRmseDegrees"]),
        ("Speed RMSE", "bodySpeedRmseMetersPerSecond", limits["bodySpeedRmseMetersPerSecond"]),
    ]
    x = np.arange(len(names))
    width = 0.24
    fig, axis = plt.subplots(figsize=(12, 6), constrained_layout=True)
    for index, (label, key, limit) in enumerate(series):
        normalized = [results[name][key] / limit for name in names]
        axis.bar(x + (index - 1) * width, normalized, width, label=label)
    axis.axhline(1, color="#dc2626", linestyle="--", linewidth=2, label="Acceptance limit")
    axis.set(
        title="MSS Otter validation — RMSE as fraction of acceptance limit",
        ylabel="RMSE / acceptance limit",
        xticks=x,
        xticklabels=labels,
    )
    axis.grid(True, axis="y", alpha=0.25)
    axis.legend()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=VALIDATION / "mss-plots")
    parser.add_argument("--golden", type=Path, default=VALIDATION / "mss-golden")
    args = parser.parse_args()
    output = args.output.resolve()
    replay_dir = output / "simulator-traces"
    output.mkdir(parents=True, exist_ok=True)

    subprocess.run(
        ["node", str(VALIDATION / "exportMssReplay.js"), str(replay_dir)],
        cwd=ROOT,
        check=True,
    )

    config = json.loads((VALIDATION / "mss-reference.json").read_text(encoding="utf-8"))
    limits = {
        "positionRmseMeters": config["acceptance"]["positionRmseMeters"],
        "headingRmseDegrees": config["acceptance"]["headingRmseDegrees"],
        "bodySpeedRmseMetersPerSecond": config["acceptance"]["bodySpeedRmseMetersPerSecond"],
    }
    results: dict[str, dict[str, float]] = {}
    for name in MANEUVERS:
        reference = load_csv(args.golden / f"{name}.csv")
        actual = load_csv(replay_dir / f"{name}.csv")
        if len(reference["t"]) != len(actual["t"]):
            raise RuntimeError(f"{name}: MSS and simulator sample counts differ")
        result = metrics(actual, reference)
        results[name] = result
        plot_maneuver(name, actual, reference, result, output / f"{name}.png")

    plot_summary(results, limits, output / "summary.png")
    report = {
        "referenceCommit": config["commit"],
        "acceptanceLimits": limits,
        "maneuvers": results,
    }
    (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote MSS comparison plots and metrics to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
