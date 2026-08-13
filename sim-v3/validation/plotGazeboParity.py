#!/usr/bin/env python3
"""
Plot Node-vs-Gazebo parity trajectories and errors.

Default usage:
    python3 validation/plotGazeboParity.py

Writes:
    gazebo/generated/reports/otter_constant-thrust_parity.png
"""

import argparse
import csv
import json
import math
import os
import subprocess
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "bcod-matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "bcod-cache"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GOLDEN = REPO_ROOT / "gazebo/generated/golden/otter/constant-thrust.csv"
DEFAULT_OUT = REPO_ROOT / "gazebo/generated/reports/otter_constant-thrust_parity.png"


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize Node vs Gazebo parity trajectory and error.")
    parser.add_argument("--maneuver", default="constant-thrust", help="Maneuver name understood by validation/goldenLogCompare.js.")
    parser.add_argument("--golden", default=str(DEFAULT_GOLDEN), help="Gazebo golden CSV to compare against.")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="Output PNG path.")
    parser.add_argument("--title", default=None, help="Optional figure title.")
    return parser.parse_args()


def load_csv(path):
    with open(path, newline="") as handle:
        rows = []
        for row in csv.DictReader(line for line in handle if not line.lstrip().startswith("#")):
            rows.append({key: float(value) for key, value in row.items()})
    if not rows:
        raise ValueError(f"No samples found in {path}")
    return rows


def load_node_replay(maneuver):
    code = f"""
import {{replayManeuver}} from './validation/goldenLogCompare.js';
const rows = replayManeuver({json.dumps(maneuver)});
console.log(JSON.stringify(rows));
"""
    result = subprocess.run(
        ["node", "--input-type=module", "-e", code],
        cwd=REPO_ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return json.loads(result.stdout)


def align(node_rows, gazebo_rows):
    count = min(len(node_rows), len(gazebo_rows))
    node = node_rows[:count]
    gazebo = gazebo_rows[:count]
    aligned = []
    for n, g in zip(node, gazebo):
        d_n = n["N"] - g["N"]
        d_e = n["E"] - g["E"]
        d_yaw = n["yaw"] - g["yaw"]
        aligned.append({
            "t": n["t"],
            "node_N": n["N"],
            "node_E": n["E"],
            "node_yaw": n["yaw"],
            "node_u": n["u"],
            "node_v": n["v"],
            "node_r": n["r"],
            "gazebo_N": g["N"],
            "gazebo_E": g["E"],
            "gazebo_yaw": g["yaw"],
            "gazebo_u": g["u"],
            "gazebo_v": g["v"],
            "gazebo_r": g["r"],
            "err_N": d_n,
            "err_E": d_e,
            "err_yaw": d_yaw,
            "err_pos": math.hypot(d_n, d_e),
            "err_u": n["u"] - g["u"],
        })
    return aligned


def rms(rows, key):
    return math.sqrt(sum(row[key] * row[key] for row in rows) / max(len(rows), 1))


def max_abs(rows, key):
    return max(abs(row[key]) for row in rows)


def series(rows, key):
    return [row[key] for row in rows]


def first_motion_time(rows, prefix):
    for row in rows:
        if math.hypot(row[f"{prefix}_u"], row[f"{prefix}_v"]) > 0.05:
            return row["t"]
    return None


def style_axes(ax, title, xlabel=None, ylabel=None):
    ax.set_title(title, loc="left", fontsize=11, fontweight="bold")
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, color="#d7dde5", linewidth=0.8, alpha=0.75)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color("#b7c0cc")


def make_figure(rows, out_path, title):
    t = series(rows, "t")
    node_color = "#0b5cad"
    gazebo_color = "#d45b2c"
    error_color = "#2d7f5e"
    yaw_color = "#6c4db5"

    metrics = {
        "rms_N": rms(rows, "err_N"),
        "rms_E": rms(rows, "err_E"),
        "rms_yaw": rms(rows, "err_yaw"),
        "max_pos": max_abs(rows, "err_pos"),
        "final_node_N": rows[-1]["node_N"],
        "final_gazebo_N": rows[-1]["gazebo_N"],
        "first_node": first_motion_time(rows, "node"),
        "first_gazebo": first_motion_time(rows, "gazebo"),
    }

    fig = plt.figure(figsize=(16, 10), facecolor="#f7f9fc")
    gs = GridSpec(3, 3, figure=fig, height_ratios=[1.1, 1.0, 0.95], hspace=0.38, wspace=0.28)
    fig.suptitle(title, x=0.055, y=0.975, ha="left", fontsize=18, fontweight="bold", color="#18212f")

    ax_traj = fig.add_subplot(gs[:2, :2])
    ax_traj.plot(series(rows, "node_E"), series(rows, "node_N"), color=node_color, lw=2.6, label="Node/core")
    ax_traj.plot(series(rows, "gazebo_E"), series(rows, "gazebo_N"), color=gazebo_color, lw=2.4, ls="--", label="Gazebo")
    ax_traj.scatter([rows[0]["node_E"]], [rows[0]["node_N"]], s=60, color="#2d3748", zorder=5, label="start")
    ax_traj.scatter([rows[-1]["node_E"], rows[-1]["gazebo_E"]], [rows[-1]["node_N"], rows[-1]["gazebo_N"]],
                    s=72, color=[node_color, gazebo_color], edgecolor="white", linewidth=1.2, zorder=6)
    ax_traj.set_aspect("equal", adjustable="datalim")
    style_axes(ax_traj, "Trajectory in N/E Plane", "East [m]", "North [m]")
    ax_traj.legend(loc="upper left", frameon=True, facecolor="white", edgecolor="#c9d2df")

    ax_summary = fig.add_subplot(gs[0, 2])
    ax_summary.axis("off")
    summary = [
        ("RMS North", f"{metrics['rms_N']:.3f} m"),
        ("RMS East", f"{metrics['rms_E']:.3e} m"),
        ("RMS yaw", f"{metrics['rms_yaw']:.3e} rad"),
        ("Max position err", f"{metrics['max_pos']:.3f} m"),
        ("Final Node N", f"{metrics['final_node_N']:.3f} m"),
        ("Final Gazebo N", f"{metrics['final_gazebo_N']:.3f} m"),
        ("First motion Node", f"{metrics['first_node']:.2f} s"),
        ("First motion Gazebo", f"{metrics['first_gazebo']:.2f} s"),
    ]
    ax_summary.text(0, 1, "Comparison Summary", fontsize=12, fontweight="bold", color="#18212f", va="top")
    for idx, (label, value) in enumerate(summary):
        y = 0.87 - idx * 0.105
        ax_summary.text(0.02, y, label, fontsize=10, color="#4b5563", va="center")
        ax_summary.text(0.98, y, value, fontsize=10.5, color="#111827", va="center", ha="right", fontweight="bold")
    ax_summary.add_patch(plt.Rectangle((0, 0.02), 1, 0.92, transform=ax_summary.transAxes,
                                       facecolor="white", edgecolor="#c9d2df", linewidth=1.0, zorder=-1))

    ax_n = fig.add_subplot(gs[1, 2])
    ax_n.plot(t, series(rows, "node_N"), color=node_color, lw=2, label="Node")
    ax_n.plot(t, series(rows, "gazebo_N"), color=gazebo_color, lw=2, ls="--", label="Gazebo")
    style_axes(ax_n, "North Position", "time [s]", "N [m]")
    ax_n.legend(loc="lower right", fontsize=9, frameon=True, facecolor="white", edgecolor="#c9d2df")

    ax_err = fig.add_subplot(gs[2, 0])
    ax_err.plot(t, series(rows, "err_pos"), color=error_color, lw=2)
    style_axes(ax_err, "Position Error Magnitude", "time [s]", "|Δpos| [m]")

    ax_u = fig.add_subplot(gs[2, 1])
    ax_u.plot(t, series(rows, "node_u"), color=node_color, lw=2, label="Node u")
    ax_u.plot(t, series(rows, "gazebo_u"), color=gazebo_color, lw=2, ls="--", label="Gazebo u")
    style_axes(ax_u, "Surge Velocity", "time [s]", "u [m/s]")
    ax_u.legend(loc="lower right", fontsize=9, frameon=True, facecolor="white", edgecolor="#c9d2df")

    ax_components = fig.add_subplot(gs[2, 2])
    ax_components.plot(t, series(rows, "err_N"), color="#1f77b4", lw=1.8, label="ΔN")
    ax_components.plot(t, series(rows, "err_E"), color="#d62728", lw=1.8, label="ΔE")
    ax_components.plot(t, series(rows, "err_yaw"), color=yaw_color, lw=1.8, label="Δyaw")
    ax_components.axhline(0, color="#6b7280", lw=0.9)
    style_axes(ax_components, "Signed Error Components", "time [s]", "error")
    ax_components.legend(loc="best", fontsize=9, frameon=True, facecolor="white", edgecolor="#c9d2df")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=170, bbox_inches="tight")
    plt.close(fig)
    return metrics


def main():
    args = parse_args()
    gazebo_path = Path(args.golden)
    out_path = Path(args.out)
    node_rows = load_node_replay(args.maneuver)
    gazebo_rows = load_csv(gazebo_path)
    rows = align(node_rows, gazebo_rows)
    title = args.title or f"Otter Gazebo Parity: {args.maneuver}"
    metrics = make_figure(rows, out_path, title)
    print(json.dumps({
        "maneuver": args.maneuver,
        "golden": str(gazebo_path),
        "out": str(out_path),
        "samples": len(rows),
        "metrics": metrics,
    }, indent=2))


if __name__ == "__main__":
    main()
