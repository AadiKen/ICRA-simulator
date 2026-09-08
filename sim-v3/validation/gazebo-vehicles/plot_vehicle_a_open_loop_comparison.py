#!/usr/bin/env python3
import json
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[2]
source = ROOT / "artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison.json"
output = ROOT / "artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison.png"
artifact = json.loads(source.read_text())

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9})
fig, axes = plt.subplots(2, 3, figsize=(13.5, 8))
titles = {
    "constant-thrust": "Constant thrust", "turning-circle": "Turning circle",
    "yaw-turn": "Yaw turn", "zig-zag": "Zig-zag",
    "coast-down": "Coast-down", "current-drift": "Current drift",
}
for ax, scenario in zip(axes.flat, artifact["scenarios"]):
    b, g, summary = scenario["bcod_sim"], scenario["gazebo_harmonic"], scenario["summary"]
    bn = [x["north_m"] for x in b]; be = [x["east_m"] for x in b]
    gn = [x["north_m"] for x in g]; ge = [x["east_m"] for x in g]
    ax.plot(be, bn, color="#1261a0", lw=3.0, label="bcod-sim", zorder=2)
    ax.plot(ge, gn, color="#ef7d32", lw=1.7, ls="--", label="Gazebo", zorder=3)
    ax.scatter([be[0]], [bn[0]], s=26, color="#212121", marker="o", zorder=4)
    ax.scatter([be[-1], ge[-1]], [bn[-1], gn[-1]], s=30,
               color=["#1261a0", "#ef7d32"], marker="x", zorder=4)
    p = summary["horizontal_position_error_m"]; h = summary["heading_error_rad"]
    ax.text(.03, .97, f"mean Δp  {p['mean']:.3f} m\nmax Δp   {p['max']:.3f} m\nmax Δψ  {h['max']*180/3.141592653589793:.2f}°",
            transform=ax.transAxes, va="top", ha="left", fontsize=8,
            bbox={"boxstyle":"round,pad=.35", "facecolor":"white", "edgecolor":"#d6dbe1", "alpha":.92})
    ax.set_title(titles[scenario["id"]], weight="bold", fontsize=11)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.grid(True, color="#dfe4ea", lw=.7); ax.set_aspect("equal", adjustable="datalim")
    ax.spines[["top", "right"]].set_visible(False)

fig.suptitle("Vehicle A: bcod-sim closely tracks Gazebo across six open-loop maneuvers", fontsize=17, weight="bold", y=.985)
fig.text(.5, .94, "Full-resolution 20 Hz trajectories · mean position error 0.026–0.203 m · maximum error ≤ 0.256 m", ha="center", fontsize=11, color="#4b5563")
legend = [Line2D([0],[0],color="#1261a0",lw=3,label="bcod-sim"), Line2D([0],[0],color="#ef7d32",lw=2,ls="--",label="Gazebo Harmonic"), Line2D([0],[0],marker="o",color="#212121",lw=0,label="Start")]
fig.legend(handles=legend, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(.5,-.015), fontsize=10)
fig.tight_layout(rect=[.01,.055,.99,.91], h_pad=1.1, w_pad=1.1)
fig.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
print(output)
