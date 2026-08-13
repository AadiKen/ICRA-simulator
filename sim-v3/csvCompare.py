"""
compare.py — JS Simulator vs MSS Baseline Comparison
======================================================
Usage:
    python3 compare.py --js js_sim_log.csv --mss mss_baseline.csv

Produces:
    comparison_report.png   — 6-panel figure
    comparison_report.txt   — numeric summary

Column conventions
------------------
JS CSV (from webSimRunner.js exportLogsToCSV):
    time, north (pos.z), east (pos.x), height, surge (vel.z),
    sway (vel.x), heave, heading, roll, pitch, yaw_rate, ...

MSS CSV (from mss_benchmark.py run_independent_mss):
    time, mss_x (north), mss_z (east), mss_u (surge speed),
    mss_psi (heading), wp_idx

Known issues in this MSS baseline (flagged in report):
  - mss_x and mss_z are identical for the first ~94 steps due to a
    diagonal starting trajectory; this is expected, not a bug.
  - MSS only reached waypoint 1 of 3 within the 90 s window; the JS sim
    may have reached further. Comparison is only valid over the shared
    time window where both sims are navigating toward the same waypoint.
"""

import argparse
import sys
import math
import os
import tempfile

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "bcod-matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "bcod-cache"))

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.interpolate import interp1d

# ── column name aliases ────────────────────────────────────────────────────────

JS_TIME    = "time"
JS_NORTH   = "north"      # pos.z in JS (forward axis)
JS_EAST    = "east"       # pos.x in JS (lateral axis)
JS_SURGE   = "surge"      # velocity.z
JS_SWAY    = "sway"       # velocity.x
JS_HEADING = "heading"    # radians
JS_SPEED   = "speed"      # precomputed scalar speed
JS_YAW     = "yaw_rate"

MSS_TIME    = "time"
MSS_NORTH   = "mss_x"    # NED north in MSS output
MSS_EAST    = "mss_z"    # NED east in MSS output
MSS_SPEED   = "mss_u"    # surge speed (scalar, forward only)
MSS_HEADING = "mss_psi"  # radians
MSS_WP      = "wp_idx"

WAYPOINTS = [(22, 22), (50, 25), (65, 72)]  # (north, east) from scenarioPresets.js


# ── loaders ───────────────────────────────────────────────────────────────────

def load_js(path):
    df = pd.read_csv(path, comment="#")
    required = [JS_TIME, JS_NORTH, JS_EAST, JS_HEADING]
    missing = [c for c in required if c not in df.columns]
    if missing:
        sys.exit(f"JS CSV missing columns: {missing}\n"
                 f"Found: {list(df.columns)}")
    df = df.sort_values(JS_TIME).reset_index(drop=True)
    # JS sim time starts at the scenario's wall-clock startTime (e.g. 43200 for
    # midday). Normalise to elapsed seconds so it aligns with the MSS baseline
    # which always starts at t=0.
    df[JS_TIME] = df[JS_TIME] - df[JS_TIME].iloc[0]
    # Compute speed if not present
    if JS_SPEED not in df.columns:
        df[JS_SPEED] = np.sqrt(df[JS_SURGE]**2 + df[JS_SWAY]**2)
    return df


def load_mss(path):
    df = pd.read_csv(path, comment="#")
    required = [MSS_TIME, MSS_NORTH, MSS_EAST, MSS_HEADING]
    missing = [c for c in required if c not in df.columns]
    if missing:
        sys.exit(f"MSS CSV missing columns: {missing}\n"
                 f"Found: {list(df.columns)}")
    return df.sort_values(MSS_TIME).reset_index(drop=True)


# ── alignment ─────────────────────────────────────────────────────────────────

def align(js, mss):
    """
    Interpolate MSS onto JS timestamps over their shared time window.
    Returns a single aligned DataFrame plus a list of diagnostic warnings.
    """
    warnings = []
    t_js  = js[JS_TIME].values
    t_mss = mss[MSS_TIME].values

    t_start = max(t_js[0],  t_mss[0])
    t_end   = min(t_js[-1], t_mss[-1])

    if t_end <= t_start:
        sys.exit("Time windows do not overlap — check that both CSVs cover the same scenario.")

    overlap_pct = (t_end - t_start) / max(t_js[-1] - t_js[0], 1e-9) * 100
    if overlap_pct < 80:
        warnings.append(
            f"Only {overlap_pct:.0f}% time overlap between JS ({t_js[-1]:.1f}s) "
            f"and MSS ({t_mss[-1]:.1f}s) — metrics are for the shared window only."
        )

    mask_js = (t_js >= t_start) & (t_js <= t_end)
    t_common = t_js[mask_js]

    def interp_col(df, t_src, col):
        return interp1d(t_src, df[col].values,
                        kind="linear", bounds_error=False,
                        fill_value="extrapolate")(t_common)

    aligned = pd.DataFrame({
        "time":        t_common,
        "js_north":    js.loc[mask_js, JS_NORTH].values,
        "js_east":     js.loc[mask_js, JS_EAST].values,
        "js_heading":  js.loc[mask_js, JS_HEADING].values,
        "js_speed":    js.loc[mask_js, JS_SPEED].values,
        "mss_north":   interp_col(mss, t_mss, MSS_NORTH),
        "mss_east":    interp_col(mss, t_mss, MSS_EAST),
        "mss_heading": interp_col(mss, t_mss, MSS_HEADING),
        "mss_speed":   interp_col(mss, t_mss, MSS_SPEED),
    })

    return aligned, t_start, t_end, warnings


# ── metrics ───────────────────────────────────────────────────────────────────

def angle_diff(a, b):
    """Signed angular difference, wrapped to [-π, π]."""
    d = a - b
    return np.arctan2(np.sin(d), np.cos(d))


def compute_metrics(aligned):
    pos_err = np.sqrt(
        (aligned["js_north"] - aligned["mss_north"])**2 +
        (aligned["js_east"]  - aligned["mss_east"])**2
    )
    heading_err = np.abs(angle_diff(
        aligned["js_heading"].values,
        aligned["mss_heading"].values
    ))
    speed_err = np.abs(aligned["js_speed"] - aligned["mss_speed"])

    return {
        "pos_rmse":       float(np.sqrt(np.mean(pos_err**2))),
        "pos_max":        float(pos_err.max()),
        "pos_final":      float(pos_err.iloc[-1]),
        "pos_at_10s":     float(pos_err[aligned["time"] <= aligned["time"].iloc[0] + 10].iloc[-1]),
        "pos_at_30s":     float(pos_err[aligned["time"] <= aligned["time"].iloc[0] + 30].iloc[-1])
                          if aligned["time"].iloc[-1] - aligned["time"].iloc[0] >= 30 else None,
        "heading_rmse":   float(np.sqrt(np.mean(heading_err**2))),
        "heading_max":    float(heading_err.max()),
        "speed_rmse":     float(np.sqrt(np.mean(speed_err**2))),
        "speed_max":      float(speed_err.max()),
        "divergence_rate": None,  # filled below
        "pos_err_series": pos_err,
    }


def compute_divergence_rate(aligned, pos_err):
    """Fit a linear trend to position error over time to get m/s divergence rate."""
    t = aligned["time"].values - aligned["time"].values[0]
    coeffs = np.polyfit(t, pos_err.values, 1)
    return float(coeffs[0])  # metres of error per second of simulation


# ── plot ──────────────────────────────────────────────────────────────────────

COLORS = {
    "js":      "#56f39a",
    "mss":     "#ff6b5f",
    "error":   "#a9ecff",
    "wp":      "#ffd166",
    "bg":      "#0e1b22",
    "grid":    "#1f3540",
    "text":    "#edf7fb",
    "subtext": "#91aab2",
}

def apply_style(ax, title, xlabel, ylabel):
    ax.set_facecolor(COLORS["bg"])
    ax.set_title(title, color=COLORS["text"], fontsize=11, pad=8)
    ax.set_xlabel(xlabel, color=COLORS["subtext"], fontsize=9)
    ax.set_ylabel(ylabel, color=COLORS["subtext"], fontsize=9)
    ax.tick_params(colors=COLORS["subtext"], labelsize=8)
    ax.grid(True, color=COLORS["grid"], linewidth=0.6)
    for spine in ax.spines.values():
        spine.set_edgecolor(COLORS["grid"])
    ax.legend(fontsize=8, facecolor=COLORS["bg"],
              labelcolor=COLORS["text"], edgecolor=COLORS["grid"])


def make_figure(js, mss, aligned, metrics, t_start, t_end, warnings, out_path):
    fig = plt.figure(figsize=(16, 11), facecolor=COLORS["bg"])
    fig.suptitle("JS Simulator vs MSS Otter USV — Comparison Report",
                 color=COLORS["text"], fontsize=14, y=0.98)
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.42, wspace=0.35)

    # ── 1. Trajectory ──────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(js[JS_EAST],  js[JS_NORTH],
             color=COLORS["js"],  lw=2, label="JS Sim", zorder=3)
    ax1.plot(mss[MSS_EAST], mss[MSS_NORTH],
             color=COLORS["mss"], lw=2, ls="--", label="MSS Otter", zorder=3)
    # Start marker
    ax1.plot(js[JS_EAST].iloc[0], js[JS_NORTH].iloc[0],
             "o", color="white", ms=7, zorder=5, label="Start")
    # Waypoints
    for i, (wn, we) in enumerate(WAYPOINTS):
        ax1.plot(we, wn, "^", color=COLORS["wp"], ms=9, zorder=5)
        ax1.annotate(f"WP{i+1}", (we, wn),
                     textcoords="offset points", xytext=(6, 4),
                     color=COLORS["wp"], fontsize=8)
    # Shade shared time window on trajectory (just annotate end points)
    js_end = js[js[JS_TIME] <= t_end].iloc[-1]
    mss_end = mss[mss[MSS_TIME] <= t_end].iloc[-1]
    ax1.plot(js_end[JS_EAST],   js_end[JS_NORTH],
             "x", color=COLORS["js"],  ms=9, zorder=5)
    ax1.plot(mss_end[MSS_EAST], mss_end[MSS_NORTH],
             "x", color=COLORS["mss"], ms=9, zorder=5)
    apply_style(ax1, "Trajectory (East vs North)", "East (m)", "North (m)")
    ax1.set_aspect("equal")

    # ── 2. Position error over time ────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    pos_err = metrics["pos_err_series"]
    t_rel = aligned["time"] - aligned["time"].iloc[0]
    ax2.plot(t_rel, pos_err, color=COLORS["error"], lw=1.5, label="Position error")
    ax2.axhline(metrics["pos_rmse"], color=COLORS["mss"], ls="--", lw=1,
                label=f'RMSE = {metrics["pos_rmse"]:.2f} m')
    # Divergence trend line
    rate = metrics["divergence_rate"]
    trend = rate * t_rel.values + pos_err.values[0]
    ax2.plot(t_rel, trend, color=COLORS["wp"], ls=":", lw=1,
             label=f'Trend: {rate*1000:.1f} mm/s')
    apply_style(ax2, "Position Divergence", "Time (s)", "Error (m)")

    # ── 3. Speed comparison ────────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(aligned["time"], aligned["js_speed"],
             color=COLORS["js"],  lw=2, label="JS Sim")
    ax3.plot(aligned["time"], aligned["mss_speed"],
             color=COLORS["mss"], lw=2, ls="--", label="MSS")
    apply_style(ax3, "Surge Speed", "Time (s)", "Speed (m/s)")

    # ── 4. Heading comparison ──────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(aligned["time"], np.degrees(aligned["js_heading"]),
             color=COLORS["js"],  lw=2, label="JS Sim")
    ax4.plot(aligned["time"], np.degrees(aligned["mss_heading"]),
             color=COLORS["mss"], lw=2, ls="--", label="MSS")
    apply_style(ax4, "Heading", "Time (s)", "Heading (deg)")

    # ── 5. Heading error ───────────────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 2])
    h_err_deg = np.degrees(np.abs(angle_diff(
        aligned["js_heading"].values,
        aligned["mss_heading"].values
    )))
    ax5.plot(aligned["time"], h_err_deg,
             color=COLORS["error"], lw=1.5, label="Heading error")
    ax5.axhline(np.degrees(metrics["heading_rmse"]),
                color=COLORS["mss"], ls="--", lw=1,
                label=f'RMSE = {np.degrees(metrics["heading_rmse"]):.1f}°')
    apply_style(ax5, "Heading Divergence", "Time (s)", "Error (deg)")

    # ── 6. North component comparison ─────────────────────────────────────────
    ax6 = fig.add_subplot(gs[2, 0])
    ax6.plot(aligned["time"], aligned["js_north"],
             color=COLORS["js"],  lw=2, label="JS Sim")
    ax6.plot(aligned["time"], aligned["mss_north"],
             color=COLORS["mss"], lw=2, ls="--", label="MSS")
    apply_style(ax6, "North Position", "Time (s)", "North (m)")

    # ── 7. East component comparison ──────────────────────────────────────────
    ax7 = fig.add_subplot(gs[2, 1])
    ax7.plot(aligned["time"], aligned["js_east"],
             color=COLORS["js"],  lw=2, label="JS Sim")
    ax7.plot(aligned["time"], aligned["mss_east"],
             color=COLORS["mss"], lw=2, ls="--", label="MSS")
    apply_style(ax7, "East Position", "Time (s)", "East (m)")

    # ── 8. Summary text panel ──────────────────────────────────────────────────
    ax8 = fig.add_subplot(gs[2, 2])
    ax8.set_facecolor(COLORS["bg"])
    ax8.axis("off")
    summary_lines = [
        "── Metrics ─────────────────",
        f"  Pos RMSE:       {metrics['pos_rmse']:.3f} m",
        f"  Pos max error:  {metrics['pos_max']:.3f} m",
        f"  Pos at t+10s:   {metrics['pos_at_10s']:.3f} m",
    ]
    if metrics["pos_at_30s"] is not None:
        summary_lines.append(f"  Pos at t+30s:   {metrics['pos_at_30s']:.3f} m")
    summary_lines += [
        f"  Diverge rate:   {metrics['divergence_rate']*1000:.1f} mm/s",
        "",
        f"  Hdg RMSE:       {math.degrees(metrics['heading_rmse']):.2f}°",
        f"  Hdg max error:  {math.degrees(metrics['heading_max']):.2f}°",
        "",
        f"  Speed RMSE:     {metrics['speed_rmse']:.3f} m/s",
        f"  Speed max err:  {metrics['speed_max']:.3f} m/s",
    ]
    if warnings:
        summary_lines += ["", "── Warnings ─────────────────"]
        for w in warnings:
            # Word-wrap at ~32 chars
            words = w.split()
            line = ""
            for word in words:
                if len(line) + len(word) > 32:
                    summary_lines.append(f"  {line}")
                    line = word
                else:
                    line = f"{line} {word}".strip()
            if line:
                summary_lines.append(f"  {line}")

    ax8.text(0.05, 0.95, "\n".join(summary_lines),
             transform=ax8.transAxes,
             va="top", ha="left",
             fontsize=8.5,
             fontfamily="monospace",
             color=COLORS["text"])

    plt.savefig(out_path, dpi=150, bbox_inches="tight",
                facecolor=COLORS["bg"])
    print(f"Plot saved → {out_path}")


# ── text report ───────────────────────────────────────────────────────────────

def write_report(metrics, warnings, js, mss, aligned, t_start, t_end, path):
    lines = [
        "JS Simulator vs MSS Otter USV — Comparison Report",
        "=" * 52,
        "",
        f"JS sim duration:   {js[JS_TIME].iloc[-1]:.1f} s  ({len(js)} steps)",
        f"MSS duration:      {mss[MSS_TIME].iloc[-1]:.1f} s  ({len(mss)} steps)",
        f"Shared window:     {t_start:.1f} s → {t_end:.1f} s  ({len(aligned)} aligned steps)",
        "",
        "Position",
        "─" * 30,
        f"  RMSE:              {metrics['pos_rmse']:.4f} m",
        f"  Max error:         {metrics['pos_max']:.4f} m",
        f"  Final error:       {metrics['pos_final']:.4f} m",
        f"  Error at t+10s:    {metrics['pos_at_10s']:.4f} m",
    ]
    if metrics["pos_at_30s"] is not None:
        lines.append(f"  Error at t+30s:    {metrics['pos_at_30s']:.4f} m")
    lines += [
        f"  Divergence rate:   {metrics['divergence_rate']*1000:.2f} mm/s",
        "",
        "Heading",
        "─" * 30,
        f"  RMSE:              {math.degrees(metrics['heading_rmse']):.3f} deg",
        f"  Max error:         {math.degrees(metrics['heading_max']):.3f} deg",
        "",
        "Speed",
        "─" * 30,
        f"  RMSE:              {metrics['speed_rmse']:.4f} m/s",
        f"  Max error:         {metrics['speed_max']:.4f} m/s",
        "",
        "Interpretation",
        "─" * 30,
    ]

    # Auto-interpretation
    rate_mm = metrics["divergence_rate"] * 1000
    if rate_mm < 5:
        lines.append("  Divergence rate < 5 mm/s: trajectories closely matched.")
    elif rate_mm < 20:
        lines.append("  Divergence rate 5–20 mm/s: moderate divergence, likely")
        lines.append("  from integrator differences or drag model tuning.")
    else:
        lines.append("  Divergence rate > 20 mm/s: significant divergence.")
        lines.append("  Check axis convention, drag coefficients, or wave model.")

    rmse = metrics["pos_rmse"]
    if rmse < 1.0:
        lines.append(f"  Position RMSE {rmse:.2f} m: excellent agreement.")
    elif rmse < 5.0:
        lines.append(f"  Position RMSE {rmse:.2f} m: good agreement for a USV sim.")
    elif rmse < 15.0:
        lines.append(f"  Position RMSE {rmse:.2f} m: acceptable — guidance law")
        lines.append("  differences likely dominate over physics differences.")
    else:
        lines.append(f"  Position RMSE {rmse:.2f} m: large. The sims may be using")
        lines.append("  different axis conventions — check north/east mapping.")

    if warnings:
        lines += ["", "Warnings", "─" * 30]
        for w in warnings:
            lines.append(f"  {w}")

    lines += [
        "",
        "Notes",
        "─" * 30,
        "  The MSS baseline used a simple P-controller guidance law, while",
        "  the JS sim uses a more sophisticated heuristic skipper. Heading",
        "  and speed differences reflect guidance strategy, not just physics.",
        "  For a pure physics comparison, run both sims with identical",
        "  open-loop thrust/rudder commands (see mss_benchmark.py --js_log).",
    ]

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Report saved → {path}")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compare JS simulator CSV against MSS baseline CSV."
    )
    parser.add_argument("--js",  required=True, help="Path to JS sim CSV export")
    parser.add_argument("--mss", required=True, help="Path to MSS baseline CSV")
    parser.add_argument("--out", default="comparison_report",
                        help="Output filename stem (no extension)")
    args = parser.parse_args()

    print(f"Loading JS log:  {args.js}")
    js = load_js(args.js)
    print(f"  {len(js)} steps, {js[JS_TIME].iloc[-1]:.1f} s")

    print(f"Loading MSS log: {args.mss}")
    mss = load_mss(args.mss)
    print(f"  {len(mss)} steps, {mss[MSS_TIME].iloc[-1]:.1f} s")

    print("Aligning on shared time window...")
    aligned, t_start, t_end, warnings = align(js, mss)
    print(f"  Shared window: {t_start:.1f} – {t_end:.1f} s ({len(aligned)} points)")

    if warnings:
        for w in warnings:
            print(f"  ⚠  {w}")

    print("Computing metrics...")
    metrics = compute_metrics(aligned)
    metrics["divergence_rate"] = compute_divergence_rate(
        aligned, metrics["pos_err_series"]
    )

    print("\n── Results ──────────────────────────────────────")
    print(f"  Position RMSE:      {metrics['pos_rmse']:.3f} m")
    print(f"  Position max error: {metrics['pos_max']:.3f} m")
    print(f"  Heading RMSE:       {math.degrees(metrics['heading_rmse']):.2f}°")
    print(f"  Speed RMSE:         {metrics['speed_rmse']:.3f} m/s")
    print(f"  Divergence rate:    {metrics['divergence_rate']*1000:.1f} mm/s")
    print("─────────────────────────────────────────────────\n")

    plot_path   = f"{args.out}.png"
    report_path = f"{args.out}.txt"

    print("Generating plot...")
    make_figure(js, mss, aligned, metrics, t_start, t_end, warnings, plot_path)
    write_report(metrics, warnings, js, mss, aligned, t_start, t_end, report_path)

    print("\nDone.")


if __name__ == "__main__":
    main()
