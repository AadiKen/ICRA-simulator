from __future__ import annotations

import re
from pathlib import Path

from .common import COLORS, ROOT, load_json, svg_text, write_svg

DISCIPLINE = ROOT / "artifacts" / "extensibility-discipline" / "report.md"
SWEEP = ROOT / "artifacts" / "extensibility-discipline" / "imu-sea-state-sweep.json"


def _discipline(text: str) -> tuple[str, list[dict]]:
    hashes = re.findall(r"`([0-9a-f]{64})`", text)
    if len(hashes) < 1 or "pre-task and post-task hashes are both" not in text:
        raise ValueError("Figure 3 requires explicit equal pre/post deterministic-core hash evidence")
    rows = []
    for sensor in ("hygrometer", "fan-rangefinder"):
        section = text.split(f"### `{sensor}`", 1)[1].split("### ", 1)[0]
        totals = re.search(r"Total additions: (\d+) lines?; total removals: (\d+) lines?", section)
        core = re.search(r"Core files modified by this addition: (\d+)", section)
        if not totals or not core:
            raise ValueError(f"Missing diff-stat evidence for {sensor}")
        rows.append({"sensor": sensor, "added": int(totals.group(1)), "removed": int(totals.group(2)), "core_files": int(core.group(1))})
    return hashes[0], rows


def build(data_paths=None) -> Path:
    paths = [Path(p) for p in (data_paths or [DISCIPLINE, SWEEP])]
    core_hash, diffs = _discipline(paths[0].read_text(encoding="utf-8"))
    sweep = load_json(paths[1])
    cells = sweep.get("cells", [])
    if len(cells) < 2 or any(cells[i]["beaufort"] >= cells[i+1]["beaufort"] for i in range(len(cells)-1)):
        raise ValueError("Figure 3 requires an ordered multi-cell sea-state sweep")

    width, height = 1220, 620
    body = [svg_text(28, 39, "Extensible sensing without core changes", size=23, weight=700),
            svg_text(28, 64, "Two plugin additions and an offline IMU environmental-response sweep", size=13, fill=COLORS["muted"])]
    body.append(svg_text(35, 108, "A · Add-a-sensor diff", size=15, weight=700))
    body.append('<rect x="28" y="126" width="480" height="310" rx="9" fill="#f8fafc" stroke="#cbd5e1"/>')
    for x,label in zip((48,225,315,400),("Plugin","Added","Removed","Core files")):
        body.append(svg_text(x,155,label,size=11,weight=700))
    for i,row in enumerate(diffs):
        y=184+i*65
        body.append(f'<rect x="42" y="{y-23}" width="448" height="52" fill="{("#ffffff" if i==0 else "#f1f5f9")}"/>')
        body.append(svg_text(48,y,row["sensor"],size=12,weight=700))
        body.append(svg_text(241,y,row["added"],size=13,weight=700,anchor="middle",fill=COLORS["blue"]))
        body.append(svg_text(337,y,row["removed"],size=13,weight=700,anchor="middle"))
        body.append(svg_text(435,y,row["core_files"],size=13,weight=700,anchor="middle",fill=COLORS["identical"]))
    body.append(svg_text(48,330,"Deterministic core hash",size=11,weight=700))
    body.append(svg_text(48,353,core_hash[:32],size=10,fill=COLORS["blue"]))
    body.append(svg_text(48,371,core_hash[32:],size=10,fill=COLORS["blue"]))
    body.append(svg_text(48,405,"UNCHANGED",size=14,weight=700,fill=COLORS["identical"]))

    body.append(svg_text(554,108,"B · IMU noise responds to sea state",size=15,weight=700))
    x0,y0,pw,ph=590,145,580,300
    body.append(f'<rect x="{x0}" y="{y0}" width="{pw}" height="{ph}" fill="#ffffff" stroke="#cbd5e1"/>')
    max_acc=max(c["declared_accel_noise_std_mps2"] for c in cells)
    max_gyro=max(c["declared_gyro_noise_std_rad_s"] for c in cells)
    for tick in range(5):
        y=y0+ph-tick*ph/4
        body.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0+pw}" y2="{y:.1f}" stroke="{COLORS["grid"]}"/>')
        body.append(svg_text(x0-8,y+4,f"{tick*max_acc/4:.3f}",size=9,anchor="end"))
    def points(key: str, maximum: float) -> str:
        return " ".join(f'{x0+i*pw/(len(cells)-1):.1f},{y0+ph-c[key]/maximum*ph:.1f}' for i,c in enumerate(cells))
    body.append(f'<polyline points="{points("declared_accel_noise_std_mps2",max_acc)}" fill="none" stroke="{COLORS["blue"]}" stroke-width="3"/>')
    # Gyro uses its own right-hand scale, normalized to the same panel height.
    body.append(f'<polyline points="{points("declared_gyro_noise_std_rad_s",max_gyro)}" fill="none" stroke="{COLORS["divergence"]}" stroke-width="3" stroke-dasharray="7 4"/>')
    for i,c in enumerate(cells):
        x=x0+i*pw/(len(cells)-1)
        body.append(svg_text(x,y0+ph+24,c["beaufort"],size=10,anchor="middle"))
    body.append(svg_text(x0+pw/2,y0+ph+48,"Beaufort sea state",size=11,anchor="middle"))
    body.append(svg_text(x0-48,y0+ph/2,"accel σ (m/s²)",size=10,anchor="middle",rotate=-90))
    body.append(svg_text(x0+pw+42,y0+ph/2,"gyro σ (rad/s)",size=10,anchor="middle",rotate=90))
    body.append(f'<line x1="690" y1="486" x2="720" y2="486" stroke="{COLORS["blue"]}" stroke-width="3"/>')
    body.append(svg_text(728,490,"accelerometer declared noise σ",size=10))
    body.append(f'<line x1="935" y1="486" x2="965" y2="486" stroke="{COLORS["divergence"]}" stroke-width="3" stroke-dasharray="7 4"/>')
    body.append(svg_text(973,490,"gyroscope declared noise σ",size=10))
    body.append(svg_text(554,533,f'Fixed truth · seed {sweep["protocol"]["seed"]} · {sweep["protocol"]["samples_per_cell"]} samples/cell · actual ImuPlugin + HullMotionPlatformStateService',size=10,fill=COLORS["muted"]))
    caption=("Hygrometer and fan-rangefinder were added entirely outside the deterministic core. "
             "With truth, scenario, configuration, and random seed fixed, the production IMU plugin's configured noise floor increases with sea state.")
    body.append(svg_text(28,582,caption,size=11,fill=COLORS["muted"]))
    return write_svg("fig3_sensing","\n".join(body),width,height,paths,caption)
