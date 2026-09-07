from __future__ import annotations

from pathlib import Path

from .common import COLORS, ROOT, load_json, svg_text, write_svg

DEFAULT = ROOT / "artifacts" / "determinism-sweep" / "report.json"


def build(data_paths=None) -> Path:
    paths = [Path(p) for p in (data_paths or [DEFAULT])]
    report = load_json(paths[0])
    cells = report.get("cells")
    if not isinstance(cells, list) or len(cells) != 55:
        raise ValueError(f"Figure 4 requires exactly 55 cells; found {len(cells or [])}")

    columns = []
    for cell in cells:
        key = (cell["vehicle"], cell["plant"])
        if key not in columns:
            columns.append(key)
    axes = []
    for cell in cells:
        if cell["axis"] not in axes:
            axes.append(cell["axis"])

    index = {(c["axis"], c["vehicle"], c["plant"]): c for c in cells}
    left, top, cw, ch = 315, 108, 132, 43
    width, height = left + cw * len(columns) + 38, top + ch * len(axes) + 126
    out = [svg_text(24, 34, "Determinism under controlled perturbations", size=22, weight=700)]
    out.append(svg_text(24, 58, "55-cell sweep · perturbation axis × vehicle/model", size=13, fill=COLORS["muted"]))
    labels = {"passed": (COLORS["identical"], "IDENTICAL"),
              "expected-float-divergence": (COLORS["divergence"], "EXPECTED"),
              "not-applicable": (COLORS["na"], "N/A"),
              "negative-control-detected": (COLORS["negative"], "DETECTED")}
    for j, (vehicle, plant) in enumerate(columns):
        x = left + j * cw + cw / 2
        out.append(svg_text(x, 78, vehicle.replace("vehicle-", ""), size=11, weight=700, anchor="middle"))
        out.append(svg_text(x, 94, plant, size=10, anchor="middle", fill=COLORS["muted"]))
    for i, axis in enumerate(axes):
        y = top + i * ch
        out.append(svg_text(left - 12, y + 27, axis.replace("-", " "), size=11, anchor="end"))
        for j, (vehicle, plant) in enumerate(columns):
            cell = index.get((axis, vehicle, plant))
            if cell is None:
                continue
            status = cell["status"]
            color, label = labels.get(status, ("#ffffff", status.upper()))
            x = left + j * cw
            text_color = COLORS["ink"] if status == "not-applicable" else "#ffffff"
            out.append(f'<rect class="hairline" x="{x}" y="{y}" width="{cw-3}" height="{ch-3}" rx="4" fill="{color}"/>')
            out.append(svg_text(x + (cw-3)/2, y + 18, label, size=9, weight=700, anchor="middle", fill=text_color))
            if status == "expected-float-divergence":
                out.append(svg_text(x + (cw-3)/2, y + 32, f'{cell["divergence_magnitude"]:.3e}', size=9, anchor="middle", fill=text_color))
            elif status == "not-applicable":
                reason = str(cell.get("notes", "reason recorded"))
                short = "tensor: planar3 only" if "Tensor" in reason else "requires coupled6"
                out.append(svg_text(x + (cw-3)/2, y + 32, short, size=8, anchor="middle", fill=COLORS["muted"]))

    legend_y = top + len(axes) * ch + 24
    cursor = 24
    for status in ("passed", "expected-float-divergence", "not-applicable", "negative-control-detected"):
        color, label = labels[status]
        out.append(f'<rect x="{cursor}" y="{legend_y-13}" width="15" height="15" rx="2" fill="{color}" stroke="{COLORS["grid"]}"/>')
        out.append(svg_text(cursor + 22, legend_y, label.lower().replace("detected", "negative control detected"), size=10))
        cursor += 190
    caption = ("Bit-identical compares SHA-256 digests of canonical full trace rows (step, truth, observation, "
               "metrics, and info); the sensor toggle compares truth only and Node↔tensor uses its reduced-state contract. "
               "The Node↔tensor cell reports maximum divergence; N/A cells retain their reason.")
    out.append(svg_text(24, legend_y + 35, caption, size=11, fill=COLORS["muted"]))
    return write_svg("fig4_determinism", "\n".join(out), width, height, paths, caption)
