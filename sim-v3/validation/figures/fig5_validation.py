from __future__ import annotations

from pathlib import Path

from .common import COLORS, ROOT, load_json, svg_text, write_svg

BENCHMARK = ROOT / "artifacts" / "baselines" / "usv-bench-policy-report.json"
MARIN = ROOT / "validation" / "external-references" / "kvlcc2-marin" / "artifacts" / "kvlcc2-marin-trajectory-comparison.json"


def build_benchmark_half(*_args, **_kwargs):
    raise NotImplementedError(
        "Figure 5 benchmark heatmap is blocked by missing Vehicle B/C classical reference rates "
        "and the pending Vehicle A 1.5M phase-1 result; Gate 5 is resolved as two separate conditions"
    )


def build(data_paths=None) -> Path:
    paths = [Path(p) for p in (data_paths or [BENCHMARK, MARIN])]
    benchmark = load_json(paths[0])
    marin = load_json(paths[1])
    scope = benchmark["validation_scope"]
    required = {
        "vehicle-b-rudder": "model-structure-trajectory-scored-fail",
        "vehicle-c-azimuth": "allocation-demonstrated-dynamics-unvalidated",
    }
    for vehicle, verbatim in required.items():
        if scope.get(vehicle) != verbatim:
            raise ValueError(f"Figure 5 requires verbatim validation_scope[{vehicle}]={verbatim!r}")
    runs = marin["simulator_scoring"]["runs"]
    if len(runs) != 6:
        raise ValueError(f"Figure 5 expects six MARIN runs; found {len(runs)}")
    failed = sum(not row["passed"] for row in runs)
    errors = [metric["absolute_percent_error"] for row in runs for metric in row["imo"] if not metric["passed"]]

    width, height = 1200, 565
    body = [svg_text(28, 38, "Vehicle validation status", size=23, weight=700),
            svg_text(28, 63, "Physical-evidence scope is kept separate from software benchmark execution",
                     size=13, fill=COLORS["muted"])]
    columns = [28, 210, 610, 930]
    headings = ["Vehicle", "Validation scope (verbatim)", "Checked against", "Result"]
    body.append(f'<rect x="24" y="88" width="1150" height="45" fill="#eef2f6"/>')
    for x, label in zip(columns, headings):
        body.append(svg_text(x + 8, 117, label, size=11, weight=700))
    rows = [
        ("A · OTTER", scope["vehicle-a-otter"], "Official MSS Otter traces", "5 maneuver families; pinned tolerances"),
        ("B · rudder", scope["vehicle-b-rudder"], "6 measured MARIN KVLCC2 runs", f"{failed}/6 fail · metric error {min(errors):.2f}–{max(errors):.2f}%"),
        ("C · azimuth", scope["vehicle-c-azimuth"], "Allocation campaign; no matched vessel data", "Composability only; dynamics unvalidated"),
    ]
    for i, row in enumerate(rows):
        y = 133 + i * 96
        body.append(f'<rect x="24" y="{y}" width="1150" height="96" fill="{("#ffffff" if i % 2 == 0 else "#fafafa")}" stroke="{COLORS["grid"]}"/>')
        body.append(svg_text(columns[0] + 8, y + 31, row[0], size=13, weight=700))
        # Long status strings remain verbatim, wrapped only visually.
        status = row[1]
        if len(status) > 35:
            cut = status.rfind("-", 0, 35) + 1
            body.append(svg_text(columns[1] + 8, y + 27, status[:cut], size=10, weight=700))
            body.append(svg_text(columns[1] + 8, y + 45, status[cut:], size=10, weight=700))
        else:
            body.append(svg_text(columns[1] + 8, y + 36, status, size=10, weight=700))
        body.append(svg_text(columns[2] + 8, y + 34, row[2], size=10))
        body.append(svg_text(columns[3] + 8, y + 27, row[3], size=10))
    body.append(svg_text(28, 456, "MARIN gap", size=13, weight=700, fill=COLORS["negative"]))
    body.append(svg_text(132, 456, "measured rudder replay RMSE < 1e−12°; propeller replay RMSE = 0;", size=11))
    body.append(svg_text(132, 477, "turning advance errors 6.46% and 8.33% (5% limit); failed zig-zag overshoots 10.57%–212.86% (10% limit).", size=11))
    body.append(f'<rect x="24" y="500" width="1150" height="38" rx="5" fill="#fff7ed" stroke="#f2b66d"/>')
    body.append(svg_text(38, 524, "Benchmark heatmap intentionally omitted: B/C reference rates and the Vehicle A 1.5M phase-1 result remain unavailable.", size=11, weight=700, fill="#9a3412"))
    caption = ("Vehicle A is MSS validated at pinned tolerances. Vehicle B has measured model-scale MARIN evidence but fails fixed maneuver limits; "
               "Vehicle C demonstrates allocation/composability without matched behavioral validation. Status strings are reproduced verbatim.")
    return write_svg("fig5_validation", "\n".join(body), width, height, paths, caption)
