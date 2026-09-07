from __future__ import annotations

from pathlib import Path

from .common import COLORS, ROOT, load_json, svg_text, write_svg

DEFAULT = ROOT / "artifacts" / "publication" / "offline-platform-report.json"

PLUGIN_PACKAGES = [
    {"id": "environment", "path": "packages/environment", "role": "environment sources and forcing"},
    {"id": "sensor-sdk", "path": "packages/sensor-sdk", "role": "sensor plugins and observations"},
    {"id": "vehicle-sdk", "path": "packages/vehicle-sdk", "role": "vehicle, plant, and actuator plugins"},
]
EXECUTION_PACKAGES = [
    {"id": "core", "path": "packages/core", "role": "deterministic execution core"},
    {"id": "backends", "path": "backends/", "role": "Node and tensor execution adapters"},
]
OUTPUT_PACKAGES = [
    {"id": "metrics", "path": "packages/metrics", "role": "benchmark metrics"},
    {"id": "experiment-schema", "path": "packages/experiment-schema", "role": "run contracts and artifact schema"},
]


def _box(x: int, y: int, width: int, height: int, title: str, path: str,
         role: str, *, fill: str = "#f8fafc", stroke: str = "#98a2b3") -> str:
    return "\n".join([
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" rx="9" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>',
        svg_text(x + 16, y + 25, title, size=14, weight=700),
        svg_text(x + 16, y + 45, path, size=10, fill=COLORS["blue"]),
        svg_text(x + 16, y + 66, role, size=10, fill=COLORS["muted"]),
    ])


def build(data_paths=None) -> Path:
    paths = [Path(p) for p in (data_paths or [DEFAULT])]
    release = load_json(paths[0])
    if release.get("completed", {}).get("core_migration") is not True:
        raise ValueError("Figure 0 requires a release artifact confirming the migrated execution core")
    plugins = PLUGIN_PACKAGES
    execution = EXECUTION_PACKAGES
    outputs = OUTPUT_PACKAGES
    for item in plugins + execution + outputs:
        package_path = ROOT / item["path"]
        if not package_path.is_dir():
            raise FileNotFoundError(f"Figure 0 package path is absent: {package_path}")
        if item["id"] != "backends" and not (package_path / "package.json").is_file():
            raise FileNotFoundError(f"Figure 0 package manifest is absent: {package_path / 'package.json'}")

    width, height = 1160, 650
    body = [svg_text(28, 38, "BCOD-Sim architecture", size=23, weight=700),
            svg_text(28, 62, "Plugin boundaries around a deterministic execution and measurement path",
                     size=13, fill=COLORS["muted"])]
    body.append('<rect x="24" y="90" width="310" height="438" rx="13" fill="#f3f7fd" stroke="#90b4e8" stroke-width="1.5" stroke-dasharray="6 4"/>')
    body.append(svg_text(45, 120, "PLUGIN BOUNDARY", size=12, weight=700, fill=COLORS["blue"]))
    for i, item in enumerate(plugins):
        body.append(_box(45, 143 + i * 120, 268, 91, item["id"], item["path"], item["role"],
                         fill="#ffffff", stroke="#90b4e8"))

    core = next(x for x in execution if x["id"] == "core")
    backend = next(x for x in execution if x["id"] == "backends")
    body.append(_box(423, 183, 306, 112, core["id"], core["path"], core["role"],
                     fill="#eef8f1", stroke=COLORS["identical"]))
    body.append(_box(423, 344, 306, 100, backend["id"], backend["path"], backend["role"],
                     fill="#fff7ed", stroke=COLORS["divergence"]))
    for item in outputs:
        y = 183 if item["id"] == "metrics" else 344
        body.append(_box(838, y, 292, 100, item["id"], item["path"], item["role"],
                         fill="#f9f5ff", stroke="#9b7ed1"))

    arrow = '<path d="M {x1} {y1} L {x2} {y2}" fill="none" stroke="#475467" stroke-width="2" marker-end="url(#arrow)"/>'
    body.insert(2, '<defs><marker id="arrow" markerWidth="9" markerHeight="7" refX="8" refY="3.5" orient="auto"><polygon points="0 0, 9 3.5, 0 7" fill="#475467"/></marker></defs>')
    for y in (188, 308, 428):
        body.append(arrow.format(x1=313, y1=y, x2=423, y2=239))
    body.append(arrow.format(x1=576, y1=295, x2=576, y2=344))
    body.append(arrow.format(x1=729, y1=239, x2=838, y2=233))
    body.append(arrow.format(x1=729, y1=394, x2=838, y2=394))
    body.append(svg_text(365, 151, "configuration + plugins", size=10, fill=COLORS["muted"]))
    body.append(svg_text(750, 217, "state / events", size=10, fill=COLORS["muted"]))
    body.append(svg_text(747, 379, "run records", size=10, fill=COLORS["muted"]))
    caption = ("Environment, sensor, and vehicle plugins enter through SDK boundaries. The deterministic core "
               "executes through backend adapters; metrics and experiment-schema produce benchmark evidence and reproducible artifacts.")
    body.append(svg_text(28, 573, caption, size=11, fill=COLORS["muted"]))
    body.append(svg_text(28, 597, "Package paths reflect the repository structure; release state is read from the committed platform artifact.",
                         size=10, fill=COLORS["muted"]))
    return write_svg("fig0_architecture", "\n".join(body), width, height, paths, caption)
