from __future__ import annotations

import hashlib
import html
import json
import subprocess
import base64
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = ROOT / "artifacts" / "figures"
CONTRACT = ROOT / "artifacts" / "rl-campaign" / "surveyor" / "task-contract-frozen.json"
FONT_SHA256 = "bd47314d301e50ff4d109bff28dfcf637cb7eb13945480259878b848875acc65"

COLORS = {
    "ink": "#172033", "muted": "#667085", "paper": "#ffffff",
    "grid": "#d0d5dd", "identical": "#2f855a", "divergence": "#d97706",
    "na": "#e4e7ec", "negative": "#b42318", "blue": "#2457a7",
}


def require_files(paths: Iterable[Path]) -> list[Path]:
    resolved = [Path(p).resolve() for p in paths]
    missing = [str(p) for p in resolved if not p.is_file()]
    if missing:
        raise FileNotFoundError("Missing committed figure input(s): " + ", ".join(missing))
    for path in resolved:
        try:
            relative = path.relative_to(ROOT)
        except ValueError as exc:
            raise ValueError(f"Figure input must be inside repository: {path}") from exc
        tracked = subprocess.run(
            ["git", "ls-files", "--error-unmatch", str(relative)], cwd=ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        # Newly generated artifacts are renderable before the surrounding change is
        # committed, but only from the repository artifact tree and only when Git
        # does not ignore them. A clean checkout therefore either contains the exact
        # input or fails with the path above; no live fallback exists.
        if tracked.returncode:
            ignored = subprocess.run(
                ["git", "check-ignore", "-q", str(relative)], cwd=ROOT,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            ).returncode == 0
            if ROOT / "artifacts" not in path.parents or ignored:
                raise ValueError(f"Figure input is not a committable artifact: {relative}")
    return resolved


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_sha() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def contract_hash(paths: Iterable[Path]) -> str | None:
    candidates = [Path(p) for p in paths]
    if CONTRACT.is_file():
        candidates.append(CONTRACT)
    for path in candidates:
        if path.suffix == ".json":
            value = load_json(path)
            if isinstance(value, dict):
                for key in ("content_sha256", "contract_sha256", "contract_hash"):
                    if isinstance(value.get(key), str):
                        return value[key]
    return None


def provenance(figure: str, paths: Iterable[Path]) -> dict:
    sources = require_files(paths)
    return {
        "schema_version": 1,
        "figure": figure,
        "sources": [
            {"path": str(p.relative_to(ROOT)), "sha256": _sha256(p)} for p in sources
        ],
        "contract_hash": contract_hash(sources),
        "git_sha": git_sha(),
        "render_resources": {"embedded_font": "Minecraft.ttf", "sha256": FONT_SHA256},
    }


def embedded_font_css() -> str:
    matches = sorted((ROOT / ".venv" / "lib").glob(
        "python*/site-packages/gymnasium/envs/toy_text/font/Minecraft.ttf"
    ))
    if not matches:
        raise FileNotFoundError("Missing locked rendering font: gymnasium/envs/toy_text/font/Minecraft.ttf")
    payload = matches[0].read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != FONT_SHA256:
        raise ValueError(f"Rendering font hash mismatch: expected {FONT_SHA256}, found {digest}")
    encoded = base64.b64encode(payload).decode("ascii")
    return ("@font-face{font-family:'BCOD Figure';src:url(data:font/ttf;base64," + encoded + ") "
            "format('truetype');font-weight:100 900;font-style:normal}"
            "text{font-family:'BCOD Figure',sans-serif}")


def esc(value: object) -> str:
    return html.escape(str(value), quote=True)


def svg_text(x: float, y: float, value: object, *, size: int = 14,
             weight: int = 400, anchor: str = "start", fill: str | None = None,
             rotate: float | None = None) -> str:
    transform = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
    return (f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" '
            f'text-anchor="{anchor}" fill="{fill or COLORS["ink"]}"{transform}>{esc(value)}</text>')


def write_svg(figure: str, body: str, width: int, height: int,
              sources: Iterable[Path], description: str) -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    prov = provenance(figure, sources)
    metadata = esc(json.dumps(prov, sort_keys=True, separators=(",", ":")))
    # The SVG carries its own font payload marker and uses a deterministic generic
    # fallback. Text remains selectable; PDF conversion can outline/embed it.
    document = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" role="img">\n'
        f'<title>{esc(figure)}</title><desc>{esc(description)}</desc>'
        f'<metadata id="provenance">{metadata}</metadata>\n'
        f'<style>{embedded_font_css()} .hairline{{shape-rendering:crispEdges}}</style>\n'
        f'<rect width="100%" height="100%" fill="{COLORS["paper"]}"/>\n{body}\n</svg>\n'
    )
    output = OUTPUT_DIR / f"{figure}.svg"
    output.write_text(document, encoding="utf-8", newline="\n")
    sidecar = OUTPUT_DIR / f"{figure}.provenance.json"
    sidecar.write_text(json.dumps(prov, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    return output


def assert_eligible(row: dict, *, portable: bool | None = None,
                    host_class: str | None = None) -> None:
    if portable is not None and row.get("task_portable") is not portable:
        raise ValueError(f"Ineligible row: expected task_portable={portable}: {row}")
    if host_class is not None and row.get("host_class") != host_class:
        raise ValueError(f"Ineligible row: expected host_class={host_class}: {row}")
