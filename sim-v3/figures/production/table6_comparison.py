from __future__ import annotations

from pathlib import Path

from .common import COLORS, ROOT, load_json, svg_text, write_svg

DEFAULT = ROOT / "artifacts" / "publication" / "table6-comparison.json"
ALLOWED_TAGS = {"measured", "cited", "not-obtainable"}


def validate_publication(data: dict) -> list[dict]:
    if data.get("artifact_kind") != "publication-competitor-comparison":
        raise ValueError("Table 6 requires a publication-competitor-comparison artifact")
    rows = data.get("rows")
    if not isinstance(rows, list) or not rows:
        raise ValueError("Table 6 requires at least one comparison row")
    for row in rows:
        if not isinstance(row.get("tool"), str):
            raise ValueError(f"Table 6 row lacks tool name: {row}")
        for claim_name in ("setup", "determinism"):
            claim = row.get(claim_name)
            if not isinstance(claim, dict) or not isinstance(claim.get("value"), str):
                raise ValueError(f"{row['tool']} lacks {claim_name} claim text")
            tag = claim.get("methodology")
            if tag not in ALLOWED_TAGS:
                raise ValueError(
                    f"{row['tool']} {claim_name} has invalid methodology tag {tag!r}; "
                    f"expected one of {sorted(ALLOWED_TAGS)}"
                )
    return rows


def _wrap(text: str, width: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    current = ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if current and len(candidate) > width:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    return lines


def _claim_cell(out: list[str], x: int, y: int, width: int, claim: dict) -> None:
    tag = claim["methodology"]
    tag_colors = {
        "measured": COLORS["identical"],
        "cited": COLORS["blue"],
        "not-obtainable": COLORS["muted"],
    }
    badge_width = 120 if tag == "not-obtainable" else 78
    out.append(f'<rect x="{x}" y="{y + 11}" width="{badge_width}" height="20" rx="10" fill="{tag_colors[tag]}"/>')
    out.append(svg_text(x + badge_width / 2, y + 25, tag.upper(), size=9, weight=700,
                        anchor="middle", fill="#ffffff"))
    for index, line in enumerate(_wrap(claim["value"], 54)[:4]):
        out.append(svg_text(x, y + 49 + index * 17, line, size=11))


def build(data_paths=None) -> Path:
    paths = [Path(p) for p in (data_paths or [DEFAULT])]
    if len(paths) != 1:
        raise ValueError("Table 6 accepts exactly one normalized publication artifact")
    data = load_json(paths[0])
    rows = validate_publication(data)

    left, tool_w, claim_w, row_h = 24, 180, 430, 126
    header_y = 76
    width = left * 2 + tool_w + claim_w * 2
    height = header_y + 38 + row_h * len(rows) + 76
    out = [svg_text(left, 34, "Table 6 · simulator comparison", size=22, weight=700)]
    out.append(svg_text(left, 58, "Every claim carries a normalized methodology tag", size=13,
                        fill=COLORS["muted"]))
    columns = [(left, tool_w, "Tool"), (left + tool_w, claim_w, "Setup / first step"),
               (left + tool_w + claim_w, claim_w, "Determinism evidence")]
    for x, col_w, label in columns:
        out.append(f'<rect x="{x}" y="{header_y}" width="{col_w}" height="38" fill="{COLORS["ink"]}"/>')
        out.append(svg_text(x + 10, header_y + 25, label, size=12, weight=700, fill="#ffffff"))

    for row_index, row in enumerate(rows):
        y = header_y + 38 + row_index * row_h
        shade = "#f8fafc" if row_index % 2 == 0 else COLORS["paper"]
        out.append(f'<rect class="hairline" x="{left}" y="{y}" width="{tool_w + claim_w * 2}" '
                   f'height="{row_h}" fill="{shade}" stroke="{COLORS["grid"]}"/>')
        out.append(svg_text(left + 10, y + 30, row["tool"], size=12, weight=700))
        _claim_cell(out, left + tool_w + 10, y, claim_w - 20, row["setup"])
        _claim_cell(out, left + tool_w + claim_w + 10, y, claim_w - 20, row["determinism"])
        out.append(f'<line class="hairline" x1="{left + tool_w}" y1="{y}" x2="{left + tool_w}" '
                   f'y2="{y + row_h}" stroke="{COLORS["grid"]}"/>')
        out.append(f'<line class="hairline" x1="{left + tool_w + claim_w}" y1="{y}" '
                   f'x2="{left + tool_w + claim_w}" y2="{y + row_h}" stroke="{COLORS["grid"]}"/>')

    caption_y = header_y + 38 + row_h * len(rows) + 28
    caption = ("Determinism statements apply only to the named tested component and scenario. "
               "VRX evidence establishes same-seed repeatability for its configured stochastic IMU; "
               "it is not a simulator-wide determinism claim.")
    for index, line in enumerate(_wrap(caption, 150)):
        out.append(svg_text(left, caption_y + index * 17, line, size=11, fill=COLORS["muted"]))
    return write_svg("table6_comparison", "\n".join(out), width, height, paths, caption)


if __name__ == "__main__":
    print(build())
