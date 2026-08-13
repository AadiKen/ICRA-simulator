#!/usr/bin/env python3
"""Exact local execution of upstream `load_hsva` Kedro node semantics.

The archived upstream project pins Kedro 0.17.6/Python 3.7 and cannot run in the
platform's pinned Python 3.12+ environment. This executes its two pure transform
nodes without changing their behavior and records that compatibility route.
"""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "validation/datasets/raw/kvlcc2-simman2008-third-party/kvlcc2-hsva"
OUTPUT = ROOT / "validation/datasets/processed/kvlcc2-simman2008-third-party/kvlcc2-hsva"
UPSTREAM_ARCHIVE_SHA256 = "65afd940aeed546f610acfc15eb862d21ee58b5fa4261e86e4116d16026722f6"
COLUMNS = ["time", "x0", "y0", "psi", "u", "v", "p", "r", "delta", "rev", "thrust", "torque"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    rows = []
    names = []
    for source in sorted(SOURCE.glob("HSVA_CPMC_KVLCC2_*.dat")):
        frame = pd.read_csv(source, header=4, encoding="cp1252", sep=r"\s+", index_col=0, names=COLUMNS)
        frame[["delta", "psi", "r"]] = frame[["delta", "psi", "r"]].apply(lambda column: column * 3.141592653589793 / 180.0)
        frame.index -= frame.index[0]
        destination = OUTPUT / f"{source.stem}.csv"
        frame.to_csv(destination, index=True, encoding="utf8")
        names.append(source.stem)
        rows.append({"id": source.stem, "source_sha256": sha256(source), "processed_sha256": sha256(destination), "rows": len(frame), "columns": list(frame.columns)})
    (OUTPUT / "run_yml.yml").write_text("\n".join(f"- {name}" for name in names) + "\n", encoding="utf8")
    manifest = {
        "schema_version": 1,
        "status": "preprocessed-cache-only",
        "pipeline": "kvlcc2_hsva_create/load_hsva",
        "execution": "upstream Kedro 0.17.6 pure-node compatibility execution",
        "upstream_revision": "289a9e6 (Zenodo version 5 archive)",
        "upstream_archive_sha256": UPSTREAM_ARCHIVE_SHA256,
        "redistribution_permitted": False,
        "gating_permitted": False,
        "runs": rows,
    }
    (OUTPUT / "preprocessing-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf8")
    tracked_manifest = ROOT / "validation/external-references/kvlcc2-marin/hsva-preprocessing-manifest.json"
    tracked_manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf8")
    print(json.dumps({"output": str(OUTPUT), "runs": len(rows), "rows": sum(row["rows"] for row in rows)}, indent=2))


if __name__ == "__main__":
    main()
