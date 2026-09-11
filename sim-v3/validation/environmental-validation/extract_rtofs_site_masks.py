#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import xarray as xr


ROOT = Path(__file__).resolve().parents[2]
SITES = {
    "honolulu": (21.289, -157.865),
    "miami": (25.731, -80.162),
    "boston": (42.354, -70.989),
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("rtofs", type=Path)
    parser.add_argument("--sites", nargs="+", choices=SITES, default=list(SITES))
    parser.add_argument("--radius-deg", type=float, default=0.5)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/environmental-validation")
    args = parser.parse_args()
    dataset = xr.open_dataset(args.rtofs)
    try:
        lat = np.asarray(dataset["Latitude"].values)
        lon = np.asarray(dataset["Longitude"].values)
        lon = ((lon + 180) % 360) - 180
        variable = "u_barotropic_velocity" if "u_barotropic_velocity" in dataset else "u"
        field = np.asarray(dataset[variable].isel({dim: 0 for dim in dataset[variable].dims if dim not in ("Y", "X")}).values)
        for site in args.sites:
            site_lat, site_lon = SITES[site]
            region = (abs(lat - site_lat) <= args.radius_deg) & (abs(lon - site_lon) <= args.radius_deg)
            indices = np.argwhere(region)
            if not len(indices):
                raise RuntimeError(f"RTOFS grid has no cells around {site}")
            cells = [{
                "rtofs_y": int(y), "rtofs_x": int(x),
                "latitude_deg": float(lat[y, x]), "longitude_deg": float(lon[y, x]),
                "rtofs_mask": "water" if math_isfinite(field[y, x]) else "land",
            } for y, x in indices]
            artifact = {
                "schema_version": 1, "artifact_kind": "rtofs-site-wet-mask-boundary",
                "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "site": site, "site_location": {"latitude_deg": site_lat, "longitude_deg": site_lon},
                "radius_deg": args.radius_deg,
                "method": f"Native RTOFS {variable} finite-value mask cropped around the site; no interpolation or accuracy scoring.",
                "rtofs_source": {"path": str(args.rtofs), "checksum_sha256": sha256(args.rtofs)},
                "cells": cells,
                "summary": {"cell_count": len(cells), "wet_cells": sum(c["rtofs_mask"] == "water" for c in cells), "land_cells": sum(c["rtofs_mask"] == "land" for c in cells)},
            }
            output = args.output_dir / f"rtofs-mask-{site}.json"
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(artifact, indent=2) + "\n")
            print(json.dumps({"output": str(output), **artifact["summary"]}))
    finally:
        dataset.close()


def math_isfinite(value: float) -> bool:
    return bool(np.isfinite(value))


if __name__ == "__main__":
    main()
