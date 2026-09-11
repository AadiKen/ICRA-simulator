#!/usr/bin/env python3
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from era5_common import DATASET, retrieve_wind, sha256, wind_rows


ROOT = Path(__file__).resolve().parents[2]
STATIONS = {
    "san-francisco": (37.759, -122.833, "46026"),
    "honolulu": (21.303, -157.865, "OOUH1"),
    "miami": (25.731, -80.162, "VAKF1"),
    "boston": (42.346, -70.651, "44013"),
}
SITE_LABELS = {
    "san-francisco": "San Francisco",
    "honolulu": "Honolulu",
    "miami": "Miami",
    "boston": "Boston",
}
START = "2026-07-13T00:00:00Z"
STOP = "2026-07-15T23:00:00Z"


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "bcod-sim ERA5 wind validation"})
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def ndbc_rows(payload: bytes) -> list[dict]:
    text = gzip.decompress(payload).decode()
    lines = [line for line in text.splitlines() if line.strip()]
    header_index = next(i for i, line in enumerate(lines) if line.startswith("#YY"))
    keys = lines[header_index].lstrip("#").split()
    rows = []
    for line in lines[header_index + 1 :]:
        if line.startswith("#"):
            continue
        values = line.split()
        get = lambda key: values[keys.index(key)]
        instant = datetime(
            int(get("YY")), int(get("MM")), int(get("DD")), int(get("hh")), int(get("mm")), tzinfo=timezone.utc
        )
        if not (datetime.fromisoformat(START.replace("Z", "+00:00")) <= instant <= datetime.fromisoformat(STOP.replace("Z", "+00:00"))):
            continue
        direction = get("WDIR")
        speed = get("WSPD")
        if direction in {"MM", "999"} or speed in {"MM", "99.0", "99"}:
            rows.append({"time": instant.isoformat().replace("+00:00", "Z"), "qc_pass": False})
            continue
        toward = math.radians(float(direction) + 180)
        magnitude = float(speed)
        rows.append(
            {
                "time": instant.isoformat().replace("+00:00", "Z"),
                "u_east_mps": magnitude * math.sin(toward),
                "v_north_mps": magnitude * math.cos(toward),
                "wind_speed_mps": magnitude,
                "wind_direction_from_deg": float(direction),
                "qc_pass": True,
            }
        )
    return rows


def scalar_stats(errors: list[float]) -> dict:
    absolute = np.abs(errors)
    return {
        "n": len(errors),
        "bias": float(np.mean(errors)),
        "mae": float(np.mean(absolute)),
        "rmse": float(np.sqrt(np.mean(np.square(errors)))),
        "p95_absolute_error": float(np.percentile(absolute, 95)),
    }


def vector_stats(rows: list[dict]) -> dict:
    u = [row["model_u"] - row["reference_u"] for row in rows]
    v = [row["model_v"] - row["reference_v"] for row in rows]
    speed = [math.hypot(row["model_u"], row["model_v"]) - math.hypot(row["reference_u"], row["reference_v"]) for row in rows]
    model = np.asarray([complex(row["model_u"], row["model_v"]) for row in rows])
    reference = np.asarray([complex(row["reference_u"], row["reference_v"]) for row in rows])
    model -= model.mean()
    reference -= reference.mean()
    denominator = math.sqrt(float(np.vdot(model, model).real * np.vdot(reference, reference).real))
    correlation = np.vdot(reference, model) / denominator if denominator else complex(float("nan"), float("nan"))
    return {
        "n": len(rows),
        "u": scalar_stats(u),
        "v": scalar_stats(v),
        "speed": scalar_stats(speed),
        "complex_correlation": {
            "magnitude": abs(correlation) if denominator else None,
            "phase_deg": math.degrees(math.atan2(correlation.imag, correlation.real)) if denominator else None,
            "real": correlation.real if denominator else None,
            "imaginary": correlation.imag if denominator else None,
            "definition": "mean-centered complex correlation",
        },
    }


def markdown_report(site: str, station: str, stats: dict | None, rejections: dict, artifact_name: str) -> str:
    excluded = rejections["ndbc_non_hourly_excluded"]
    failures = (
        rejections["ndbc_qc_failed"]
        + rejections["ndbc_invalid_vector_or_time"]
        + rejections["no_temporal_match"]
    )
    if stats is None:
        results = f"""No accuracy statistics can be reported because NDBC station {station} supplied zero
usable wind vectors in the requested window. Its {rejections['ndbc_qc_failed']} candidate records all
failed wind QC (the archive uses missing-value sentinels for wind direction and speed). This is a
source-data limitation, not an ERA5 accuracy result; a wind-reporting reference station must be
selected before this site can support the requested comparison.
"""
    else:
        correlation = stats["complex_correlation"]
        results = f"""| Metric | Value |
|---|---:|
| Matched hours | {stats['n']} |
| QC/temporal failures | {failures} |
| Wind-speed bias | {stats['speed']['bias']:+.3f} m/s |
| Wind-speed RMSE | {stats['speed']['rmse']:.3f} m/s |
| Eastward-component bias | {stats['u']['bias']:+.3f} m/s |
| Eastward-component RMSE | {stats['u']['rmse']:.3f} m/s |
| Northward-component bias | {stats['v']['bias']:+.3f} m/s |
| Northward-component RMSE | {stats['v']['rmse']:.3f} m/s |
| Complex-correlation magnitude | {correlation['magnitude']:.3f} |
| Complex-correlation phase | {correlation['phase_deg']:.3f} deg |

Match quality was {stats['n']}/{stats['n']}, with {failures} QC or temporal-match failures.
The {excluded} sub-hourly records are exclusions, not failed hourly matches.
"""
    return f"""# ERA5 versus NDBC wind validation — {SITE_LABELS[site]}

## Scope

ERA5 hourly 10 m wind vectors were compared with NDBC station {station} observations from
2026-07-13 00:00 UTC through 2026-07-15 23:00 UTC. Each QC-passing NDBC observation exactly
on the hour was paired with the nearest ERA5 grid cell at the identical timestamp. The {excluded}
sub-hourly station rows were excluded explicitly so that an ERA5 hour was not reused.

Hourly samples are serially correlated, so the report does not claim that the matched hours are
independent observations or provide an independent-sample confidence interval. GDOP is not
applicable to a single-station wind instrument.

## Results

{results}

The machine-readable report, including all matched vectors, rejection counts, source URLs,
and source checksums, is `artifacts/environmental-validation/{artifact_name}`.
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sites", nargs="+", choices=STATIONS, default=["san-francisco"])
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/environmental-validation/era5-ndbc-wind-20260713-15.json")
    parser.add_argument("--markdown-output", type=Path)
    args = parser.parse_args()
    all_matches = []
    sources = []
    rejections = {"ndbc_qc_failed": 0, "ndbc_invalid_vector_or_time": 0, "ndbc_non_hourly_excluded": 0, "no_temporal_match": 0}
    for site in args.sites:
        latitude, longitude, station = STATIONS[site]
        cache_site = "sf" if site == "san-francisco" else site
        cache = ROOT / ".cache/environmental-validation" / f"era5-wind-{cache_site}-20260713-15.nc"
        if not cache.exists():
            retrieve_wind(cache, year=2026, month=7, days=[13, 14, 15], times=[f"{hour:02d}:00" for hour in range(24)], area=[latitude + 0.5, longitude - 0.5, latitude - 0.5, longitude + 0.5])
        sources.append({"id": DATASET, "site": site, "version": "ERA5 hourly single levels", "url": "https://cds.climate.copernicus.eu/datasets/reanalysis-era5-single-levels", "checksum_sha256": sha256(cache)})
        archive_station = station.lower() if not station.isdigit() else station
        url = f"https://www.ndbc.noaa.gov/data/stdmet/Jul/{archive_station}72026.txt.gz"
        payload = fetch(url)
        source_path = ROOT / ".cache/environmental-validation" / f"ndbc-{station}-2026.txt.gz"
        source_path.parent.mkdir(parents=True, exist_ok=True)
        source_path.write_bytes(payload)
        sources.append({"id": f"NDBC-{station}", "version": "2026 historical standard meteorological", "url": url, "checksum_sha256": hashlib.sha256(payload).hexdigest()})
        model = {row["time"]: row for row in wind_rows(cache, latitude, longitude)}
        for observed in ndbc_rows(payload):
            if not observed.get("qc_pass"):
                rejections["ndbc_qc_failed"] += 1
                continue
            if not all(math.isfinite(observed[key]) for key in ("u_east_mps", "v_north_mps")):
                rejections["ndbc_invalid_vector_or_time"] += 1
                continue
            if datetime.fromisoformat(observed["time"].replace("Z", "+00:00")).minute != 0:
                rejections["ndbc_non_hourly_excluded"] += 1
                continue
            candidate = model.get(observed["time"])
            if candidate is None:
                rejections["no_temporal_match"] += 1
                continue
            all_matches.append({"site": site, "station": station, "time": observed["time"], "model_time": candidate["time"], "model_grid_id": f"era5:{candidate['latitude_deg']},{candidate['longitude_deg']}", "distance_m": None, "time_delta_s": 0, "model_u": candidate["u_east_mps"], "model_v": candidate["v_north_mps"], "reference_u": observed["u_east_mps"], "reference_v": observed["v_north_mps"], "reference_speed_mps": observed["wind_speed_mps"]})
    by_site_rows = {site: [row for row in all_matches if row["site"] == site] for site in args.sites}
    by_site = {site: vector_stats(rows) if rows else None for site, rows in by_site_rows.items()}
    overall = vector_stats(all_matches) if all_matches else None
    validation_status = "complete" if all(stats is not None for stats in by_site.values()) else "insufficient_reference_data"
    report = {
        "schema_version": 1,
        "artifact_kind": "era5-vs-ndbc-wind-validation",
        "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "validation_status": validation_status,
        "window_selection": {"start": START, "stop": STOP, "sites": args.sites},
        "sources": sources,
        "wind": {
            "matching_method": "Each QC-passing NDBC observation exactly on the hour is matched to the nearest ERA5 grid cell at the identical hour; sub-hourly station records are explicitly excluded rather than reusing an ERA5 hour.",
            "matching_tolerances": {"time_s": 0, "distance_m": None, "gdop": "not applicable to station wind"},
            "statistical_limitation": "Hourly samples are serially correlated; no independent-sample confidence interval is reported.",
            "rejections": rejections,
            "overall": overall,
            "by_site": by_site,
            "matches": all_matches,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    if args.markdown_output:
        if len(args.sites) != 1:
            parser.error("--markdown-output requires exactly one site")
        site = args.sites[0]
        station = STATIONS[site][2]
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown_report(site, station, by_site[site], rejections, args.output.name))
    print(json.dumps({"output": str(args.output), "status": validation_status, "matches": len(all_matches), "overall": overall}, indent=2))


if __name__ == "__main__":
    main()
