#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]


def rmse(errors: list[float]) -> float:
    return float(np.sqrt(np.mean(np.square(errors))))


def baselines(rows: list[dict], interval_hours: int, group_key: str | None = None) -> dict:
    groups = {"overall": rows}
    if group_key:
        groups.update({value: [row for row in rows if row[group_key] == value] for value in sorted({row[group_key] for row in rows})})
    output = {}
    for name, selected in groups.items():
        observed = {(
            row.get("latitude_deg", row.get("station")),
            row.get("longitude_deg", row.get("station")),
            row["time"],
        ): (row["reference_u"], row["reference_v"]) for row in selected}
        zero_errors = [math.hypot(row["reference_u"], row["reference_v"]) for row in selected]
        persistence_errors = []
        for row in selected:
            instant = datetime.fromisoformat(row["time"].replace("Z", "+00:00"))
            previous = instant.timestamp() - interval_hours * 3600
            previous_time = datetime.fromtimestamp(previous, timezone.utc).isoformat().replace("+00:00", "Z")
            key = (row.get("latitude_deg", row.get("station")), row.get("longitude_deg", row.get("station")), previous_time)
            if key in observed:
                pu, pv = observed[key]
                persistence_errors.append(math.hypot(pu, pv) - math.hypot(row["reference_u"], row["reference_v"]))
        output[name] = {
            "zero_baseline": {"n": len(zero_errors), "speed_rmse_mps": rmse(zero_errors)},
            "persistence_baseline": {"n": len(persistence_errors), "interval_hours": interval_hours, "speed_rmse_mps": rmse(persistence_errors)},
        }
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/environmental-validation/environmental-baselines-20260713-15.json")
    args = parser.parse_args()
    current_path = ROOT / "artifacts/environmental-validation/report-3dz-20260713-15.json"
    wind_paths = {
        "san-francisco": ROOT / "artifacts/environmental-validation/era5-ndbc-wind-20260713-15.json",
        "boston": ROOT / "artifacts/environmental-validation/era5-ndbc-wind-boston-20260713-15.json",
    }
    current = json.loads(current_path.read_text())["currents"]["matches"]
    winds = {site: json.loads(path.read_text())["wind"]["matches"] for site, path in wind_paths.items()}
    report = {
        "schema_version": 1,
        "artifact_kind": "environmental-zero-and-persistence-baselines",
        "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "definition": "Speed RMSE against zero speed, and against observed speed at the preceding native model interval at the same spatial location.",
        "currents": baselines(current, 6, "zone"),
        "wind": {site: baselines(rows, 1)["overall"] for site, rows in winds.items()},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
