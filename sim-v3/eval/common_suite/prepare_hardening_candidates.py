from __future__ import annotations

import argparse
import json
from copy import deepcopy
from pathlib import Path

from .contracts import content_hash


def main() -> None:
    parser = argparse.ArgumentParser(description="Create content-hashed terminal-radius calibration candidates")
    parser.add_argument("--nominal", type=Path, default=Path(__file__).with_name("contracts") / "nominal.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--radii", nargs="+", type=float, default=[1.5, 1.0, 0.75])
    args = parser.parse_args()
    source = json.loads(args.nominal.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for radius in args.radii:
        candidate = deepcopy(source)
        candidate["contract_version"] = "1.1.0-candidate"
        candidate["status"] = "hardening-calibration-required"
        candidate["termination"]["success_radius_m"] = radius
        candidate["hardening"] = {
            "lever": "terminal-success-radius",
            "source_contract_sha256": source["content_sha256"],
            "environmental_disturbance_added": False,
        }
        candidate["content_sha256"] = content_hash(candidate)
        output = args.output_dir / f"nominal-radius-{radius:g}m.json"
        output.write_text(json.dumps(candidate, indent=2) + "\n")
        print(f"{radius:g} {candidate['content_sha256']} {output}")


if __name__ == "__main__":
    main()
