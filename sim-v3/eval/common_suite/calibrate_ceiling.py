from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Callable

from .contracts import content_hash


def select_candidate(candidates: list[dict[str, Any]], evaluate: Callable[[dict[str, Any]], float], *, band: tuple[float, float] = (0.4, 0.7)) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """First-pass selection using only bcod-sim native nominal performance."""
    rows = [{"candidate": candidate, "success_rate": float(evaluate(candidate))} for candidate in candidates]
    eligible = [row for row in rows if band[0] <= row["success_rate"] <= band[1]]
    if not eligible:
        raise RuntimeError("No difficulty candidate produced mid-band native nominal performance")
    selected = min(eligible, key=lambda row: abs(row["success_rate"] - sum(band) / 2))["candidate"]
    return selected, rows

def confirm_all_arms(rates: dict[str, float], saturation_bounds: tuple[float, float] = (.05, .95)) -> None:
    required={"bcod-sim","gazebo","holoocean","stonefish"};missing=required-rates.keys()
    if missing:raise RuntimeError(f"Native nominal calibration is missing arms: {sorted(missing)}")
    saturated={arm:rate for arm,rate in rates.items() if not saturation_bounds[0]<float(rate)<saturation_bounds[1]}
    if saturated:raise RuntimeError(f"Cannot freeze a ceiling/floor-saturated task: {saturated}")


def freeze_contract(document: dict[str, Any], difficulty: dict[str, Any]) -> dict[str, Any]:
    version=str(document.get("contract_version","1.0.0")).removesuffix("-candidate")
    frozen = {**document, "contract_version":version,"difficulty": difficulty, "status": "frozen", "hash_scope": "canonical JSON of every field except content_sha256 and hash_scope"}
    frozen["content_sha256"] = content_hash(frozen)
    return frozen


def main() -> None:
    parser = argparse.ArgumentParser(description="First-pass bcod calibration plus all-arm saturation gate")
    parser.add_argument("--calibration-result", type=Path, required=True, help="JSON candidates with bcod rate and native_nominal_success_rate_by_arm")
    parser.add_argument("--contract-dir", type=Path, default=Path(__file__).with_name("contracts"))
    parser.add_argument("--output",type=Path,required=True,help="Write the auditable Gate 1 calibration artifact")
    args = parser.parse_args()
    source = json.loads(args.calibration_result.read_text())
    rates = {json.dumps(row["difficulty"], sort_keys=True): row["bcod_native_nominal_success_rate"] for row in source["candidates"]}
    candidates = [row["difficulty"] for row in source["candidates"]]
    selected, rows = select_candidate(candidates, lambda candidate: rates[json.dumps(candidate, sort_keys=True)])
    selected_row=next(row for row in source["candidates"] if row["difficulty"]==selected)
    confirm_all_arms(selected_row["native_nominal_success_rate_by_arm"])
    hashes={}
    for path in args.contract_dir.glob("*.json"):
        document = json.loads(path.read_text())
        frozen=freeze_contract(document,selected);path.write_text(json.dumps(frozen,indent=2)+"\n");hashes[frozen["condition_id"]]=frozen["content_sha256"]
    report={"schema_version":1,"artifact_kind":"common-suite-difficulty-calibration","selected": selected, "bcod_first_pass": rows, "native_nominal_success_rate_by_arm":selected_row["native_nominal_success_rate_by_arm"],"saturation_bounds":{"exclusive":[.05,.95]},"contract_hashes":hashes,"passed":True,"scope":"bcod first-pass; all four native arms required before freeze"};args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))


if __name__ == "__main__":
    main()
