from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any
import numpy as np


def _fmt(metric: dict[str, Any], digits: int = 2) -> str:
    return f"{metric['mean']:.{digits}f} [{metric['ci95'][0]:.{digits}f}, {metric['ci95'][1]:.{digits}f}]"


def build_tables(report: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    breakdown = []
    for cell in report["aggregates"]:
        breakdown.append({"arm": cell["arm"], "condition": cell["condition"], "n": cell["episode_count"], "contract_sha256": cell["contract_sha256"], "domain_gap":cell["domain_gap"],"cross_track_error_m_mean_ci95": _fmt(cell["cross_track_error_m"]), "success_rate_ci95": f"{cell['success_rate']:.3f} [{cell['success_ci95'][0]:.3f}, {cell['success_ci95'][1]:.3f}]", "completion_time_s_mean_ci95": _fmt(cell["completion_time_s"]), "propulsion_cost_ns_mean_ci95": _fmt(cell["propulsion_cost_ns"]), "safety_violations_mean_ci95": _fmt(cell["safety_violations"])})
    nominal = [row for row in breakdown if row["condition"] == "nominal"]
    by_arm: dict[str, dict[str, Any]] = {}
    for episode in report["episodes"]:
        by_arm.setdefault(episode["arm"], {"arm": episode["arm"], "environment_steps": episode["training_steps"], "training_wall_clock_s": episode["training_wall_clock_s"]})
    steps = {row["environment_steps"] for row in by_arm.values()}
    if len(steps) != 1:
        raise ValueError("Headline sample-efficiency table requires equal training environment steps")
    comparisons=[];rng=np.random.default_rng(7319)
    by_condition={condition:{} for condition in {row["condition"] for row in report["episodes"]}}
    for row in report["episodes"]:by_condition[row["condition"]].setdefault(row["arm"],{})[row["seed"]]=row["cross_track_error_m"]
    for condition,arms in sorted(by_condition.items()):
        ranked=sorted(arms,key=lambda arm:np.mean(list(arms[arm].values())))
        if len(ranked)<2:continue
        winner,runner_up=ranked[:2];seeds=sorted(set(arms[winner])&set(arms[runner_up]));differences=np.asarray([arms[runner_up][seed]-arms[winner][seed] for seed in seeds]);means=rng.choice(differences,size=(20_000,len(differences)),replace=True).mean(axis=1);p=min(1.,2*min(float(np.mean(means<=0)),float(np.mean(means>=0))))
        comparisons.append({"condition":condition,"winner":winner,"runner_up":runner_up,"mean_cross_track_advantage_m":float(differences.mean()),"raw_p":p})
    order=sorted(range(len(comparisons)),key=lambda i:comparisons[i]["raw_p"]);running=0.
    for rank,index in enumerate(order):running=max(running,min(1.,comparisons[index]["raw_p"]*(len(order)-rank)));comparisons[index]["holm_adjusted_p"]=running
    return {"headline_matched_steps": nominal, "wall_clock": list(by_arm.values()), "per_condition": breakdown,"primary_cross_track_holm":comparisons,"judge_validation":report["judge_validation"]["arms"], "adapter_preflight": [{"arm": arm, **result} for arm, result in report["preflight"].items()]}


def write_tables(tables: dict[str, list[dict[str, Any]]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, rows in tables.items():
        path = output_dir / f"{name}.csv"
        with path.open("w", newline="") as handle:
            if rows:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]), extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)
    (output_dir / "methods.txt").write_text("All four independently trained PPO policies were transferred through arm-specific adapters to a standalone RK4 Fossen-plant judge. The judge is independent of every training engine but is not asserted dynamics-neutral. Each condition used a shared frozen 200-seed set; per-arm open-loop domain gap is reported in the result table.\n")
    (output_dir / "limitations.txt").write_text("The analytic judge may be closer to one simulator's native dynamics than another; the domain-gap column exposes but does not eliminate this confound. Vehicle C retains an unresolved symmetric-thrust lateral/heading residual. Adapter determinism establishes repeatability, not fidelity. Holm-adjusted primary cross-track comparisons are reported. Adapter engineering effort was not blind, and bcod-sim received more implicit attention.\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    write_tables(build_tables(json.loads(args.result.read_text())), args.output_dir)


if __name__ == "__main__":
    main()
