from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
from typing import Any
from .contracts import load_contracts

ARMS=("bcod-sim","gazebo","holoocean","stonefish")

def gate1(contract_dir:Path,calibration:dict[str,Any])->dict[str,Any]:
 contracts=load_contracts(contract_dir);rates=calibration.get("native_nominal_success_rate_by_arm",{});missing=sorted(set(ARMS)-rates.keys());saturated={arm:{"rate":float(rate),"direction":"floor" if float(rate)<=.05 else "ceiling"} for arm,rate in rates.items() if float(rate)<=.05 or float(rate)>=.95};unfrozen=[c.condition_id for c in contracts if c.document.get("status")!="frozen"];hashes={c.condition_id:c.content_sha256 for c in contracts};hash_mismatch=hashes!=calibration.get("contract_hashes");passed=not missing and not saturated and not unfrozen and not hash_mismatch and calibration.get("passed") is True
 return {"gate":1,"name":"condition-contracts-frozen","passed":passed,"contract_hashes":hashes,"calibration_hash_mismatch":hash_mismatch,"nominal_success_rate_by_arm":rates,"missing_arms":missing,"saturated":saturated,"unfrozen_conditions":unfrozen}

def gate2(validation:dict[str,Any])->dict[str,Any]:
 rows=validation.get("arms",[]);by_arm={r.get("arm"):r for r in rows};missing=sorted(set(ARMS)-by_arm.keys());unreviewed=not bool(validation.get("human_reviewed"));failed=[arm for arm,row in by_arm.items() if row.get("classification")=="failed"];passed=not missing and not unreviewed and not failed and all(by_arm[a].get("classification") in ("clean","flagged","vehicle-c-known-residual") for a in ARMS if a in by_arm)
 return {"gate":2,"name":"analytic-judge-validation","passed":passed,"arms":rows,"missing_arms":missing,"human_reviewed":not unreviewed,"failed_arms":failed}

def gate3(adapter_report:dict[str,Any])->dict[str,Any]:
 source=adapter_report.get("adapters",adapter_report);rows=[]
 for arm in ARMS:
  result=source.get(arm);rows.append({"arm":arm,"passed":bool(result and result.get("passed")),"status":"missing" if result is None else result.get("status"),"first_divergence_step":None if result is None else result.get("first_divergence_step"),"max_abs_divergence":None if result is None else result.get("max_abs_divergence"),"notes":None if result is None else result.get("notes")})
 return {"gate":3,"name":"adapter-replay-determinism","passed":all(r["passed"] for r in rows),"scope":"repeatability only; not fidelity","adapters":rows}

def gate4(checkpoints:dict[str,Any])->dict[str,Any]:
 rows=[]
 for arm in ARMS:
  item=checkpoints.get(arm,{})
  rows.append({"arm":arm,"steps":item.get("steps"),"checkpoint_save_interval":item.get("checkpoint_save_interval"),"checkpoint_sha256":item.get("checkpoint_sha256")})
 missing=[r["arm"] for r in rows if not isinstance(r["steps"],int) or r["steps"]<=0];target=checkpoints.get("target_steps")
 if not isinstance(target,int) or target<=0:missing.append("target_steps")
 cadences={r["checkpoint_save_interval"] for r in rows if isinstance(r["checkpoint_save_interval"],int) and r["checkpoint_save_interval"]>0};common_cadence=len(cadences)==1 and len(rows)==sum(isinstance(r["checkpoint_save_interval"],int) and r["checkpoint_save_interval"]>0 for r in rows);tolerance=next(iter(cadences)) if common_cadence else (None if not isinstance(target,int) else .01*target);rule="common checkpoint-save interval" if common_cadence else "1% target fallback (save cadences differ or are missing)"
 for row in rows:
  row["delta_from_target_steps"]=None if not isinstance(target,int) or not isinstance(row["steps"],int) else row["steps"]-target;row["within_tolerance"]=False if tolerance is None or row["delta_from_target_steps"] is None else abs(row["delta_from_target_steps"])<=tolerance
 return {"gate":4,"name":"training-step-parity","passed":not missing and all(r["within_tolerance"] for r in rows),"target_steps":target,"tolerance_steps":tolerance,"tolerance_rule":rule,"missing":missing,"arms":rows}

def run_preflight(*,contract_dir:Path,calibration:dict[str,Any],judge_validation:dict[str,Any],adapter_report:dict[str,Any],checkpoints:dict[str,Any])->dict[str,Any]:
 gates=[];g1=gate1(contract_dir,calibration);gates.append(g1)
 if not g1["passed"]:return _report(gates)
 g2=gate2(judge_validation);gates.append(g2)
 if not g2["passed"]:return _report(gates)
 g3=gate3(adapter_report);gates.append(g3)
 if not g3["passed"]:return _report(gates)
 gates.append(gate4(checkpoints));return _report(gates)
def _report(gates):
 passed=len(gates)==4 and all(g["passed"] for g in gates);payload={"schema_version":1,"artifact_kind":"cross-simulator-common-suite-preflight","passed":passed,"runner_authorized":passed,"gates":gates,"stopped_after_gate":None if passed else gates[-1]["gate"],"prohibition":None if passed else "runner.py must not execute against real checkpoints"};encoded=json.dumps(payload,sort_keys=True,separators=(",",":")).encode();payload["content_sha256"]=hashlib.sha256(encoded).hexdigest();return payload
def verify_preflight(report:dict[str,Any])->None:
 supplied=report.get("content_sha256");payload={k:v for k,v in report.items() if k!="content_sha256"};actual=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(",",":")).encode()).hexdigest()
 if supplied!=actual:raise RuntimeError("Preflight artifact content hash mismatch")
 if not report.get("passed") or not report.get("runner_authorized") or len(report.get("gates",[]))!=4 or not all(g.get("passed") for g in report["gates"]):raise RuntimeError("All four preflight gates must pass before runner execution")
def main():
 p=argparse.ArgumentParser();p.add_argument("--contract-dir",type=Path,default=Path(__file__).with_name("contracts"));p.add_argument("--calibration",type=Path,required=True);p.add_argument("--judge-validation",type=Path,required=True);p.add_argument("--adapter-report",type=Path,required=True);p.add_argument("--checkpoints",type=Path,required=True);p.add_argument("--output",type=Path,required=True);a=p.parse_args();load=lambda path:json.loads(path.read_text());report=run_preflight(contract_dir=a.contract_dir,calibration=load(a.calibration),judge_validation=load(a.judge_validation),adapter_report=load(a.adapter_report),checkpoints=load(a.checkpoints));a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));raise SystemExit(0 if report["passed"] else report["stopped_after_gate"])
if __name__=="__main__":main()
