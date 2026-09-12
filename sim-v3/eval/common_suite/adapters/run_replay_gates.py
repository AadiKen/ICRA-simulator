from __future__ import annotations
import argparse,importlib,json
from pathlib import Path
from .determinism_gate import load_replay,run_determinism_gate
ARMS=("bcod-sim","gazebo","holoocean","stonefish")
def factory(spec):
 module,sep,name=spec.partition(":")
 if not sep:raise ValueError("adapter factory must be module:callable")
 return getattr(importlib.import_module(module),name)()
def main():
 p=argparse.ArgumentParser();p.add_argument("--config",type=Path,required=True,help="JSON with adapter_factories and native_replays for all four arms");p.add_argument("--output",type=Path,required=True);a=p.parse_args();cfg=json.loads(a.config.read_text());results={}
 for arm in ARMS:
  try:result=run_determinism_gate(factory(cfg["adapter_factories"][arm]),load_replay(Path(cfg["native_replays"][arm]))).to_dict()
  except Exception as error:result={"arm":arm,"status":"failed","passed":False,"bit_identical":False,"first_divergence_step":None,"max_abs_divergence":None,"notes":f"{type(error).__name__}: {error}"}
  results[arm]=result;print(f"{arm}: {'PASS' if result['passed'] else 'FAIL'} {result.get('notes','')}")
 report={"schema_version":1,"artifact_kind":"four-adapter-replay-gates","passed":all(r["passed"] for r in results.values()),"scope":"repeatability only; not fidelity","adapters":results};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n");raise SystemExit(0 if report["passed"] else 3)
if __name__=="__main__":main()
