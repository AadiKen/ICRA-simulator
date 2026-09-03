from __future__ import annotations
import json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];PARTS=[ROOT/f"artifacts/rl-campaign/p3-local/terminal-radius-sweep-n200-part-{i}.json" for i in range(4)];OUT=ROOT/"artifacts/rl-campaign/p3-local/terminal-radius-sweep-n200.json";parts=[json.loads(x.read_text())for x in PARTS];raw=[r for p in parts for r in p["raw"]];RADII=parts[0]["radii_m"];B=20000
def quantile(v,q):
 a=sorted(v);p=(len(a)-1)*q;i=int(p);f=p-i;return a[i]+(a[min(i+1,len(a)-1)]-a[i])*f
def bootstrap(values_a,values_b):
 # Paired by seed; deterministic resampling predeclared here solely for the n=200 CI.
 state=20270901;out_a=[];out_b=[];out_gap=[]
 for _ in range(B):
  ia=[]
  for _ in range(len(values_a)):
   state=(state+0x6D2B79F5)&0xffffffff;z=state;z=((z^(z>>15))*(z|1))&0xffffffff;z=(z^(z+(((z^(z>>7))*(z|61))&0xffffffff)))&0xffffffff;ia.append(((z^(z>>14))&0xffffffff)%len(values_a))
  a=sum(values_a[i]for i in ia)/len(ia);b=sum(values_b[i]for i in ia)/len(ia);out_a.append(a);out_b.append(b);out_gap.append(a-b)
 return {"classical_95_ci":[quantile(out_a,.025),quantile(out_a,.975)],"untrained_95_ci":[quantile(out_b,.025),quantile(out_b,.975)],"gap_95_ci":[quantile(out_gap,.025),quantile(out_gap,.975)]}
cells=[]
for h in (120,240):
 for variant in ("pass_through","full"):
  key=f"{variant}_{h}"
  for radius in RADII:
   by={policy:sorted([r for r in raw if r["policy"]==policy],key=lambda x:x["seed"])for policy in ("frozen-untrained-policy-v1","LOS-PID-v2","LOS-SPEEDCAP-v2")};u=[int(x[key][str(radius)] if isinstance(next(iter(x[key])),str) else x[key][radius]) for x in by["frozen-untrained-policy-v1"]];pid=[int(x[key][str(radius)] if isinstance(next(iter(x[key])),str) else x[key][radius]) for x in by["LOS-PID-v2"]];mpc=[int(x[key][str(radius)] if isinstance(next(iter(x[key])),str) else x[key][radius]) for x in by["LOS-SPEEDCAP-v2"]];classical_name="LOS-PID-v2" if sum(pid)>=sum(mpc) else "LOS-SPEEDCAP-v2";c=pid if classical_name=="LOS-PID-v2" else mpc;ci=bootstrap(c,u);rate_c=sum(c)/len(c);rate_u=sum(u)/len(u);width=ci["gap_95_ci"][1]-ci["gap_95_ci"][0];gap=rate_c-rate_u;cells.append({"timeout_s":h,"variant":variant,"radius_m":radius,"rates":{"untrained":rate_u,"los_pid":sum(pid)/len(pid),"los_mpc":sum(mpc)/len(mpc),"best_classical":rate_c},"best_classical_policy":classical_name,"bootstrap_95_ci":ci,"gap":gap,"separation_ratio":gap/width if width else None,"qualifies":.4<=rate_c<=.7 and radius<=10})
qual=[x for x in cells if x["qualifies"]]
# Amendment 7 K0 settles the structure: only 120 s, pass-through cells may be
# selected.  The remaining 240 s/full cells are retained in `cells` as context.
selection_qual=[x for x in qual if x["timeout_s"]==120 and x["variant"]=="pass_through"]
selection_qual.sort(key=lambda x:(-x["separation_ratio"],x["radius_m"]))
report={"schema_version":1,"artifact_kind":"terminal-radius-sweep-n200","status":"complete-halt-before-contract-revision","seed_set":{"first":10000,"count":200,"extension":"original 50 plus 150 new fixed seeds"},"bootstrap":{"resamples":B,"seed":20270901,"paired_by_seed":True},"cells":cells,"selection_rule":"Among Amendment-7 K0-approved 120 s pass-through cells only, maximize (best_classical-untrained)/CI_width(gap), subject to best classical 40-70% and radius <=10 m; smaller radius breaks ties. All variants/timeouts remain reported but are not selectable.","selected_candidate":selection_qual[0] if selection_qual else None,"all_qualifying_cells":qual,"decision":{"contract_revision_applied":False,"human_approval_required":True}};tmp=Path(str(OUT)+".tmp");tmp.write_text(json.dumps(report,indent=2)+"\n");tmp.replace(OUT);print(json.dumps({"selected_candidate":report["selected_candidate"],"selection_qualifying":selection_qual,"all_qualifying":qual},indent=2))
