#!/usr/bin/env python3
"""Compare VRX frozen-reset disturbance draws to bcod-sim's shared formula."""
import json,math,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim.common_task_env import CommonWaypointEnv

class Formula(CommonWaypointEnv):
    def __init__(self): super().__init__(ROOT,bridge=object())

seeds=list(range(30000,30005));formula=Formula()
vrx=json.loads(subprocess.check_output(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/dump-vrx-reset-randomization.ts"),*map(str,seeds)],cwd=ROOT,text=True))
rows=[]
for seed,external in zip(seeds,vrx):
    _,_,_,current,wind=formula._randomization(seed);d=external["disturbance"]
    bcod={"current_speed_m_s":math.hypot(current[0],current[1]),"current_direction_deg":math.degrees(math.atan2(current[1],current[0]))%360,
          "wind_speed_m_s":math.hypot(wind[0],wind[1]),"wind_direction_deg":math.degrees(math.atan2(wind[1],wind[0]))%360}
    vrx_values={k:d[k] for k in bcod};errors={k:vrx_values[k]-bcod[k] for k in bcod}
    rows.append({"seed":seed,"bcod_sim":bcod,"vrx":vrx_values,"absolute_errors":{k:abs(v) for k,v in errors.items()},"match":all(abs(v)<=1e-12 for v in errors.values())})
report={"schema_version":1,"artifact_kind":"vrx-bcod-reset-randomization-parity","contract_path":"artifacts/rl-campaign/surveyor/task-contract-frozen.json",
        "seeds":seeds,"distribution":"shared Mulberry32 uniform draws in contract order over reset_randomization ranges",
        "rows":rows,"all_match":all(row["match"] for row in rows),"maximum_absolute_error":max(v for row in rows for v in row["absolute_errors"].values())}
out=ROOT/"artifacts/rl-campaign/vrx-reset-randomization-parity.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
raise SystemExit(0 if report["all_match"] else 1)
