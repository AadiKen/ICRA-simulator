#!/usr/bin/env python3
"""Controlled half-hull Surface ablation for startup angular impulses."""
import json,os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv
runtime=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")]
conditions=[]
for mode in ("both","port","starboard","none"):
    os.environ["BCOD_VRX_SURFACE_MODE"]=mode
    env=VrxGymEnv(ROOT,runtime,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="zero")
    rows=[]
    try:
        _,info=env.reset()
        for step in range(1,7):
            _,_,_,_,info=env.step(np.zeros(2,np.float32))
            truth=env.bridge._request({"op":"truth"})["truth"]
            rows.append({"step":step,"simulation_time_s":env.bridge._request({"op":"diagnostic_status"})["simulation_time_s"],"roll_rad":truth["attitude_rad"][0],"pitch_rad":truth["attitude_rad"][1],"yaw_rad":truth["attitude_rad"][2],"angular_rate_body_rad_s":truth["angular_rate_body_rad_s"]})
    finally: env.close()
    conditions.append({"surface_mode":mode,"rows":rows,"peak_abs_yaw_rate_rad_s":max(abs(r["angular_rate_body_rad_s"][2]) for r in rows),"peak_abs_roll_rad":max(abs(r["roll_rad"]) for r in rows),"peak_abs_pitch_rad":max(abs(r["pitch_rad"]) for r in rows)})
os.environ.pop("BCOD_VRX_SURFACE_MODE",None)
report={"schema_version":1,"artifact_kind":"vrx-surface-half-hull-angular-ablation","seed":30000,"command":"zero","disturbance_mode":"zero","conditions":conditions}
out=ROOT/"artifacts/rl-campaign/vrx-surface-half-hull-angular-ablation.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
