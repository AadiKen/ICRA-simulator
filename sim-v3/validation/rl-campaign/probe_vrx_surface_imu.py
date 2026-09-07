#!/usr/bin/env python3
"""A/B raw IMU acceleration with VRX Surface enabled versus disabled."""
import json,os,statistics,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv
runtime=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")];conditions=[]
for enabled in (True,False):
    if enabled:os.environ.pop("BCOD_VRX_DISABLE_SURFACE",None)
    else:os.environ["BCOD_VRX_DISABLE_SURFACE"]="1"
    env=VrxGymEnv(ROOT,runtime,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="zero");samples=[]
    try:
        env.reset()
        for _ in range(20):
            env.step(np.zeros(2,np.float32));status=env.bridge._request({"op":"diagnostic_status"});samples.append(status["yaw_diagnostic"]["imu_raw_linear_acceleration_flu_mps2"])
        z=[row[2] for row in samples];conditions.append({"surface_enabled":enabled,"samples":len(z),"raw_z_mps2":{"min":min(z),"max":max(z),"median":statistics.median(z),"population_std":statistics.pstdev(z)},"first_samples":z[:10]})
    finally:env.close()
os.environ.pop("BCOD_VRX_DISABLE_SURFACE",None)
report={"schema_version":1,"artifact_kind":"vrx-surface-imu-ablation","conditions":conditions,"conclusion":"buoyancy-specific" if conditions[0]["raw_z_mps2"]["population_std"]>5*conditions[1]["raw_z_mps2"]["population_std"] else "not-isolated"}
out=ROOT/"artifacts/rl-campaign/vrx-surface-imu-ablation.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
