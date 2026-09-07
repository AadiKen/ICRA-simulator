#!/usr/bin/env python3
"""Capture reset and first-step VRX yaw provenance without changing runtime behavior."""
import json,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv

results=[];seeds=tuple(map(int,sys.argv[1:])) or (30000,30001,30007,30013)
for seed in seeds:
    env=VrxGymEnv(ROOT,[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")],
                  allow_unconformant_diagnostic=True,fixed_reset_seed=seed,disturbance_mode="zero")
    rows=[]
    try:
        obs,_=env.reset()
        for phase in ["reset","step_1","step_2","step_3"]:
            status=env.bridge._request({"op":"diagnostic_status"});truth=env.last_truth
            observed=float(obs[6]);expected=float(truth["attitude_rad"][2])
            rows.append({"phase":phase,"simulation_time_s":status["simulation_time_s"],
                         "observation_yaw_ned_rad":observed,"truth_yaw_ned_rad":expected,
                         "circular_error_rad":(observed-expected+np.pi)%(2*np.pi)-np.pi,
                         **status["yaw_diagnostic"]})
            if phase!="step_3": obs,*_=env.step(np.zeros(2,dtype=np.float32))
        results.append({"seed":seed,"samples":rows})
    finally: env.close()
print(json.dumps(results,indent=2))
