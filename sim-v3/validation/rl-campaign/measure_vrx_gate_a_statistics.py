#!/usr/bin/env python3
"""Measure live VRX GPS noise and initial IMU yaw conversion error."""
import json,math,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv

def angle_error(a,b): return (a-b+math.pi)%(2*math.pi)-math.pi

env=VrxGymEnv(ROOT,[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")],
              allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="zero")
try:
    obs,_=env.reset(); samples=[]; initial_yaw=None; seen=set()
    for _ in range(220):
        obs,*_=env.step(np.zeros(2,dtype=np.float32))
        truth=env.last_truth
        if initial_yaw is None and np.any(obs[:7]!=0):
            initial_yaw={"observed_ned_rad":float(obs[6]),"truth_ned_rad":float(truth["attitude_rad"][2]),
                         "circular_error_rad":angle_error(float(obs[6]),float(truth["attitude_rad"][2]))}
        status=env.bridge._request({"op":"diagnostic_status"}); diag=status["gps_conditioning"]["held_diagnostic"]
        if diag and diag["timestamp_s"] not in seen:
            seen.add(diag["timestamp_s"]); samples.append(diag)
        if len(samples)>=20 and initial_yaw is not None: break
    magnitudes=[x["noise_magnitude_m"] for x in samples]
    axes=[v for x in samples for v in x["noise_ned_m"]]
    report={"simulation_time_basis":True,"sample_count":len(samples),"configured_per_axis_std_m":0.8,
            "gps_position_noise":{"first_noise_ned_m":samples[0]["noise_ned_m"],
             "first_magnitude_m":magnitudes[0],"mean_magnitude_m":sum(magnitudes)/len(magnitudes),
             "rms_radial_m":math.sqrt(sum(v*v for v in magnitudes)/len(magnitudes)),
             "component_rms_m":math.sqrt(sum(v*v for v in axes)/len(axes)),"maximum_magnitude_m":max(magnitudes)},
            "initial_yaw_conversion":initial_yaw}
    print(json.dumps(report,indent=2))
finally: env.close()
