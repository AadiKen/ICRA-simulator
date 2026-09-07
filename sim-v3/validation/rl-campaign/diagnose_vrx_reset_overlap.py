#!/usr/bin/env python3
"""Check whether paused launch removes pre-reset disturbance drift."""
import json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv
runtime=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")];rows=[]
for _ in range(3):
    env=VrxGymEnv(ROOT,runtime,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="seeded")
    try:
        env.reset();status=env.bridge._request({"op":"diagnostic_status"});rows.append({"simulation_time_s":status["simulation_time_s"],"position_ned_m":env.last_truth["position_ned_m"][:2],"yaw_ned_rad":env.last_truth["attitude_rad"][2]})
    finally:env.close()
deltas=[math.dist(rows[0]["position_ned_m"],row["position_ned_m"]) for row in rows[1:]]
print(json.dumps({"launch":"paused; no -r","rows":rows,"maximum_position_delta_m":max(deltas),"pass_below_0.05_m":max(deltas)<=.05},indent=2))
