#!/usr/bin/env python3
"""Measure the accepted VRX current-coupling residual on multiple Gate C seeds."""
import json,math,statistics,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv,VrxGymEnv

RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")]
SEEDS=[30000,30001,30007,30013]
def circular(a,b):return abs((a-b+math.pi)%(2*math.pi)-math.pi)

rows=[]
for seed in SEEDS:
    bcod=CommonWaypointEnv(ROOT,fixed_reset_seed=seed,disturbance_mode="seeded")
    vrx=VrxGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=seed,disturbance_mode="seeded")
    yaw=[];rate=[]
    try:
        bcod.reset();vrx.reset()
        for i in range(12):
            action=np.asarray([.2+.01*i,.2-.005*i],np.float32)
            bcod.step(action);vrx.step(action)
            yaw.append(circular(bcod.last_truth["attitude_rad"][2],vrx.last_truth["attitude_rad"][2]))
            rate.append(abs(bcod.last_truth["angular_rate_body_rad_s"][2]-vrx.last_truth["angular_rate_body_rad_s"][2]))
    finally:bcod.close();vrx.close()
    rows.append({"seed":seed,"yaw_rate_divergence_rad_s":{"median_abs":statistics.median(rate),"max_abs":max(rate)},"yaw_angle_divergence_rad":{"median_abs":statistics.median(yaw),"max_abs":max(yaw)},"per_step":{"yaw_rate_abs":rate,"yaw_angle_abs":yaw}})

all_rate=[x for row in rows for x in row["per_step"]["yaw_rate_abs"]]
all_yaw=[x for row in rows for x in row["per_step"]["yaw_angle_abs"]]
report={"schema_version":1,"artifact_kind":"vrx-gate-c-accepted-current-coupling-residual","status":"documented-residual","seeds":SEEDS,"protocol":"12-step mixed Gate C command sequence under identical seeded wind/current; post-Munk-moment current plugin","correctly_modeled":"Leading-order current-induced yaw torque via the Surveyor diagonal-added-mass Munk moment, in addition to current-relative surge/sway damping and body-relative wind force.","remaining_unmodeled":"Complete current-relative added-mass dynamics, including acceleration reaction and the full Coriolis force/torque coupling through VRX's 3D plant.","rows":rows,"aggregate":{"yaw_rate_divergence_rad_s":{"median_abs":statistics.median(all_rate),"max_abs":max(all_rate)},"yaw_angle_divergence_rad":{"median_abs":statistics.median(all_yaw),"max_abs":max(all_yaw)}},"acceptance_disposition":"Accepted and captioned simulator limitation for Gate C; it is not evidence of full plant parity and must not be hidden in eventual comparisons.","figure_caption_requirement":"VRX includes the leading-order current-induced Munk yaw moment but not bcod-sim's complete current-relative added-mass and Coriolis coupling. Across seeds 30000, 30001, 30007, and 30013, the retained residual is reported from this artifact and comparisons must not imply full plant parity."}
out=ROOT/"artifacts/rl-campaign/vrx-gate-c-current-coupling-limitation.json"
out.write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps({"artifact":str(out),"rows":[{k:v for k,v in row.items() if k!="per_step"} for row in rows],"aggregate":report["aggregate"]},indent=2))
