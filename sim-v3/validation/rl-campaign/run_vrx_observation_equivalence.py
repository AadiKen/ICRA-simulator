#!/usr/bin/env python3
"""VRX Gate C observation-equivalence trace only; no other gate checks."""
import json,math,statistics,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv,VrxGymEnv
names=["accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","yaw","goal_north","goal_east","fix_valid","prev_port","prev_starboard","prev_steer_0","prev_steer_1","time_remaining"]
tol=[.1,.1,.1,.01,.01,.01,.01,4,4,0,1e-7,1e-7,1e-7,1e-7,1e-7];div=[[] for _ in names];truth={"yaw_rad":[],"yaw_rate_rad_s":[]};rows=[];seed=30000
bcod=CommonWaypointEnv(ROOT,fixed_reset_seed=seed,disturbance_mode="seeded");vrx=VrxGymEnv(ROOT,[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")],allow_unconformant_diagnostic=True,fixed_reset_seed=seed,disturbance_mode="seeded")
try:
    bcod.reset();vrx.reset();start=vrx.bridge._request({"op":"diagnostic_status"})["simulation_time_s"]
    for i in range(12):
        action=np.asarray([.2+.01*i,.2-.005*i],np.float32);bo,*_=bcod.step(action);vo,*_=vrx.step(action)
        delta=[abs(float(a)-float(b)) for a,b in zip(bo,vo)];delta[6]=abs((float(bo[6])-float(vo[6])+math.pi)%(2*math.pi)-math.pi)
        for target,value in zip(div,delta):target.append(value)
        yaw_delta=abs((bcod.last_truth["attitude_rad"][2]-vrx.last_truth["attitude_rad"][2]+math.pi)%(2*math.pi)-math.pi);rate_delta=abs(bcod.last_truth["angular_rate_body_rad_s"][2]-vrx.last_truth["angular_rate_body_rad_s"][2]);truth["yaw_rad"].append(yaw_delta);truth["yaw_rate_rad_s"].append(rate_delta)
        status=vrx.bridge._request({"op":"diagnostic_status"});rows.append({"control_step":i+1,"simulation_time_s":status["simulation_time_s"],"odometry_yaw_rate_warmup":status.get("odometry_yaw_rate_warmup"),"filter_imu_timestamp_s":vrx.last_observation["sensors"].get("imu",{}).get("timestampS"),"filtered_acceleration_body_mps2":vo[:3].tolist(),"bcod_yaw_rate_rad_s":bcod.last_truth["angular_rate_body_rad_s"][2],"vrx_yaw_rate_rad_s":vrx.last_truth["angular_rate_body_rad_s"][2],"yaw_rate_abs_divergence_rad_s":rate_delta,"yaw_abs_divergence_rad":yaw_delta})
    elapsed=vrx.bridge._request({"op":"diagnostic_status"})["simulation_time_s"]-start
finally:bcod.close();vrx.close()
fields={name:{"median_abs":statistics.median(v),"max_abs":max(v),"max_tolerance":limit,"passed":max(v)<=limit+1e-12} for name,v,limit in zip(names,div,tol)}
report={"schema_version":1,"artifact_kind":"vrx-filtered-observation-equivalence","filter_protocol":"artifacts/rl-campaign/vrx-imu-filter-preregistered.json","odometry_yaw_rate_warmup":{"duration_s":.5,"invalid_control_steps_after_reset":4,"policy":"zero-fill odometry-derived truth yaw rate; IMU gyro is unaffected"},"seed":seed,"control_steps":12,"elapsed_simulation_s":elapsed,"fields":fields,"truth_path_divergence":{k:{"median_abs":statistics.median(v),"max_abs":max(v)} for k,v in truth.items()},"per_step":rows,"passed":math.isclose(elapsed,1.2,abs_tol=1e-9) and all(x["passed"] for x in fields.values())}
out=ROOT/"artifacts/rl-campaign/vrx-filtered-observation-equivalence.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));raise SystemExit(0 if report["passed"] else 1)
