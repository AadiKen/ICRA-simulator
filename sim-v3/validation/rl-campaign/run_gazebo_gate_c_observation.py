#!/usr/bin/env python3
"""Gate C Step 2 only: scripted 15-field observation equivalence."""
from __future__ import annotations
import json,math,platform,statistics,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv,GazeboGymEnv

SEED=30000
NAMES=["accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","yaw","goal_north","goal_east","fix_valid","prev_port","prev_starboard","prev_steer_0","prev_steer_1","time_remaining"]
TOLERANCE={"accel_x":.1,"accel_y":.1,"accel_z":.1,"gyro_x":.01,"gyro_y":.01,"gyro_z":.01,"yaw":.01,"goal_north":4.,"goal_east":4.,"fix_valid":0.,"prev_port":1e-7,"prev_starboard":1e-7,"prev_steer_0":1e-7,"prev_steer_1":1e-7,"time_remaining":1e-7}
OUT=ROOT/"artifacts/rl-campaign/gazebo-gate-c-observation-equivalence.json"

def circular(a,b):return abs((a-b+math.pi)%(2*math.pi)-math.pi)
def main():
 runtime=[sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]
 bcod=CommonWaypointEnv(ROOT,fixed_reset_seed=SEED);gaz=GazeboGymEnv(ROOT,runtime,allow_unconformant_diagnostic=True,fixed_reset_seed=SEED)
 divergences=[[] for _ in NAMES];rows=[];truth={"position_m":[],"yaw_rad":[],"velocity_body_mps":[],"yaw_rate_rad_s":[]}
 try:
  bobs,_=bcod.reset();gobs,_=gaz.reset();start=gaz.bridge._request({"op":"diagnostic_status"})["simulation_time_s"]
  for i in range(12):
   action=np.asarray([.2+.01*i,.2-.005*i],np.float32)
   bobs,_,bt,btr,_=bcod.step(action);gobs,_,gt,gtr,_=gaz.step(action)
   delta=[abs(float(a)-float(b)) for a,b in zip(bobs,gobs)];delta[6]=circular(float(bobs[6]),float(gobs[6]))
   for values,value in zip(divergences,delta):values.append(value)
   bp,gp=bcod.last_truth,gaz.last_truth
   truth["position_m"].append(math.dist(bp["position_ned_m"][:2],gp["position_ned_m"][:2]))
   truth["yaw_rad"].append(circular(bp["attitude_rad"][2],gp["attitude_rad"][2]))
   truth["velocity_body_mps"].append(math.dist(bp["velocity_body_mps"][:2],gp["velocity_body_mps"][:2]))
   truth["yaw_rate_rad_s"].append(abs(bp["angular_rate_body_rad_s"][2]-gp["angular_rate_body_rad_s"][2]))
   rows.append({"control_step":i+1,"action":action.tolist(),"bcod_observation":bobs.tolist(),"gazebo_observation":gobs.tolist(),"abs_divergence":delta})
   if bt or btr or gt or gtr:break
  elapsed=gaz.bridge._request({"op":"diagnostic_status"})["simulation_time_s"]-start
 finally:bcod.close();gaz.close()
 fields={name:{"median_abs":statistics.median(values),"max_abs":max(values),"max_tolerance":TOLERANCE[name],"passed":max(values)<=TOLERANCE[name]+1e-12} for name,values in zip(NAMES,divergences)}
 timing_pass=math.isclose(elapsed,len(rows)*.1,rel_tol=0,abs_tol=1e-9);passed=timing_pass and all(item["passed"] for item in fields.values())
 report={"schema_version":1,"artifact_kind":"gazebo-gate-c-step-2-observation-equivalence","status":"PASS" if passed else "FAIL","seed":SEED,"script":"12 identical deterministic differential-thrust controls; port=0.2+0.01*i, starboard=0.2-0.005*i","runtime":{"image":"icra27-gazebo-harmonic:harmonic-8.15.0","image_id":"174e8baad590","host_architecture":platform.machine(),"container_architecture":"linux/amd64","execution":"AMD64 emulation on Apple Silicon","timing_basis":"Gazebo /clock simulation time only","internal_step_s":.005,"contract_physics_tick_s":.05,"control_interval_s":.1},"tolerance_basis":{"imu":"Conservative envelopes above bcod-sim defaults (acceleration noise 0.015 m/s^2 plus 0.02 m/s^2 bias; gyro noise 0.0008 rad/s plus 0.001 rad/s bias).","gps":"4 m maximum per horizontal axis versus 0.8 m standard deviation in each independently conditioned path.","deterministic_fields":"Numerical precision only.","rule":"Every field maximum must be within its stated envelope; medians are reported but not substituted for maxima."},"control_steps":len(rows),"simulation_time":{"expected_elapsed_s":len(rows)*.1,"observed_elapsed_s":elapsed,"passed":timing_pass},"fields":fields,"truth_path_divergence":{name:{"median_abs":statistics.median(values),"max_abs":max(values)} for name,values in truth.items()},"per_step":rows,"passed":passed,"next_step_started":False,"training_authorized":False,"notes":["Step 2 only; wind/current and termination work were not started.","The Vehicle A idle-stability artifact passed before this run."]}
 OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if passed else 1
if __name__=="__main__":raise SystemExit(main())
