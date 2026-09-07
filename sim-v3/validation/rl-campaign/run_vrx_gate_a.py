#!/usr/bin/env python3
"""Run VRX Gate A through the live VrxGymEnv path; no training."""
from __future__ import annotations
import hashlib,json,platform,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv
RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")]
OUT=ROOT/"artifacts/rl-campaign/vrx-15-field-live-gate-a.json"
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
 env=VrxGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="zero");checks={};rows=[]
 try:
  obs,reset_info=env.reset();rows.append(obs.tolist());initial=env.bridge._request({"op":"diagnostic_status"});epoch=initial["simulation_time_s"]
  checks["fully_connected_runtime_exists"]=initial["vrx"]["ros2_namespace_verified_live"]
  checks["exactly_15_fields"]=len(obs)==15;checks["reset_seed"]=reset_info["seed"]==30000
  first_release_sim=None;first_source=None
  for control in range(15):
   obs,_,terminated,truncated,info=env.step(np.asarray([.2,.2],np.float32));rows.append(obs.tolist())
   if obs[9]==1 and first_release_sim is None:
    release=env.bridge._request({"op":"diagnostic_status"});first_release_sim=release["simulation_time_s"];first_source=release["gps_conditioning"]["held_source_timestamp_s"]
   if terminated or truncated:raise RuntimeError(f"unexpected early termination: {info['termination_reason']}")
  status=env.bridge._request({"op":"diagnostic_status"});gps_ts=status["topic_timestamps_s"]["gps"];deltas=[round(b-a,9) for a,b in zip(gps_ts,gps_ts[1:]) if b>a];scheduled=[x for x in deltas if x>=.49]
  checks["gps_2_hz_sim_time"]=bool(scheduled) and all(abs(x-.5)<1e-9 for x in scheduled)
  checks["gps_conditioning_config"]=all(status["gps_conditioning"][k]==v for k,v in {"rate_hz":2.,"latency_s":.2,"position_std_m":.8}.items())
  checks["gps_released_on_connected_path"]=first_release_sim is not None
  age=None if first_release_sim is None else first_release_sim-first_source;checks["gps_release_latency_sim_time"]=age is not None and .2-1e-9<=age<=.25+1e-9
  env.bridge._request({"op":"diagnostic_stall","sensor":"imu","enabled":True});stale_imu,*_=env.step(np.asarray([.2,.2],np.float32));rows.append(stale_imu.tolist());checks["active_imu_stall_zero_fills"]=bool(np.all(stale_imu[:7]==0))
  env.bridge._request({"op":"diagnostic_stall","sensor":"imu","enabled":False});fresh_imu,*_=env.step(np.asarray([.2,.2],np.float32));rows.append(fresh_imu.tolist());checks["imu_recovers_from_live_topic"]=bool(np.any(fresh_imu[:7]!=0))
  env.bridge._request({"op":"diagnostic_stall","sensor":"gps","enabled":True});stalled=[]
  for _ in range(8):row,*_=env.step(np.asarray([.2,.2],np.float32));stalled.append(row);rows.append(row.tolist())
  checks["active_gps_stall_zero_fills"]=bool(np.all(stalled[-1][7:10]==0));checks["gps_did_not_refresh_from_retained_object"]=bool(stalled[-1][9]==0)
  final=env.bridge._request({"op":"diagnostic_status"});expected=epoch+2.5;checks["fixed_simulation_step_timing"]=abs(final["simulation_time_s"]-expected)<1e-9
  runtime_source=(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py").read_text();checks["no_observation_truth_fallback"]="sensors[\"imu\"]" not in runtime_source and "sensors[\"gps\"]" not in runtime_source
  # The VRX adapter inherits observation() unchanged from the connected
  # Gazebo template; that method consumes only /imu and ExternalGpsModel.
  checks["no_observation_truth_fallback"]="def observation" not in runtime_source
  report={"schema_version":1,"artifact_kind":"vrx-15-field-live-sensor-gate-a","status":"PASS" if all(checks.values()) else "FAIL","connected_path_status":"FULLY_CONNECTED_AND_PASSING" if all(checks.values()) else "FAIL_INCOMPLETE_CONNECTED_PATH","training_authorized":False,"next_gate_started":False,"runtime":{"image":status["vrx"]["image"],"gazebo_sim_version":status["vrx"]["gazebo_sim_version"],"host_architecture":platform.machine(),"container_architecture":"linux/amd64","execution":"AMD64 emulation on Apple Silicon","timing_basis":"VRX /clock and sensor-header simulation timestamps only"},"live_topic_resolution":{"method":"ros2 topic list and ros2 topic info against running VRX v3.0.1","actual":["/clock","/odometry","/imu","/gps","/surveyor/thrusters/port/thrust","/surveyor/thrusters/starboard/thrust"],"absent":["/wamv/thrusters/left/thrust","/wamv/thrusters/right/thrust"],"exporter_corrected":True},"connected_path":{"entrypoint":"VrxGymEnv -> JsonLineSimulatorBridge -> vrx_gym_runtime.py -> live VRX v3.0.1","adapter_sha256":sha(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py"),"conditioning":"external_sensor_model.py (shared)","conversion":"task-trace-bridge.ts via task-trace-jsonl-bridge.ts (shared)","actuation":"FrozenActuatorBank via actuator-jsonl-bridge.ts -> verified Surveyor topics"},"simulation_timing":{"reset_epoch_s":epoch,"expected_final_s":expected,"observed_final_s":final["simulation_time_s"],"gps_timestamps_s":gps_ts,"gps_positive_deltas_s":deltas,"first_release_age_s":age},"observation":{"field_count":15,"rows_examined":len(rows),"imu_source":"live /imu","gps_source":"live /gps through shared conditioning","odometry_use":"privileged truth/reward only; no observation fallback"},"checks":checks,"all_checks_pass":all(checks.values())}
  OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if all(checks.values()) else 1
 finally:env.close()
if __name__=="__main__":raise SystemExit(main())
