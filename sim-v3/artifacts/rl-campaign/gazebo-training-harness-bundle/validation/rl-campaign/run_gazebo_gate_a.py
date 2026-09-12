#!/usr/bin/env python3
"""Execute Gate A through the live GazeboGymEnv observation path; no training."""
from __future__ import annotations
import hashlib,json,platform,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import GazeboGymEnv

RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]
OUT=ROOT/"artifacts/rl-campaign/gazebo-15-field-live-gate-a.json"
PHYSICS_DT_S=.05
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    env=GazeboGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=30000)
    checks={}; rows=[]
    try:
        obs,reset_info=env.reset(); rows.append(obs.tolist())
        reset_status=env.bridge._request({"op":"diagnostic_status"});reset_sim_time=reset_status["simulation_time_s"]
        checks["exactly_15_fields"]=len(obs)==15
        checks["reset_seed"]=reset_info["seed"]==30000
        first_gps_release=None
        # Equal positive commands actively drive the live trajectory. Each Gym
        # step advances exactly 0.1 s (two adapter physics-step requests).
        first_gps_release_sim_time=None;first_gps_source_time=None
        for control_step in range(15):
            obs,_,terminated,truncated,info=env.step(np.asarray([.2,.2],np.float32));rows.append(obs.tolist())
            if obs[9]==1 and first_gps_release is None:
                first_gps_release=info["terminal_state"].copy();first_gps_release_control=control_step+1
                release_status=env.bridge._request({"op":"diagnostic_status"});first_gps_release_sim_time=release_status["simulation_time_s"];first_gps_source_time=release_status["gps_conditioning"]["held_source_timestamp_s"]
            if terminated or truncated:raise RuntimeError(f"unexpected early termination: {info['termination_reason']}")
        status=env.bridge._request({"op":"diagnostic_status"})
        gps_ts=status["topic_timestamps_s"]["gps"]
        gps_deltas=[round(b-a,9) for a,b in zip(gps_ts,gps_ts[1:]) if b>a]
        # Gazebo may emit one initialization sample before settling onto the
        # configured schedule. Rate assessment begins with the first 0.5 s gap.
        scheduled_gps_deltas=gps_deltas[1:] if gps_deltas and gps_deltas[0] != .5 else gps_deltas
        checks["gps_2_hz_sim_time"]=len(scheduled_gps_deltas)>=1 and all(abs(x-.5)<1e-9 for x in scheduled_gps_deltas)
        checks["gps_conditioning_config"]=all(status["gps_conditioning"][k]==v for k,v in {"rate_hz":2.0,"latency_s":.2,"position_std_m":.8}.items())
        checks["gps_released_on_connected_path"]=first_gps_release is not None
        gps_release_age=None if first_gps_release_sim_time is None else first_gps_release_sim_time-first_gps_source_time
        checks["gps_release_latency_sim_time"]=gps_release_age is not None and .2-1e-9<=gps_release_age<=.2+PHYSICS_DT_S+1e-9
        # Stall live IMU ingestion while /clock continues. One 0.1 s Gym step
        # exceeds its 0.08 s age budget, so all seven IMU fields must zero.
        env.bridge._request({"op":"diagnostic_stall","sensor":"imu","enabled":True})
        imu_stale_obs,*_=env.step(np.asarray([.2,.2],np.float32));rows.append(imu_stale_obs.tolist())
        checks["active_imu_stall_zero_fills"]=bool(np.all(imu_stale_obs[:7]==0))
        env.bridge._request({"op":"diagnostic_stall","sensor":"imu","enabled":False})
        imu_fresh_obs,*_=env.step(np.asarray([.2,.2],np.float32));rows.append(imu_fresh_obs.tolist())
        checks["imu_recovers_from_live_topic"]=bool(np.any(imu_fresh_obs[:7]!=0))
        # Stall GPS callbacks but retain the last message object. Advance beyond
        # the 0.75 s hold/latency/physics-slack budget and require zero-fill.
        env.bridge._request({"op":"diagnostic_stall","sensor":"gps","enabled":True})
        gps_stall_rows=[]
        for _ in range(8):
            stale,*_=env.step(np.asarray([.2,.2],np.float32));gps_stall_rows.append(stale);rows.append(stale.tolist())
        checks["active_gps_stall_zero_fills"]=bool(np.all(gps_stall_rows[-1][7:10]==0))
        checks["gps_did_not_refresh_from_retained_object"]=bool(gps_stall_rows[-1][9]==0)
        final_status=env.bridge._request({"op":"diagnostic_status"})
        expected_final_sim_time=reset_sim_time+(15+1+1+8)*.1
        checks["fixed_simulation_step_timing"]=abs(final_status["simulation_time_s"]-expected_final_sim_time)<1e-9
        report={"schema_version":3,"artifact_kind":"gazebo-15-field-live-sensor-gate-a","status":"PASS" if all(checks.values()) else "FAIL",
          "contract_sha256":env.EXPECTED_CONTRACT_SHA256,"training_authorized":False,"next_gate_started":False,
          "runtime":{"image":"icra27-gazebo-harmonic:harmonic-8.15.0","image_id":"174e8baad590","gazebo_sim_version":"8.15.0","host_architecture":platform.machine(),"container_architecture":"linux/amd64","execution":"AMD64 emulation on Apple Silicon","timing_basis":"/world/.../clock and sensor header simulation timestamps only; wall clock excluded"},
          "connected_path":{"entrypoint":"GazeboGymEnv -> JsonLineSimulatorBridge -> gazebo_gym_runtime.py","adapter":"validation/rl-campaign/ports/gazebo_gym_runtime.py","adapter_sha256":sha(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py"),"gps_conditioner":"validation/rl-campaign/ports/external_sensor_model.py","gps_conditioner_sha256":sha(ROOT/"validation/rl-campaign/ports/external_sensor_model.py"),"frame_converter":"validation/rl-campaign/ports/task-trace-bridge.ts via task-trace-jsonl-bridge.ts","actuator":"validation/rl-campaign/ports/shared-actuators.ts via actuator-jsonl-bridge.ts"},
          "simulation_timing":{"physics_step_s":.05,"gym_control_interval_s":.1,"post_reset_simulation_epoch_s":reset_sim_time,"note":"Harmonic model reset preserves the already-paused /clock epoch; interval assertions are relative to this epoch.","expected_final_simulation_time_s":expected_final_sim_time,"final_simulation_time_s":final_status["simulation_time_s"],"gps_timestamps_s":gps_ts,"gps_positive_deltas_s":gps_deltas,"gps_scheduled_deltas_s":scheduled_gps_deltas,"first_conditioned_gps_fix_control_step":first_gps_release_control if first_gps_release else None,"first_conditioned_gps_source_timestamp_s":first_gps_source_time,"first_conditioned_gps_release_simulation_time_s":first_gps_release_sim_time,"first_conditioned_gps_release_age_s":gps_release_age,"release_tolerance":"configured 0.2 s plus at most one 0.05 s physics tick"},
          "observation":{"field_count":15,"rows_examined":len(rows),"imu_fields_source":"live /imu linearAcceleration, angularVelocity, orientation","gps_fields_source":"live /gps through ExternalGpsModel","missing_invalid_stale_policy":"zero-fill","odometry_use":"privileged truth/reward/termination only; never observation fallback"},
          "active_staleness":{"method":"Ignore live topic callbacks while retaining the last message object; advance Gazebo by fixed simulation steps through the same Gym endpoint.","imu_age_limit_s":env.SENSOR_MAX_AGE_S["imu"],"gps_age_limit_s":env.SENSOR_MAX_AGE_S["gps"]},
          "checks":checks,"all_checks_pass":all(checks.values()),"notes":["No training was run.","Gazebo plant vertical instability observed during the sensor run is retained for later gate diagnosis; it did not alter the sensor source or simulation-time checks."]}
        OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if all(checks.values()) else 1
    finally:env.close()
if __name__=="__main__":raise SystemExit(main())
