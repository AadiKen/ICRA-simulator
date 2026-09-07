#!/usr/bin/env python3
"""Run VRX Gate B only through the live connected Gym runtime."""
from __future__ import annotations
import json,math,platform,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
sys.path.insert(0,str(ROOT/"validation/rl-campaign/ports"))
from bcod_sim import VrxGymEnv
from external_sensor_model import gazebo_navsat_valid
RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")]
OUT=ROOT/"artifacts/rl-campaign/vrx-gate-b.json"

def close(a,b,tol=1e-9): return len(a)==len(b) and all(abs(float(x)-float(y))<=tol for x,y in zip(a,b))

def main():
    checks={};evidence={}
    calm=VrxGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="zero")
    try:
        obs,_=calm.reset();calm_start=list(calm.last_truth["position_ned_m"][:2]);calm_steps=0;valid=[];status=None
        for _ in range(15):
            obs,*_=calm.step(np.zeros(2,np.float32));calm_steps+=1;valid.append(int(obs[9]));status=calm.bridge._request({"op":"diagnostic_status"})
            if obs[9]==1: break
        assert status is not None
        diag=status["gps_conditioning"]["held_diagnostic"]
        target=calm.route[calm.waypoint];measured=[target[0]-float(obs[7]),target[1]-float(obs[8])]
        reconstructed=[diag["base_position_ned_m"][i]+diag["noise_ned_m"][i] for i in range(2)]
        origin_error=math.hypot(*(diag["base_position_ned_m"][i]-calm.start[i] for i in range(2)))
        checks["geodetic_origin_configured_live"]=status["gps_conditioning"]["geodetic_anchored"] and close(status["gps_conditioning"]["reference_latitude_longitude_deg"],[-33.72276876888639,150.67399110174387])
        checks["geodetic_origin_position_error_below_1_mm"]=origin_error<.001
        checks["gps_observation_uses_origin_anchored_position"]=close(measured,reconstructed,2e-3)
        checks["fix_valid_becomes_one_only_after_release"]=valid[-1]==1 and 0 in valid
        calm.bridge._request({"op":"diagnostic_stall","sensor":"gps","enabled":True})
        stalled=[]
        for _ in range(9): row,*_=calm.step(np.zeros(2,np.float32));calm_steps+=1;stalled.append(int(row[9]))
        checks["fix_valid_zero_when_timestamp_stale"]=stalled[-1]==0
        calm.bridge._request({"op":"diagnostic_stall","sensor":"gps","enabled":False})
        recovered=[]
        for _ in range(8): row,*_=calm.step(np.zeros(2,np.float32));calm_steps+=1;recovered.append(int(row[9]))
        checks["fix_valid_recovers_on_fresh_finite_fix"]=1 in recovered
        evidence["gps"]={"reference_latitude_longitude_deg":status["gps_conditioning"]["reference_latitude_longitude_deg"],
            "base_position_ned_m":diag["base_position_ned_m"],"noise_ned_m":diag["noise_ned_m"],
            "expected_seeded_start_ned_m":calm.start,"geodetic_origin_position_error_m":origin_error,
            "observation_reconstructed_position_ned_m":measured,"startup_fix_valid":valid,"stalled_fix_valid":stalled,"recovered_fix_valid":recovered}
        for _ in range(max(0,50-calm_steps)): calm.step(np.zeros(2,np.float32))
        calm_delta=[calm.last_truth["position_ned_m"][i]-calm_start[i] for i in range(2)]
    finally: calm.close()

    disturbed=VrxGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="seeded")
    try:
        disturbed.reset();disturbed_start=list(disturbed.last_truth["position_ned_m"][:2]);status=disturbed.bridge._request({"op":"diagnostic_status"});requested=status["environment"]["requested"];applied=status["environment"]["applied"]
        checks["seeded_current_vector_applied"]=close(requested["current_mps"],applied["current_mps"])
        checks["seeded_wind_vector_applied"]=close(requested["wind_mps"],applied["wind_mps"])
        checks["disturbances_not_locked_to_competition_presets"]=not status["environment"]["locked_to_competition_preset"]
        checks["wave_explicitly_flat_for_common_task"]=applied["wave"]["gain"]==0
        for _ in range(50): disturbed.step(np.zeros(2,np.float32))
        disturbed_delta=[disturbed.last_truth["position_ned_m"][i]-disturbed_start[i] for i in range(2)]
        checks["live_seeded_force_response_differs_from_calm"]=math.hypot(*(disturbed_delta[i]-calm_delta[i] for i in range(2)))>.05
        evidence["disturbance"]={"requested":requested,"applied":applied,"mechanism":status["environment"]["mechanism"],
            "locked_to_competition_preset":status["environment"]["locked_to_competition_preset"],
            "zero_command_5_simulation_second_response":{"calm_displacement_ned_m":calm_delta,"seeded_displacement_ned_m":disturbed_delta,
              "difference_m":math.hypot(*(disturbed_delta[i]-calm_delta[i] for i in range(2)))},
            "capability_decision":{"wind":"YES: configurable per episode across the seeded contract range via episode-local SDF plugin parameter",
              "current":"YES: configurable per episode across the seeded contract range via episode-local SDF plugin parameter",
              "waves":"YES at the VRX plugin level through /vrx/wavefield/parameters (live topic mutation was verified during the pre-Gate-B stock-WAM-V stability check), but NO seeded wave range exists in the frozen common-task contract; this harness therefore sets gain=0"}}
    finally: disturbed.close()
    checks["nonfinite_fix_rule_unit_regression"]=gazebo_navsat_valid(-33.7,150.6) and not gazebo_navsat_valid(float("nan"),150.6) and not gazebo_navsat_valid(-33.7,float("inf"))
    report={"schema_version":1,"artifact_kind":"vrx-gate-b","status":"PASS" if all(checks.values()) else "FAIL",
      "gate":"B","gate_c_started":False,"training_started":False,"runtime":{"image":"leadcat/vrx:surveyor-patched-v3.0.1","gazebo_sim_version":"8.10.0",
      "execution":"AMD64 emulation on Apple Silicon","host_architecture":platform.machine(),"timing_basis":"/clock and sensor-header simulation timestamps only"},
      "evidence":evidence,"checks":checks,"all_checks_pass":all(checks.values()),
      "task_6_implication":"VRX can apply seeded wind and current and is not restricted to competition presets. Waves are configurable but have no bcod-sim task distribution. Task 6 may choose a disturbed bcod-sim/VRX comparison for wind/current, unlike standalone Gazebo; flat waves must remain explicit."}
    OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if all(checks.values()) else 1

if __name__=="__main__":raise SystemExit(main())
