#!/usr/bin/env python3
"""Run Gate C acceptance checks and stop; never trains a policy."""
from __future__ import annotations
import json,math,platform,statistics,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import CommonWaypointEnv,GazeboGymEnv

RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]
OUT=ROOT/"artifacts/rl-campaign/gazebo-gate-c.json"
SEED=30000

def gaz_env():return GazeboGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=SEED)
def angle_error(a,b):return abs((a-b+math.pi)%(2*math.pi)-math.pi)
def reset_snapshot():
    env=gaz_env()
    try:
        _,info=env.reset();return env.last_truth,info,env._config(SEED)
    finally:env.close()

def main():
    first,first_info,config=reset_snapshot();second,_,_=reset_snapshot()
    reset_position_delta=math.dist(first["position_ned_m"][:2],second["position_ned_m"][:2]);reset_heading_delta=angle_error(first["attitude_rad"][2],second["attitude_rad"][2])
    bcod=CommonWaypointEnv(ROOT,fixed_reset_seed=SEED);gaz=gaz_env();divergences=[[] for _ in range(15)]
    try:
        bobs,binfo=bcod.reset();gobs,ginfo=gaz.reset();gaz_start_time=gaz.bridge._request({"op":"diagnostic_status"})["simulation_time_s"]
        expected_position=bcod.last_truth["position_ned_m"][:2];expected_heading=bcod.last_truth["attitude_rad"][2]
        seeded_position_error=math.dist(gaz.last_truth["position_ned_m"][:2],expected_position);seeded_heading_error=angle_error(gaz.last_truth["attitude_rad"][2],expected_heading)
        for i in range(12):
            action=np.asarray([.2+0.01*i,.2-0.005*i],np.float32)
            bobs,_,bt,btr,_=bcod.step(action);gobs,_,gt,gtr,_=gaz.step(action)
            for field,(a,b) in enumerate(zip(bobs,gobs)):divergences[field].append(abs(float(a)-float(b)))
            if bt or btr or gt or gtr:break
        gaz_status=gaz.bridge._request({"op":"diagnostic_status"});gaz_elapsed=gaz_status["simulation_time_s"]-gaz_start_time
        controls=len(divergences[0])
    finally:bcod.close();gaz.close()
    names=["accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","yaw","goal_north","goal_east","fix_valid","prev_port","prev_starboard","prev_steer_0","prev_steer_1","time_remaining"]
    per_field={name:{"median_abs":statistics.median(values),"max_abs":max(values)} for name,values in zip(names,divergences)}
    adapter=(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py").read_text()
    termination_support={
      "success":{"implemented":True,"source":"shared CommonWaypointEnv 2.0 m pass-through"},
      "timeout":{"implemented":True,"source":"shared 2400-physics-step timeout"},
      "grounding":{"implemented":False,"source":"no live Gazebo contact/bathymetry classification in adapter"},
      "object_collision":{"implemented":False,"source":"no live Gazebo contact classification in adapter"},
      "instability":{"implemented":False,"source":"adapter does not emit 60-degree/1-second instability stop_reason"},
      "allocation_failure":{"implemented":False,"source":"shared actuator bridge does not return allocation-failure diagnostics to runtime"}}
    checks={
      "reset_determinism":reset_position_delta<=.05 and reset_heading_delta<=math.radians(.5),
      "seeded_reset_parity_post_heading_fix":seeded_position_error<=.25 and seeded_heading_error<=math.radians(1),
      "wind_and_current_applied":False,
      "step_timing_and_freshness":abs(gaz_elapsed-controls*.1)<1e-9 and json.loads((ROOT/"artifacts/rl-campaign/gazebo-15-field-live-gate-a.json").read_text())["all_checks_pass"],
      "all_termination_modes":all(x["implemented"] for x in termination_support.values()),
      "observation_equivalence":False
    }
    report={"schema_version":1,"artifact_kind":"gazebo-gate-c-closed-loop-acceptance","status":"PASS" if all(checks.values()) else "FAIL","contract_sha256":"2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36","training_authorized":False,"next_gate_started":False,
      "runtime":{"image":"icra27-gazebo-harmonic:harmonic-8.15.0","image_id":"174e8baad590","host_architecture":platform.machine(),"container_architecture":"linux/amd64","execution":"AMD64 emulation on Apple Silicon","timing_basis":"Gazebo simulation time only"},
      "reset_determinism":{"seed":SEED,"tolerance":{"position_m":.05,"heading_deg":.5},"position_delta_m":reset_position_delta,"heading_delta_rad":reset_heading_delta,"passed":checks["reset_determinism"]},
      "seeded_reset_parity":{"reference":"bcod-sim live CommonWaypointEnv post-heading-fix reset; expected heading is seeded, not constant pi/4","expected_position_ned_m":expected_position,"gazebo_position_ned_m":first["position_ned_m"][:2],"position_error_m":seeded_position_error,"expected_heading_rad":expected_heading,"gazebo_heading_rad":first["attitude_rad"][2],"heading_error_rad":seeded_heading_error,"passed":checks["seeded_reset_parity_post_heading_fix"]},
      "disturbances":{"requested":{"current_mps_ned":config["environment"]["current_mps"],"wind_mps_ned":config["environment"]["wind_mps"]},"applied":{"current":False,"wind":False},"finding":"gazebo_gym_runtime.py currently serializes neither reset disturbance into a simulator system; the generated world contains only current metadata and has no wind application system.","passed":False},
      "step_timing":{"control_steps":controls,"expected_elapsed_simulation_s":controls*.1,"observed_elapsed_simulation_s":gaz_elapsed,"freshness_gate_a_reused":True,"passed":checks["step_timing_and_freshness"]},
      "termination_parity":{"conditions":termination_support,"passed":checks["all_termination_modes"]},
      "observation_equivalence":{"seed":SEED,"script":"12 identical deterministic differential-thrust controls","fields":per_field,"passed":False,"failure_reason":"Multiple divergences cannot yet be attributed only to sensor noise because the Gazebo plant becomes vertically/attitudinally unstable and does not track the bcod-sim trajectory."},
      "checks":checks,"all_checks_pass":all(checks.values()),"blocking_rework":["Apply and verify seeded wind and current in the running Gazebo simulator.","Add contact/bathymetry mapping for grounding versus object collision.","Emit instability and allocation-failure termination signals.","Repair the Gazebo plant instability, then repeat stepwise observation equivalence."],"notes":["No training was run.","Gate C failure blocks reward/action parity and Gate D."]}
    OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if all(checks.values()) else 1
if __name__=="__main__":raise SystemExit(main())
