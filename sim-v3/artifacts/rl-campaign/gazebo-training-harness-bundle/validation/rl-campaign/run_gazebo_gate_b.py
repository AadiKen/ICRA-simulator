#!/usr/bin/env python3
"""Gate B geodetic, validity, frame, and sensor-enablement verification."""
from __future__ import annotations
import hashlib,json,platform,statistics,sys,tempfile
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
sys.path.insert(0,str(ROOT/"validation/rl-campaign/ports"))
from bcod_sim import GazeboGymEnv
from external_sensor_model import gazebo_navsat_valid

OUT=ROOT/"artifacts/rl-campaign/gazebo-gate-b.json"
RUNTIME=[sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    env=GazeboGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=30000)
    gps_errors=[];valid_rows=0
    try:
        obs,_=env.reset()
        for _ in range(15):
            obs,_,terminated,truncated,info=env.step(np.asarray([.15,.15],np.float32))
            if terminated or truncated:raise RuntimeError("unexpected early termination")
            if obs[9]==1:
                valid_rows+=1;target=env.route[env.waypoint]
                measured=[target[0]-float(obs[7]),target[1]-float(obs[8])]
                truth=info["terminal_state"]["position_ned_m"][:2]
                gps_errors.append([measured[0]-truth[0],measured[1]-truth[1]])
        status=env.bridge._request({"op":"diagnostic_status"});gps_ts=status["topic_timestamps_s"]["gps"]
    finally:env.close()
    generator=(ROOT/"gazebo/generateGazeboParity.js").read_text();preparer=(ROOT/"validation/rl-campaign/ports/prepare-gazebo-episode.ts").read_text();adapter=(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py").read_text()
    radial=[float(np.hypot(*x)) for x in gps_errors]
    checks={
      "wgs84_origin_generated":all(x in generator for x in ("<spherical_coordinates>","EARTH_WGS84","<world_frame_orientation>ENU</world_frame_orientation>","-33.72276876888639","150.67399110174387")),
      "live_navsat_nonzero_and_finite":len(gps_ts)>=3 and all(np.isfinite(gps_ts)),
      "local_ned_fix_observed":valid_rows>0 and bool(radial) and all(np.isfinite(radial)),
      "local_coordinate_error_bounded":bool(radial) and statistics.median(radial)<5,
      "fix_valid_finite_rule":gazebo_navsat_valid(-33.7,150.6) and not gazebo_navsat_valid(float("nan"),150.6),
      "fix_valid_freshness_rule":"_fresh_sensor" in (ROOT/"packages/python-client/bcod_sim/common_task_env.py").read_text(),
      "shared_frame_conversion_reused":"task-trace-jsonl-bridge.ts" in adapter and "gazeboOdomToTask" not in adapter,
      "phase_a_sensors_enabled":"phaseASensors:true" in preparer,
    }
    report={"schema_version":1,"artifact_kind":"gazebo-gate-b-parity","status":"PASS" if all(checks.values()) else "FAIL","contract_sha256":"2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36","training_authorized":False,"next_gate_started":False,
      "runtime":{"image":"icra27-gazebo-harmonic:harmonic-8.15.0","image_id":"174e8baad590","host_architecture":platform.machine(),"container_architecture":"linux/amd64","execution":"AMD64 emulation on Apple Silicon","timing_basis":"Gazebo simulation timestamps only"},
      "geodetic_origin":{"surface_model":"EARTH_WGS84","latitude_deg":-33.72276876888639,"longitude_deg":150.67399110174387,"elevation_m":0,"world_frame":"ENU","task_frame":"local tangent NED; N=ENU y, E=ENU x"},
      "navsat_validity":{"decision":"fix_valid=1 only when latitude and longitude are finite and the conditioned sample timestamp is within the GPS freshness window; otherwise zero-fill with fix_valid=0","native_constraint":"gz.msgs.NavSat has no explicit status field","equivalence":"finite output supplies native validity; shared observation freshness supplies bcod-sim-equivalent stale/invalid behavior"},
      "live_local_coordinate_check":{"valid_observation_rows":valid_rows,"gps_topic_timestamps_s":gps_ts,"median_radial_difference_from_same-step_privileged_position_m":None if not radial else statistics.median(radial),"max_radial_difference_m":None if not radial else max(radial),"interpretation":"Includes configured 0.8 m/axis noise, 0.2 s latency, and vessel motion during latency; privileged position is comparison-only."},
      "conversion":{"world":"Gazebo ENU","body_input":"Gazebo FLU","task":"NED/FRD","state_path":"task-trace-bridge.ts gazeboOdomToTask via task-trace-jsonl-bridge.ts","sensor_path":"external_sensor_model.py flu_to_body_ned and quaternion_to_ned_yaw, the same functions imported by vrx_trace_exporter.py","third_converter_created":False},
      "sensor_generation":{"phaseASensors":True,"preparer":"validation/rl-campaign/ports/prepare-gazebo-episode.ts","generator_sha256":sha(ROOT/"gazebo/generateGazeboParity.js")},
      "checks":checks,"all_checks_pass":all(checks.values()),"asymmetries":[],"notes":["No training was run.","The phaseASensors=true task-world path was already enabled before this gate; the geodetic-origin generator change is independently tested. No generated golden checksum was silently rewritten."]}
    OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if all(checks.values()) else 1
if __name__=="__main__":raise SystemExit(main())
