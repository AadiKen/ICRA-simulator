#!/usr/bin/env python3
"""Vehicle A / Surveyor 120 s zero-command Gazebo stability acceptance."""
from __future__ import annotations
import json,math,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import GazeboGymEnv
RUNTIME=["/usr/bin/env","BCOD_GAZEBO_IDLE_MODE=1",sys.executable,str(ROOT/"validation/rl-campaign/ports/gazebo_gym_runtime.py")]
OUT=ROOT/"artifacts/rl-campaign/gazebo-vehicle-a-idle-stability.json"
def main():
 with tempfile.TemporaryDirectory(prefix="bcod-vehicle-a-sdf-") as generated:
  subprocess.run(["node","--experimental-strip-types",str(ROOT/"validation/rl-campaign/ports/prepare-gazebo-episode.ts"),"30000",generated,"--idle-no-sensors"],cwd=ROOT,check=True,capture_output=True,text=True)
  sdf_checks={}
  for name,path in (("world","/gate/worlds/gate-30000.sdf"),("model","/gate/models/surveyor/model.sdf")):
   checked=subprocess.run(["docker","run","--rm","--platform","linux/amd64","-e","GZ_SIM_RESOURCE_PATH=/gate/models","-v",f"{generated}:/gate","174e8baad590","gz","sdf","-k",path],capture_output=True,text=True)
   sdf_checks[name]={"passed":checked.returncode==0 and "Valid" in checked.stdout,"output":(checked.stdout+checked.stderr).strip()}
 env=GazeboGymEnv(ROOT,RUNTIME,allow_unconformant_diagnostic=True,fixed_reset_seed=30000)
 try:
  env.reset();result=env.bridge._request({"op":"diagnostic_idle","physics_steps":2400})
 finally:env.close()
 checks={"simulated_120_s":abs((result["end_simulation_time_s"]-result["start_simulation_time_s"])-120)<1e-8,"floats_without_sinking_or_launching":abs(result["vertical_displacement_m"])<.01 and result["vertical_range_m"]<.02,"zero_command_no_net_motion":result["horizontal_displacement_m"]<.01 and result["max_speed_m_s"]<.05,"roll_bounded":result["max_abs_roll_rad"]<math.radians(5),"pitch_bounded":result["max_abs_pitch_rad"]<math.radians(5)}
 checks["sdf_validates_under_pinned_8_15"]=sdf_checks["model"]["passed"]
 checks["headless_world_loads"]=result.get("ok") is True
 sdf_checks["world"]={"passed":result.get("ok") is True,"validation_method":"pinned Gazebo headless parse and load with model URI resolver","standalone_gz_sdf_note":"gz sdf -k does not install a model:// find callback; the included model is validated separately"}
 report={"schema_version":1,"artifact_kind":"gazebo-vehicle-a-idle-stability","status":"PASS" if all(checks.values()) else "FAIL","runtime":{"image":"icra27-gazebo-harmonic:harmonic-8.15.0","image_id":"174e8baad590","execution":"linux/amd64 emulation on Apple Silicon","timing_basis":"Gazebo simulation time","throughput_interpretation":"not representative under emulation"},"plant_audit":{"mass_kg":52.3,"water_density_kg_m3":1025,"required_displaced_volume_m3":0.05102439024390244,"declared_proxy_full_volume_m3":0.32808,"buoyancy_collision_height_scale":0.31104846527616703,"equilibrium_submerged_fraction":0.5,"inertia_kg_m2":{"ixx":5.5058825,"iyy":13.4780586667,"izz":17.8912198333,"derivation":"component box geometry plus parallel-axis theorem about the assumed CG","triangle_inequality_pass":True},"center_of_mass_body_m":[0,0,-0.05],"center_of_mass_status":"stability-fixture engineering assumption; source CG is unmeasured","center_of_buoyancy_z_at_equilibrium_m":-0.02230925148354192,"metacentric_height_m":{"transverse":1.9638687121875864,"longitudinal":5.583521962665597},"hydrodynamic_damping":{"linear":{"Zw":-1200,"Kp":-100,"Mq":-250},"quadratic":{"Zww":-200,"Kpp":-20,"Mqq":-50},"status":"unmeasured stability-fixture estimates"},"reset_lifecycle_fix":"fresh generated world initialized once; reset-all prohibited because it removes Harmonic buoyancy enabled-entity state"},"sdf_validation":sdf_checks,"acceptance_tolerances":{"vertical_displacement_m":.01,"vertical_range_m":.02,"horizontal_displacement_m":.01,"max_speed_m_s":.05,"max_abs_roll_pitch_deg":5},"measurement":result,"checks":checks,"all_checks_pass":all(checks.values()),"training_authorized":False,"next_step_started":False}
 OUT.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));return 0 if all(checks.values()) else 1
if __name__=="__main__":raise SystemExit(main())
