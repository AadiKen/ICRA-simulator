"""Validate and summarize native four-arm actuator-envelope captures."""
from __future__ import annotations
import argparse,json,math
from pathlib import Path

ARMS=("bcod-sim","gazebo","holoocean","stonefish")
LEVELS=(.25,.5,.75,1.)

def _finite(value,label):
 if not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError(f"{label} must be finite")
 return float(value)

def validate_arm(data,arm):
 if data.get("schema_version")!=1 or data.get("artifact_kind")!="native-actuator-envelope-capture" or data.get("arm")!=arm:raise ValueError(f"{arm}: invalid capture identity")
 if data.get("disturbance")!={"current_mps":0.0,"wind_mps":0.0,"waves":"off"}:raise ValueError(f"{arm}: calm-water contract mismatch")
 rows=data.get("speed_curve",[])
 if [float(x.get("command_level",-1)) for x in rows]!=list(LEVELS):raise ValueError(f"{arm}: speed curve must contain ordered 25/50/75/100% rows")
 for row in rows:
  for key in ("steady_speed_mps","steady_window_speed_slope_mps2","commanded_force_n_per_thruster","delivered_force_n_per_thruster"):_finite(row.get(key),f"{arm}.{key}")
  if abs(row["steady_window_speed_slope_mps2"])>.005:raise ValueError(f"{arm}: command {row['command_level']:.2f} did not reach steady state")
  if abs(row["commanded_force_n_per_thruster"]-row["delivered_force_n_per_thruster"])>max(.5,.01*abs(row["commanded_force_n_per_thruster"])):raise ValueError(f"{arm}: command/delivered force mismatch exceeds tolerance")
  times=row.get("time_to_speed_s",{})
  for threshold in ("0.5","1.0","1.5"):
   if threshold not in times:raise ValueError(f"{arm}: missing time-to-{threshold} m/s")
   if times[threshold] is not None:_finite(times[threshold],f"{arm}.time_to_speed_s.{threshold}")
 turning=data.get("turning",{})
 for probe in ("heading_step","waypoint_90deg"):
  if probe not in turning:raise ValueError(f"{arm}: missing {probe} probe")
 for key in ("settling_time_s","overshoot_deg"):_finite(turning["heading_step"].get(key),f"{arm}.heading_step.{key}")
 for key in ("turn_time_s","distance_cost_m"):_finite(turning["waypoint_90deg"].get(key),f"{arm}.waypoint_90deg.{key}")
 return data

def rel_gap(a,b):return abs(a-b)/max(abs(a),abs(b),1e-12)

def analyze(captures):
 rows={arm:validate_arm(captures[arm],arm) for arm in ARMS};pairwise=[]
 metrics=(("full_command_speed_mps",lambda x:x["speed_curve"][-1]["steady_speed_mps"]),("heading_settling_time_s",lambda x:x["turning"]["heading_step"]["settling_time_s"]),("waypoint_turn_time_s",lambda x:x["turning"]["waypoint_90deg"]["turn_time_s"]))
 for i,a in enumerate(ARMS):
  for b in ARMS[i+1:]:
   for name,get in metrics:
    av,bv=get(rows[a]),get(rows[b]);gap=rel_gap(av,bv);pairwise.append({"arms":[a,b],"metric":name,"a":av,"b":bv,"relative_gap":gap,"flagged":gap>.10})
 operating={arm:rows[arm].get("policy_operating_point") for arm in ARMS};missing_operating=[arm for arm,value in operating.items() if not value]
 recommendation={"option":None,"status":"requires-decision","reason":None}
 if missing_operating:recommendation["reason"]="Option 3 is the default, but policy operating-point captures are missing for: "+", ".join(missing_operating)
 else:recommendation={"option":3,"status":"recommended-not-applied","reason":"Task-operating-point parity compares the command, speed, and acceleration regime used by each policy on the candidate route."}
 return {"schema_version":1,"artifact_kind":"cross-simulator-actuator-envelope-analysis","caps_changed":False,"arms":rows,"pairwise":pairwise,"pairwise_flag_threshold":.10,"recommendation":recommendation,"invalidated_after_future_cap_change":["HoloOcean current-only 200-seed nominal calibration","four-arm zero-current/zero-wind nominal calibration","HoloOcean reachability/theoretical-minimum-time calculation","Gate 1 saturation classifications using the old HoloOcean cap"],"not_invalidated":["harness mechanics","energy-cap implementation","time-cap fix","Gates 2-4 status"]}

def main():
 p=argparse.ArgumentParser();p.add_argument("--input-dir",type=Path,required=True);p.add_argument("--output",type=Path,required=True);a=p.parse_args();captures={arm:json.loads((a.input_dir/f"{arm}.json").read_text()) for arm in ARMS};report=analyze(captures);a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
if __name__=="__main__":main()
