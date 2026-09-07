#!/usr/bin/env python3
"""Drive the live Surveyor into real grounding and object contact targets."""
import json,os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"))
from bcod_sim import VrxGymEnv
runtime=[sys.executable,str(ROOT/"validation/rl-campaign/ports/vrx_gym_runtime.py")];rows=[]
scenarios=(("grounding","grounding"),("object_collision","object_collision"))
if len(sys.argv)>1:scenarios=tuple(item for item in scenarios if item[0] in sys.argv[1:])
for scenario,expected in scenarios:
    os.environ["BCOD_VRX_COLLISION_SCENARIO"]=scenario
    env=VrxGymEnv(ROOT,runtime,allow_unconformant_diagnostic=True,fixed_reset_seed=30000,disturbance_mode="zero")
    try:
        env.reset();terminated=truncated=False;info={};controls=0
        while controls<20 and not (terminated or truncated):
            _,_,terminated,truncated,info=env.step(np.ones(2,dtype=np.float32));controls+=1
        status=env.bridge._request({"op":"diagnostic_status"});counts={name:len(values) for name,values in status["topic_timestamps_s"].items() if "contact" in name}
        rows.append({"scenario":scenario,"expected":expected,"controls":controls,"terminated":terminated,"stop_reason":info.get("termination_reason"),"contact_topic_message_counts":counts,"contact_subscriber_exit_codes":{name:code for name,code in status["topic_process_exit_codes"].items() if "contact" in name},"contact_parse_diagnostics":{name:value for name,value in status["topic_parse_diagnostics"].items() if "contact" in name},"published_contact_topics":status["vrx"]["published_contact_topics"],"final_position_ned_m":env.last_truth["position_ned_m"][:2],"passed":terminated and info.get("termination_reason")==expected and sum(counts.values())>0})
    finally:env.close()
os.environ.pop("BCOD_VRX_COLLISION_SCENARIO",None)
report={"schema_version":1,"artifact_kind":"vrx-real-contact-termination-validation","method":"zero-disturbance live worlds with physical static targets 0.7 m ahead (inside the hull forward envelope); equal positive thrust trajectory; no diagnostic termination injection","rows":rows,"all_pass":all(row["passed"] for row in rows)}
out=ROOT/"artifacts/rl-campaign/vrx-real-contact-terminations.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2));raise SystemExit(0 if report["all_pass"] else 1)
