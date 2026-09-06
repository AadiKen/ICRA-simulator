import {createHash} from "node:crypto";
import {readFile,writeFile} from "node:fs/promises";

const path="artifacts/rl-campaign/surveyor/task-contract-frozen.json";
const expectedOldHash="7e0bedf89d58dd18b32508f430f6bee10d62de20be630a2d0dabbe7464b11e59";
const stable=(v:unknown):string=>Array.isArray(v)?`[${v.map(stable).join(",")}]`:v&&typeof v==="object"?`{${Object.entries(v as Record<string,unknown>).sort(([a],[b])=>a.localeCompare(b)).map(([k,x])=>`${JSON.stringify(k)}:${stable(x)}`).join(",")}}`:JSON.stringify(v);
const document=JSON.parse(await readFile(path,"utf8")),{content_sha256:_old,hash_scope:_scope,...contract}=document;
if(document.content_sha256!==expectedOldHash)throw new Error(`Refusing to revise unexpected contract ${document.content_sha256}`);
const common=contract.tasks.find((task:any)=>task.task_id==="common-waypoint-transit-v1"),gps=common.observation.components.find((component:any)=>component.id==="gps"),time=common.observation.components.find((component:any)=>component.id==="time_remaining");
if(gps.size!==5||JSON.stringify(gps.fields)!==JSON.stringify(["relative_goal_north_m","relative_goal_east_m","ground_speed_north_m_s","ground_speed_east_m_s","fix_valid"]))throw new Error("Unexpected source GPS observation contract.");
gps.size=3;gps.fields=["relative_goal_north_m","relative_goal_east_m","fix_valid"];
gps.portability_note="Use each simulator's standard GPS/NavSat position output converted to the contract's local tangent frame. Ground velocity is uniformly excluded.";
time.append_only_note="Field 15 after the 7-field IMU, 3-field GPS, and 4-field previous-action groups.";
contract.contract_version="7.0.0-surveyor-15-field-gps-parity";
contract.status="FROZEN_15_FIELD_GPS_PARITY_REVISION";
contract.version_note="Authorized 2026-09-06: the common portable observation is reduced uniformly from 17 to 15 fields. Reward, termination, curriculum, action space, randomization, acceptance thresholds, and every non-observation execution setting remain unchanged.";
contract.observation_revision={
  authorized_date:"2026-09-06",
  diagnosis_artifact:"artifacts/rl-campaign/gps-velocity-parity-diagnosis.json",
  supersedes_contract_sha256:expectedOldHash,
  old_field_count:17,new_field_count:15,
  removed_fields:["gps.ground_speed_north_m_s","gps.ground_speed_east_m_s"],
  rationale:"Ground velocity was removed to preserve cross-simulator observation parity. No external runtime provides a GPS-derived velocity that is not simulator-truth-derived: Gazebo's gz.msgs.NavSat velocity fields are populated from world velocity, the ROS NavSatFix conversion discards velocity entirely, and VRX exposes no separate GPS velocity topic. Deriving velocity by differencing 2 Hz NavSat fixes yields approximately 2.19 m/s noise against bcod-sim's 0.08 m/s, and filtering to bcod-sim's level requires an approximately 14 s window with approximately 7 s group delay—not a comparable sensor. bcod-sim's own 0.08 m/s value was introduced without a datasheet, citation, or stated physical basis, and was produced by adding independent Gaussian noise to ground-truth velocity rather than by modeling Doppler-derived measurement. The field was therefore neither attributable nor reproducible across implementations, and is removed uniformly rather than retained asymmetrically."
};
const content_sha256=createHash("sha256").update(stable(contract)).digest("hex");
await writeFile(path,JSON.stringify({...contract,content_sha256,hash_scope:"canonical JSON of every field except content_sha256 and hash_scope"},null,2)+"\n");
console.log(content_sha256);
