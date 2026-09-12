import {readFileSync} from "node:fs";
import {resolve} from "node:path";

export const FROZEN_TASK_CONTRACT_PATH=resolve("artifacts/rl-campaign/surveyor/task-contract-frozen.json");

export function loadFrozenTaskContract(){
  const document=JSON.parse(readFileSync(FROZEN_TASK_CONTRACT_PATH,"utf8"));
  const task=document.tasks?.find((candidate:any)=>candidate.task_id==="common-waypoint-transit-v1");
  if(!task)throw new Error("Frozen common-waypoint-transit-v1 contract is absent.");
  if(typeof document.content_sha256!=="string"||document.content_sha256.length!==64)throw new Error("Frozen task contract has no valid content hash.");
  return {document,task,contentSha256:document.content_sha256 as string};
}
