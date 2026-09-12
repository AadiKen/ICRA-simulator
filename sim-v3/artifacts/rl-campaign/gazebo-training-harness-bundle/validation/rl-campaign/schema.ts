export const RL_CAMPAIGN_SCHEMA_VERSION="1.0.0" as const;
export const TERMINATION_REASONS=["success","collision","grounding","timeout","allocation_failure","instability","other"] as const;
export const COLLISION_TYPES=["none","object","grounding"] as const;
export type EpisodeMetric={run_id:string;seed:number;simulator:string;vehicle:string;task_id:string;task_portable:boolean;policy_id:string;algorithm:string;return:number;success:boolean;episode_length:number;wall_clock_s:number;termination_reason:typeof TERMINATION_REASONS[number];collision_type:typeof COLLISION_TYPES[number];host_class:"local"|"cluster"|"synthetic"};
export const EPISODE_COLUMNS=["run_id","seed","simulator","vehicle","task_id","task_portable","policy_id","algorithm","return","success","episode_length","wall_clock_s","termination_reason","collision_type","host_class"] as const;
export function validateEpisode(row:Record<string,unknown>):asserts row is EpisodeMetric{
  for(const key of EPISODE_COLUMNS)if(!(key in row))throw new Error(`Missing episode column: ${key}`);
  for(const key of ["run_id","simulator","vehicle","task_id","policy_id","algorithm"])if(typeof row[key]!=="string"||!row[key])throw new Error(`Invalid ${key}`);
  for(const key of ["seed","return","episode_length","wall_clock_s"])if(typeof row[key]!=="number"||!Number.isFinite(row[key]))throw new Error(`Invalid ${key}`);
  if(!Number.isInteger(row.seed)||!Number.isInteger(row.episode_length)||Number(row.episode_length)<0||Number(row.wall_clock_s)<0)throw new Error("Invalid non-negative integer/count field");
  if(typeof row.success!=="boolean"||typeof row.task_portable!=="boolean"||!TERMINATION_REASONS.includes(row.termination_reason as never)||!COLLISION_TYPES.includes(row.collision_type as never))throw new Error("Invalid outcome enum");
  if(row.termination_reason==="grounding"&&row.collision_type!=="grounding")throw new Error("Grounding must remain distinct from object collision");
  if(row.termination_reason==="collision"&&row.collision_type!=="object")throw new Error("Object collision requires collision_type=object");
}
