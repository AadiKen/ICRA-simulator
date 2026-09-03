import {EPISODE_COLUMNS} from "./schema.ts";
export const FIGURE_QUERIES={
  "fig-1b-learning-curves":["task_id","task_portable","simulator","vehicle","algorithm","seed","episode_length","return","host_class"],
  "fig-1c-success":["task_id","task_portable","simulator","vehicle","algorithm","seed","success","host_class"],
  "fig-3-simulator-comparison":["task_id","task_portable","simulator","policy_id","seed","return","success","wall_clock_s","host_class"],
  "fig-5-portable-comparison":["task_id","task_portable","simulator","episode_length","wall_clock_s","return","success","host_class"],
  "fig-6-showcase-transfer":["task_id","task_portable","vehicle","policy_id","seed","return","success","host_class"],
  "fig-7-showcase-capability":["task_id","task_portable","vehicle","policy_id","seed","return","success","host_class"],
  "table-t1-outcomes":["simulator","vehicle","algorithm","success","termination_reason","collision_type","host_class"],
  "table-t2-efficiency":["simulator","algorithm","episode_length","wall_clock_s","host_class"],
  "table-t3-failures":["simulator","vehicle","termination_reason","collision_type","host_class"],
} as const;
export function validateFigureQueries():void{const columns=new Set<string>(EPISODE_COLUMNS);for(const [name,required] of Object.entries(FIGURE_QUERIES))for(const column of required)if(!columns.has(column))throw new Error(`${name} requires absent column ${column}`);}
export function assertTaskEligibleForFigure(figure:string,row:{task_id:string;task_portable:boolean;simulator:string;vehicle?:string;backend?:string}):void{if(figure==="fig-5-portable-comparison"&&!row.task_portable)throw new Error("Fig 5 may consume only portable task rows");if(figure==="fig-3-simulator-comparison"&&row.backend==="tensor"&&row.vehicle!=="vehicle-a-otter")throw new Error("Fig 3 tensor scope is frozen to Vehicle A planar3");if((figure==="fig-6-showcase-transfer"||figure==="fig-7-showcase-capability")&&(row.task_portable||row.simulator!=="bcod-sim"))throw new Error(`${figure} may consume only bcod-sim showcase rows`);}
if(import.meta.url===`file://${process.argv[1]}`){validateFigureQueries();console.log(`Validated ${Object.keys(FIGURE_QUERIES).length} planned figure/table queries against schema.`);}
