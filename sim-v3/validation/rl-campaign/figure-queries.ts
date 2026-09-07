import {EPISODE_COLUMNS} from "./schema.ts";

export const FIGURE_QUERIES={
  "fig-0-architecture":[],
  "fig-1-vessels":["task_id","task_portable","simulator","vehicle","algorithm","seed","episode_length","return","success","host_class"],
  "fig-2-geography":[],
  "fig-3-sensing":[],
  "fig-4-determinism":[],
  "fig-5-validation":["task_id","task_portable","simulator","vehicle","algorithm","seed","return","success","termination_reason","collision_type","host_class"],
  "table-6-comparison":[],
} as const;

const PORTABLE_EPISODE_FIGURES=new Set(["fig-1-vessels","fig-5-validation"]);

export function validateFigureQueries():void{
  const columns=new Set<string>(EPISODE_COLUMNS);
  for(const [name,required] of Object.entries(FIGURE_QUERIES)){
    for(const column of required)if(!columns.has(column))throw new Error(`${name} requires absent column ${column}`);
    if(required.length>0&&!required.includes("host_class"))throw new Error(`${name} reads episode rows without host_class`);
  }
}

export function assertTaskEligibleForFigure(
  figure:string,
  row:{task_id:string;task_portable:boolean;simulator:string;vehicle?:string;backend?:string;host_class?:string},
  expectedHostClass?:string,
):void{
  if(!(figure in FIGURE_QUERIES))throw new Error(`Unknown finalized figure/table ${figure}`);
  if(PORTABLE_EPISODE_FIGURES.has(figure)&&!row.task_portable)throw new Error(`${figure} may consume only portable task rows`);
  if(PORTABLE_EPISODE_FIGURES.has(figure)&&typeof row.host_class!=="string")throw new Error(`${figure} requires host_class on every episode row`);
  if(expectedHostClass!==undefined&&row.host_class!==expectedHostClass)throw new Error(`${figure} host_class mismatch: expected ${expectedHostClass}, received ${row.host_class??"missing"}`);
}

if(import.meta.url===`file://${process.argv[1]}`){validateFigureQueries();console.log(`Validated ${Object.keys(FIGURE_QUERIES).length} finalized figure/table queries against schema.`);}
