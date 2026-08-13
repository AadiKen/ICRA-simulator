export type Axis="maneuvering"|"environment"|"sensing"|"resources"|"geography"|"mission-integration";
export interface BenchmarkScenario {id:string;axis:Axis;tags:string[];seed:number;severity:{episode:number;environment:number;sensing_resources:number;geography:number;competency:number;score:number;tier:"simple"|"moderate"|"complex"}}
const definitions:Record<Axis,string[]>={
  maneuvering:["straight-acceleration","coast-down","turning-circle","zig-zag","reverse","crash-stop"],
  environment:["beam-current","following-current","crosswind","gust-front","regular-head-wave","combined-wind-current-wave"],
  sensing:["clear","dense-fog","heavy-rain","low-light","gps-denial","sensor-failure"],
  resources:["limited-battery","high-sensor-power","switching-penalty","bandwidth-budget","compute-budget","mixed-energy-tradeoff"],
  geography:["open-coastal","structured-harbor","narrow-channel","shallow-hazard","bridge-transit","restricted-region"],
  "mission-integration":["waypoint-navigation","obstacle-avoidance","station-keeping","channel-traversal","changing-weather","grounded-depth-hazard"]
};
const axes=Object.keys(definitions) as Axis[];
export const scenarios:BenchmarkScenario[]=axes.flatMap((axis,axisIndex)=>definitions[axis].map((name,index)=>{const values=[index+1,axisIndex%3+1,(index+axisIndex)%3+1,axis==="geography"?index%3+2:1,Math.min(5,index+1)];const score=values.reduce((a,b)=>a+b,0)/25;return{id:`${axis.slice(0,3)}-${String(index+1).padStart(2,"0")}-${name}`,axis,tags:[axis,name],seed:3600+axisIndex*10+index,severity:{episode:values[0],environment:values[1],sensing_resources:values[2],geography:values[3],competency:values[4],score,tier:score<0.4?"simple":score<0.65?"moderate":"complex"}};}));
export const manifest={name:"USV-Bench-36",version:"1.0.0",base_scenario_count:36,vehicles:["vehicle-a-otter","vehicle-b-rudder","vehicle-c-azimuth"],configuration_count:108,scenarios};

