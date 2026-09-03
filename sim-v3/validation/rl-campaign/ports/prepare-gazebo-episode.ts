import {mkdirSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {bcodUsvCoefficients} from "../../../core/vehicles/coefficients.js";
import {renderModelSdf,renderModelConfig,renderWorldSdf} from "../../../gazebo/generateGazeboParity.js";
import {prepareEpisode} from "./episode-driver.ts";

export function prepareGazeboEpisode(seed:number,out:string,steps=2400){
 const episode=prepareEpisode("Gazebo Harmonic",seed,steps), root=resolve(out),model=resolve(root,"models",bcodUsvCoefficients.id);mkdirSync(model,{recursive:true});mkdirSync(resolve(root,"worlds"),{recursive:true});
 writeFileSync(resolve(model,"model.config"),renderModelConfig(bcodUsvCoefficients));writeFileSync(resolve(model,"model.sdf"),renderModelSdf(bcodUsvCoefficients,{perThrusterActuation:true,trueOdometry:true,odomHz:20,phaseASensors:true}));
 const maneuver={name:`gate-${seed}`,dt:.05,steps,env:{waterV:{x:0,y:0,z:0}}};
 const world=resolve(root,"worlds",`gate-${seed}.sdf`);writeFileSync(world,renderWorldSdf(bcodUsvCoefficients,maneuver,{initialStateNed:{N:episode.reset.initial_state[0],E:episode.reset.initial_state[1],yaw:episode.reset.initial_state[2]}}));
 const fixedCommandSchedule=episode.transport.map((commands,step)=>({t:step*.05,commands:commands.map(c=>({topic:c.topic,type:"Scalar",value:c.value,messageType:c.message_type}))}));
 const manifest={vehicle:bcodUsvCoefficients.id,maneuver:`gate-${seed}`,dt:.05,steps,gazebo:{actuationMode:"perThruster",commandTopics:fixedCommandSchedule[0].commands.map((x:any)=>x.topic)},world:`worlds/gate-${seed}.sdf`,expectedGoldenCsv:`traces/gate-${seed}.csv`,fixedCommandSchedule,reset:episode.reset,inertia_provenance:episode.inertia_provenance};
 const manifestPath=resolve(root,"manifests",`gate-${seed}.json`);mkdirSync(resolve(root,"manifests"),{recursive:true});writeFileSync(manifestPath,JSON.stringify(manifest,null,2)+"\n");return{root,world,manifestPath};
}
if(process.argv[1]?.endsWith("prepare-gazebo-episode.ts")){const [seed,out]=process.argv.slice(2);if(!seed||!out)throw new Error("usage: prepare-gazebo-episode.ts seed output-dir");console.log(JSON.stringify(prepareGazeboEpisode(Number(seed),out)));}
