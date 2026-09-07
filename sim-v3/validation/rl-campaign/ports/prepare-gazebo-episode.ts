import {mkdirSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {renderModelSdf,renderModelConfig,renderWorldSdf} from "../../../gazebo/generateGazeboParity.js";
import {prepareEpisode} from "./episode-driver.ts";
import {surveyorGazeboCoefficients} from "./surveyor-gazebo-coefficients.ts";

export function prepareGazeboEpisode(seed:number,out:string,steps=2400,phaseASensors=true){
 const episode=prepareEpisode("Gazebo Harmonic",seed,steps), root=resolve(out),model=resolve(root,"models",surveyorGazeboCoefficients.id);mkdirSync(model,{recursive:true});mkdirSync(resolve(root,"worlds"),{recursive:true});
 writeFileSync(resolve(model,"model.config"),renderModelConfig(surveyorGazeboCoefficients));writeFileSync(resolve(model,"model.sdf"),renderModelSdf(surveyorGazeboCoefficients,{perThrusterActuation:true,trueOdometry:true,odomHz:100,phaseASensors,contactSensors:true}));
 const maneuver={name:`gate-${seed}`,dt:.005,steps,env:{waterV:{x:0,y:0,z:0}}};
 const initialStateNed=phaseASensors?{N:episode.reset.initial_state[0],E:episode.reset.initial_state[1],yaw:episode.reset.initial_state[2]}:{N:0,E:0,yaw:episode.reset.initial_state[2]};
 const world=resolve(root,"worlds",`gate-${seed}.sdf`);writeFileSync(world,renderWorldSdf(surveyorGazeboCoefficients,maneuver,{initialStateNed,omitSphericalCoordinates:!phaseASensors}));
 const fixedCommandSchedule=episode.transport.map((commands,step)=>({t:step*.05,commands:commands.map(c=>({topic:c.topic,type:"Scalar",value:c.value,messageType:c.message_type}))}));
 const manifest={vehicle:surveyorGazeboCoefficients.id,maneuver:`gate-${seed}`,dt:.05,steps,gazebo:{actuationMode:"perThruster",commandTopics:fixedCommandSchedule[0].commands.map((x:any)=>x.topic)},world:`worlds/gate-${seed}.sdf`,expectedGoldenCsv:`traces/gate-${seed}.csv`,fixedCommandSchedule,reset:episode.reset,inertia_provenance:episode.inertia_provenance};
 const manifestPath=resolve(root,"manifests",`gate-${seed}.json`);mkdirSync(resolve(root,"manifests"),{recursive:true});writeFileSync(manifestPath,JSON.stringify(manifest,null,2)+"\n");return{root,world,manifestPath};
}
if(process.argv[1]?.endsWith("prepare-gazebo-episode.ts")){const [seed,out,mode]=process.argv.slice(2);if(!seed||!out)throw new Error("usage: prepare-gazebo-episode.ts seed output-dir [--idle-no-sensors]");console.log(JSON.stringify(prepareGazeboEpisode(Number(seed),out,2400,mode!=="--idle-no-sensors")));}
