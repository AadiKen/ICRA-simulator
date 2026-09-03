import {readFileSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {prepareVrxEpisode} from "./prepare-vrx-episode.ts";
import {VrxWamvThrusterAdapter} from "./simulator-actuator-adapters.ts";

const [out="/tmp/vrx-yaw-probe",signArg="1"]=process.argv.slice(2),steps=400,sign=Number(signArg);
prepareVrxEpisode(20000,out,steps,0,1);
const adapter=new VrxWamvThrusterAdapter();
const command=[-0.5*sign,0.5*sign,0,0] as [number,number,number,number];
const transport=Array.from({length:steps},()=>adapter.apply(command,.05));
const path=resolve(out,"transport.json"),schedule=JSON.parse(readFileSync(path,"utf8"));
schedule.action_scale="pure-differential-0.5";schedule.physics_samples=steps;schedule.control_actions=steps/2;schedule.transport=transport;
writeFileSync(path,JSON.stringify(schedule,null,2)+"\n");
