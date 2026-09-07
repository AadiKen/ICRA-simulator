/** JSON-lines access to the existing Gazebo ENU -> task NED conversion. */
import {createInterface} from "node:readline";
import {gazeboOdomToTask} from "./task-trace-bridge.ts";

const reply=(value:unknown)=>process.stdout.write(`${JSON.stringify(value)}\n`);
const input=createInterface({input:process.stdin,crlfDelay:Infinity});
input.on("line",line=>{
  try {
    const request=JSON.parse(line);
    if(request.op==="gazebo_odom_to_task") {
      reply({ok:true,result:gazeboOdomToTask(Number(request.time_s),request.enu)});
    } else if(request.op==="close") {
      reply({ok:true}); input.close();
    } else throw new Error(`unknown operation ${request.op}`);
  } catch(error) { reply({ok:false,error:String(error)}); }
});
