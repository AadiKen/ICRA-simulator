/** Persistent JSON-lines facade over the one authoritative actuator model. */
import {createInterface} from "node:readline";
import {FrozenActuatorBank,resolveSurveyorActuatorSpec} from "./shared-actuators.ts";

let bank=new FrozenActuatorBank(resolveSurveyorActuatorSpec());
const reply=(value:unknown)=>process.stdout.write(`${JSON.stringify(value)}\n`);
const input=createInterface({input:process.stdin,crlfDelay:Infinity});
input.on("line",line=>{
 try{
  const request=JSON.parse(line);
  if(request.op==="reset"){
   bank=new FrozenActuatorBank(resolveSurveyorActuatorSpec());
   reply({ok:true,thrust_newtons:bank.thrustNewtons()});
  }else if(request.op==="step"){
   if(!Array.isArray(request.action)||request.action.length!==4)throw new Error("action must contain four contract fields");
   bank.step(request.action,Number(request.dt_s));
   reply({ok:true,thrust_newtons:bank.thrustNewtons()});
  }else if(request.op==="close"){
   reply({ok:true});input.close();
  }else throw new Error(`unknown operation ${request.op}`);
 }catch(error){reply({ok:false,error:String(error)});}
});
