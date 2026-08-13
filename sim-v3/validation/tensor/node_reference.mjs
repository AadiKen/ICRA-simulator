import {DynamicsCore} from "../../core/dynamicsCore.js";
import {RigidBodyState} from "../../core/rigidBodyState.js";
import {createOtterParameters} from "../../core/vehicles/otter.js";
import {ActuationModel} from "../../packages/core/src/actuators.js";
import {AddedMassCoriolis,HydrodynamicDamping} from "../../packages/core/src/force-components.js";
import {HydrostaticsAndWaves} from "../../packages/core/src/wave-forces.js";

const steps=Number(process.argv[2]??200),dt=Number(process.argv[3]??0.02);
if(!Number.isInteger(steps)||steps<1||!Number.isFinite(dt)||dt<=0)throw new Error("Usage: node_reference.mjs STEPS DT");

function commandAt(step,total){
  const phase=step/total;
  if(phase<.05)return[0,0];
  if(phase<.20){const ramp=(phase-.05)/.15;return[ramp,ramp];}
  if(phase<.40)return[.65,.65];
  if(phase<.60)return[.8,-.35];
  if(phase<.75)return[-.55,-.55];
  if(phase<.90)return[1,-1];
  return[-.4,.9];
}

const params=createOtterParameters(),actuator=new ActuationModel(params),core=new DynamicsCore(params,[actuator,new AddedMassCoriolis(),new HydrodynamicDamping(),new HydrostaticsAndWaves()],"rk4"),state=RigidBodyState.fromYaw({N:0,E:0,D:0},0),env={waterV:{x:0,y:0,z:0},hullWaterSamples:[]},trace=[];
for(let step=0;step<steps;step+=1){
  const [portCommand,starboardCommand]=commandAt(step,steps),command={portCommand,starboardCommand};
  command.appliedWrench=actuator.commandWrench(command,dt);
  core.step(state,env,command,dt,step*dt);
  const metrics=actuator.getEnergyMetrics();
  trace.push({command:[portCommand,starboardCommand],state:[state.position.N,state.position.E,state.eulerAngles.yaw,state.velocity.u,state.velocity.v,state.angularRate.r],acceleration:[state.acceleration.uDot,state.acceleration.vDot,state.angularAccel.rDot],thruster_state:actuator.effectors.map((item)=>item.thrust),wrench:[command.appliedWrench[0],command.appliedWrench[1],command.appliedWrench[5]],energy_j:metrics.actuator_energy_j,power_w:metrics.actuator_power_w,step:step+1});
}
console.log(JSON.stringify({vehicle:"vehicle-a-otter",plant:"planar3",dt,trace}));
