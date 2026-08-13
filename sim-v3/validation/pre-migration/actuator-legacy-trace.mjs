import {ActuationModel,FixedThruster} from "../../core/forces/actuatorModel.js";
import {VehicleParameters} from "../../core/vehicleParameters.js";

const parameters=VehicleParameters.fromGeometry(2,2,0.2,20,{maxThrust:100,motorTimeConstant:0.5});
const config={id:"legacy",type:"FixedThruster",pos:[0,0.5,0],axis:[1,0,0],maxForwardThrust:100,maxReverseThrust:60,dynamics:{tau:0.5,rateMax:80},conversion:{type:"linear",deadZone:0.2}};
const clean=(value)=>JSON.parse(JSON.stringify(value));

function traceThruster(name,commands){
  const thruster=new FixedThruster(config,parameters);
  const samples=[];
  for(let step=0;step<commands.length;step+=1){
    const entry=commands[step];
    const thrust=thruster.advance(entry.dt,{command:entry.command,...(entry.metadata??{})});
    const wrench=thruster.wrench(parameters);
    samples.push({step,input_command:entry.command,stored_command:thruster.command,thrust_n:thrust,wrench_surge_n:wrench[0],wrench_yaw_nm:wrench[5]});
  }
  const selected=samples.filter((sample)=>[0,1,3,5,7,11].includes(sample.step)||commands.length<12);
  return{name,samples:selected};
}

const thresholdCommands=[-1.01,-1,-0.20000000000000004,-0.2,-0.19999999999999998,0,0.19999999999999998,0.2,0.20000000000000004,1,1.01];
const threshold=new FixedThruster(config,parameters);
const thresholdSamples=thresholdCommands.map((command,index)=>{threshold.reset();const thrust=threshold.advance(0,{command});return{index,input_command:command,stored_command:threshold.command,thrust_n:thrust};});

const asymmetric=new ActuationModel(parameters);
const asymmetricSamples=[
  {portCommand:1,starboardCommand:0},
  {portCommand:-1,starboardCommand:0.5},
  {portCommand:1.2,starboardCommand:-1.2}
].map((command,step)=>{const wrench=asymmetric.commandWrench(command,0.1);const state=asymmetric.saveState();return{step,command,wrench_planar_n_nm:wrench,full_wrench_body_n_nm:clean(asymmetric.lastFullWrench),resolved_effector_commands:clean(asymmetric.lastEffectorCommands),effector_state:state.effectors.map(({id,command:stored_command,thrust})=>({id,stored_command,thrust_n:thrust}))};});

const saturation=new FixedThruster(config,parameters);
const saturationCommands=[0,1,1.5,1.5,1.5,1,0.5,-1,-1.5,-1.5,-1,0];
const saturationSamples=saturationCommands.map((command,step)=>({step,input_command:command,thrust_n:saturation.advance(0.1,{command})}));

const failure=new FixedThruster(config,parameters);
const failureSamples=[];
for(let step=0;step<8;step+=1){
  const failure_mode=step<4?"healthy":"failed-off";
  const thrust=failure.advance(0.1,{command:1,failure_mode});
  failureSamples.push({step,failure_mode,input_command:1,thrust_n:thrust});
}

console.log(JSON.stringify({
  schema_version:1,
  artifact_kind:"legacy-actuator-parity-golden",
  source_behavior:"core/forces/actuatorModel.js before actuator migration",
  configuration:config,
  cases:[
    traceThruster("forward-step",Array.from({length:12},()=>({dt:0.1,command:1}))),
    traceThruster("reverse-step",Array.from({length:12},()=>({dt:0.1,command:-1}))),
    traceThruster("dead-zone-ramp",[-0.3,-0.2,-0.1,0,0.1,0.2,0.3].map((command)=>({dt:0,command})))
  ],
  threshold_samples:thresholdSamples,
  saturation_entry_hold_exit:{input_commands:saturationSamples.map((sample)=>sample.input_command),thrust_n:saturationSamples.map((sample)=>sample.thrust_n)},
  asymmetric_port_starboard:asymmetricSamples,
  failure_mid_run:{failure_modes:failureSamples.map((sample)=>sample.failure_mode),thrust_n:failureSamples.map((sample)=>sample.thrust_n)},
  legacy_capability_findings:{
    configured_dead_zone:"ignored; legacy conversion is continuous linear through zero",
    failure_mode_command:"ignored; failed-off metadata does not alter thrust",
    migration_rule:"preserve these results for parity; implement dead-zone and failure semantics only in a separate post-parity change"
  }
}));
