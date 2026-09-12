import {performance} from "node:perf_hooks";
import {DynamicsCore} from "../../core/dynamicsCore.js";
import {RigidBodyState} from "../../core/rigidBodyState.js";
import {createOtterParameters} from "../../core/vehicles/otter.js";
import {ActuationModel} from "../../packages/core/src/actuators.js";
import {AddedMassCoriolis,HydrodynamicDamping} from "../../packages/core/src/force-components.js";
import {HydrostaticsAndWaves} from "../../packages/core/src/wave-forces.js";

const sampleSteps=Number(process.argv[2]??16),warmupSteps=Number(process.argv[3]??4),dt=.02;
if(!Number.isInteger(sampleSteps)||sampleSteps<2||!Number.isInteger(warmupSteps)||warmupSteps<0)throw new Error("Usage: scalar_probe.mjs SAMPLE_STEPS WARMUP_STEPS");
const params=createOtterParameters(),actuator=new ActuationModel(params),core=new DynamicsCore(params,[actuator,new AddedMassCoriolis(),new HydrodynamicDamping(),new HydrostaticsAndWaves()],"rk4"),state=RigidBodyState.fromYaw({N:0,E:0,D:0},0),environment={waterV:{x:0,y:0,z:0},hullWaterSamples:[]};
function step(index){const command={portCommand:.4,starboardCommand:.3};command.appliedWrench=actuator.commandWrench(command,dt);core.step(state,environment,command,dt,index*dt);}
for(let i=0;i<warmupSteps;i++)step(i);
const started=performance.now();for(let i=0;i<sampleSteps;i++)step(warmupSteps+i);const elapsedMs=performance.now()-started;
console.log(JSON.stringify({sample_steps:sampleSteps,warmup_steps:warmupSteps,measured_wall_clock_s:elapsedMs/1000,steps_per_second:sampleSteps/(elapsedMs/1000),ms_per_step:elapsedMs/sampleSteps}));
