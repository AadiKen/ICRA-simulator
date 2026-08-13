import assert from "node:assert/strict";
import {ActuationModel,allocationMatrix,FixedThruster} from "../src/actuators.js";

const effector:any={id:"port",type:"FixedThruster",pos:[0,-1,0],axis:[1,0,0],maxForwardThrust:100,maxReverseThrust:100,dynamics:{tau:0.001,rateMax:Infinity},conversion:{type:"linear",deadZone:0.2},power:{idleW:5,controlW:10,maxW:100}};
const params:any={geometry:{beam:2},massProps:{cg:{x:0,y:0,z:0}},controlledDOF:["surge","yaw"],actuator:{behaviorVersion:"integrated-v1",maxThrust:100,motorTimeConstant:0.001,beam:2},effectors:[effector,{...effector,id:"starboard",pos:[0,1,0]}]};

for(const [command,expected] of [[0.199999,0],[-0.199999,0],[0.2,20],[-0.2,-20],[0.200001,20.0001],[-0.200001,-20.0001]] as const){const thruster=new FixedThruster(effector,params);assert.ok(Math.abs(thruster.advance(0,{command})-expected)<1e-9,`dead-zone boundary ${command}`);}
const energyThruster:any=new FixedThruster(effector,params);assert.equal(energyThruster.advance(1,{command:0.1}),0);assert.equal(energyThruster.lastPowerW,6);assert.equal(energyThruster.energyJ,6);
assert.equal(energyThruster.propulsionEnergyJ,0,"Sub-dead-zone energy is actuator overhead, not propulsion energy.");

const failed:any=new ActuationModel(params);failed.commandWrench({effectors:{port:{command:1},starboard:{command:1}}},0.1);failed.commandWrench({failureStates:{port:{mode:"failed-off",source:"bench-fixture"}},effectors:{port:{command:1},starboard:{command:1}}},0.1);assert.equal(failed.effectors[0].thrust,0);assert.deepEqual(failed.drainEvents(),[{type:"ACTUATOR_FAILURE",actuator_id:"port",step:1,mode:"failed-off",previous_mode:"healthy",source:"bench-fixture"}]);failed.commandWrench({failureStates:{port:"failed-off"},effectors:{port:{command:1},starboard:{command:1}}},0.1);assert.equal(failed.drainEvents().length,0);
failed.commandWrench({desiredWrench:[50,0,20]},0.1);assert.equal(failed.lastAllocationDiagnostics.mode,"failed-off-reduced-set");assert.equal(failed.lastAllocationDiagnostics.rank,1);assert.equal(failed.lastAllocationDiagnostics.degradation,"rank-deficient");assert.ok(failed.lastAllocationDiagnostics.singular_values.length>0);

const stuck:any=new ActuationModel(params);stuck.commandWrench({effectors:{port:{command:0.5},starboard:{command:0}}},0.1);stuck.commandWrench({failureStates:{port:{mode:"stuck",source:"test"}},effectors:{port:{command:-1},starboard:{command:0}}},0.1);const frozen=stuck.effectors[0].thrust;const checkpoint=JSON.parse(JSON.stringify(stuck.saveState()));const expected=stuck.commandWrench({desiredWrench:[0,0,0]},0.1);const expectedDiagnostics=structuredClone(stuck.lastAllocationDiagnostics);stuck.loadState(checkpoint);assert.deepEqual(stuck.commandWrench({desiredWrench:[0,0,0]},0.1),expected);assert.deepEqual(stuck.lastAllocationDiagnostics,expectedDiagnostics);assert.equal(stuck.effectors[0].thrust,frozen);assert.equal(stuck.lastAllocationDiagnostics.mode,"stuck-bias-rejection");assert.equal(stuck.lastAllocationDiagnostics.reachable,false);assert.ok(stuck.lastAllocationDiagnostics.residual_wrench.every(Number.isFinite));

const direct:any=new ActuationModel(params);
for(const [label,command] of [
    ["tauDes",{tauDes:[1,Number.NaN,0]}],
    ["desiredWrench",{desiredWrench:[1,0,0,0,0,Number.POSITIVE_INFINITY]}],
    ["surgeForce",{surgeForce:Number.NaN}],
    ["differentialForce",{differentialForce:Number.NEGATIVE_INFINITY}],
    ["yawMoment",{yawMoment:Number.NaN}],
    ["appliedWrench",{appliedWrench:[1,0,Number.NaN]}],
    ["effector command",{effectors:{port:{command:Number.NaN},starboard:{command:0}}}],
    ["explicit keyed command",{portCommand:Number.NaN}]
] as const){assert.throws(()=>direct.commandWrench(command as any,0.1),/must be finite/,`${label} must reject non-finite input`);}
assert.throws(()=>direct.allocate([1,0,0,0,0,Number.NaN]),/must be finite/,"allocation target must reject non-finite input");
assert.throws(()=>allocationMatrix([{id:"bad",wrenchWithUnitCommand:()=>[1,0,0,0,0,Number.NaN]}] as any,params),/must be finite/,"effector allocation columns must reject non-finite output");
assert.throws(()=>new ActuationModel({...params,effectors:undefined,actuator:{...params.actuator,maxThrust:Number.NaN}}),/must be finite/,"maxThrust config must reject non-finite input");
assert.throws(()=>new ActuationModel({...params,effectors:undefined,actuator:{...params.actuator,beam:Number.NaN}}),/must be finite/,"beam config must reject non-finite input");
assert.throws(()=>new ActuationModel({...params,effectors:undefined,actuator:{...params.actuator,motorTimeConstant:Number.NaN}}),/must be finite/,"time-constant config must reject non-finite input");
assert.throws(()=>new ActuationModel({...params,effectors:[{...effector,axis:[1,Number.NaN,0]}]}),/must be finite/,"effector direction config must reject non-finite input");
assert.throws(()=>direct.prepareStep({command:{},dt:Number.NaN}),/must be finite/,"prepareStep must not convert a non-finite dt to zero");

const zeroDefaults:any=new ActuationModel({...params,effectors:undefined,actuator:{...params.actuator,maxThrust:0}});
assert.deepEqual(zeroDefaults.commandWrench({},0.1),[0,0,0],"omitted demand and explicit zero max thrust remain valid");
assert.doesNotThrow(()=>direct.commandWrench({desiredWrench:[0,0,0]},0.1),"legacy planar wrench remains supported");
console.log("Integrated actuator capability tests passed.");
