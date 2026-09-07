import assert from "node:assert/strict";
import {allocatePlanarAzimuthMinimumNorm} from "../src/azimuth-allocation.js";
import {ActuationModel} from "../src/actuators.js";
import {buildVehicleCProductionConfiguration} from "../../../backends/node/src/vehicle-c-production.ts";

const placements=[{id:"azimuth-port",pos:[-1.5,-.81,-.18],maxThrust:500},{id:"azimuth-starboard",pos:[-1.5,.81,-.18],maxThrust:500}];
const close=(actual:number,expected:number,label:string)=>assert.ok(Math.abs(actual-expected)<1e-9,`${label}: ${actual} != ${expected}`);
const check=(wrench:[number,number,number])=>{const result=allocatePlanarAzimuthMinimumNorm(wrench,placements);result.achieved_wrench.forEach((value,index)=>close(value,wrench[index],`wrench[${index}]`));return result};

const forward=check([200,0,0]);close(forward.pods[0].thrust,100,"forward port");close(forward.pods[1].thrust,100,"forward starboard");forward.pods.forEach(pod=>close(pod.azimuth,0,"forward azimuth"));

const rotation=check([0,0,81]);close(rotation.pods[0].thrust,50,"rotation port");close(rotation.pods[1].thrust,50,"rotation starboard");close(rotation.pods[0].azimuth,0,"rotation port azimuth");close(Math.abs(rotation.pods[1].azimuth),Math.PI,"rotation starboard azimuth");

const continuousRotation=allocatePlanarAzimuthMinimumNorm([0,0,81],placements.map(x=>({...x,maxReverseThrust:250})),{angleContinuityWeight:1,currentAzimuths:[0,0]});
close(continuousRotation.pods[1].azimuth,0,"continuity avoids starboard half-turn");close(continuousRotation.pods[1].thrust,-50,"continuity uses equivalent reverse thrust");continuousRotation.achieved_wrench.forEach((value,index)=>close(value,[0,0,81][index],`continuous wrench[${index}]`));

const sway=check([0,100,0]);
const expectedDifferential=1.5*100/.81;
close(sway.pods[0].force_body_n[0]-sway.pods[1].force_body_n[0],expectedDifferential,"pure-sway moment-cancelling surge couple");
close(sway.pods[0].force_body_n[1]+sway.pods[1].force_body_n[1],100,"pure-sway lateral force");
assert.ok(sway.pods.every(pod=>Math.abs(Math.abs(pod.azimuth)-Math.PI/2)>1e-3),"aft offset must tilt the pods away from naive ±90 degrees");

const production:any=new ActuationModel(buildVehicleCProductionConfiguration().parameters);
const trackingWrench=[100,0,0,0,0,25.129457569952145],alpha=1-Math.exp(-.05/.35);
production.commandWrench({desiredWrench:trackingWrench},.05);
assert.equal(production.lastAllocationDiagnostics.strategy,"minimum-thrust-norm");
assert.deepEqual(Object.keys(production.lastEffectorCommands).sort(),["azimuth-port","azimuth-starboard"]);
const targets=[production.lastEffectorCommands["azimuth-port"].thrust,production.lastEffectorCommands["azimuth-starboard"].thrust];
close(targets[0],65.51201084564947,"tracking port target");close(targets[1],34.48798915435054,"tracking starboard target");
close(production.effectors[0].thrust,targets[0]*alpha,"port first-order lag step");close(production.effectors[1].thrust,targets[1]*alpha,"starboard first-order lag step");
assert.ok(production.effectors[0].thrust>1&&production.effectors[1].thrust>1,"thrust must not be capped by the 0.5236 rad/s steering rate");
for(let step=1;step<40;step++)production.commandWrench({desiredWrench:trackingWrench},.05);
assert.ok(Math.abs(production.effectors[0].thrust-targets[0])<.25&&Math.abs(production.effectors[1].thrust-targets[1])<.25,"requested split must settle through the 0.35 s thrust lag");
assert.equal(production.effectors[0].steer.rateMax,30*Math.PI/180);assert.equal(production.effectors[1].steer.rateMax,30*Math.PI/180);

console.log("Vehicle C minimum-norm directional allocation tests passed.");
