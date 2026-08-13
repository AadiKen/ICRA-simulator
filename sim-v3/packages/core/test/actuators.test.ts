import assert from "node:assert/strict";
import {ActuationModel,FixedThruster} from "../src/actuators.js";
import {ActuationModel as CompatibilityActuationModel} from "../../../core/forces/actuatorModel.js";

const params:any={geometry:{beam:2},massProps:{cg:{x:0,y:0,z:0}},actuator:{maxThrust:100,motorTimeConstant:0.5,beam:2}};
assert.equal(CompatibilityActuationModel,ActuationModel,"Legacy facade must resolve to the typed-core actuator implementation.");
const thruster=new FixedThruster({id:"typed",type:"FixedThruster",pos:[0,0.5,0],axis:[1,0,0],maxForwardThrust:100,maxReverseThrust:60,dynamics:{tau:0.5,rateMax:80},conversion:{type:"linear",deadZone:0.2}},params);
assert.equal(thruster.advance(0.1,{command:1}),8);
assert.equal(thruster.wrench(params)[5],-4);
assert.equal(thruster.advance(0,{command:0.1}),10,"Parity migration must preserve the legacy ignored dead-zone field.");
const model=new ActuationModel(params);model.commandWrench({portCommand:1,starboardCommand:-1},0.1);const saved=structuredClone(model.saveState());const expected=model.commandWrench({portCommand:-1,starboardCommand:1},0.1);model.loadState(saved);assert.deepEqual(model.commandWrench({portCommand:-1,starboardCommand:1},0.1),expected);
console.log("Typed-core actuator migration tests passed.");
