import assert from "node:assert/strict";
import {allocateSurveyorWrenchYawFirst,FrozenActuatorBank,resolveSurveyorActuatorSpec,resolveVehicleAActuatorSpec} from "./shared-actuators.ts";
const spec=resolveVehicleAActuatorSpec();
assert.equal(spec.effectors.length,2);assert.equal(spec.effectors[0].tau_s,.35);assert.equal(spec.effectors[1].tau_s,.35);
const bank=new FrozenActuatorBank(spec),action=bank.step([1,-1,0,0],.05),expected=1-Math.exp(-.05/.35);
assert.ok(Math.abs(action[0]-expected)<1e-12);assert.ok(Math.abs(action[1]+expected)<1e-12);assert.deepEqual(bank.thrustNewtons(),[expected*spec.effectors[0].max,-expected*Math.abs(spec.effectors[1].min)]);
assert.throws(()=>bank.step([0,0,.1,0],.05),/unused actuator/);console.log("Shared Vehicle A actuator specification and lag passed.");
const surveyor=resolveSurveyorActuatorSpec();assert.equal(surveyor.effectors[0].tau_s,.35);assert.deepEqual(surveyor.effectors.map(x=>x.position_body_m),[[-.65,-.33,0],[-.65,.33,0]]);assert.deepEqual(surveyor.effectors.map(x=>[x.min,x.max]),[[-70,70],[-70,70]]);
assert.deepEqual(allocateSurveyorWrenchYawFirst(140,46.2),[-1,1,0,0]);assert.deepEqual(allocateSurveyorWrenchYawFirst(140,0),[1,1,0,0]);assert.deepEqual(allocateSurveyorWrenchYawFirst(140,23.1),[0,1,0,0]);console.log("Shared Surveyor actuator specification and yaw-priority envelope passed.");
