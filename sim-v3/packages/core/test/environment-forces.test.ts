import assert from "node:assert/strict";
import {currentNedFromEnv,WindLoad} from "../src/environment-forces.js";
import {WindLoad as CompatibilityWindLoad} from "../../../core/forces/windLoad.js";
assert.equal(CompatibilityWindLoad,WindLoad);
assert.deepEqual(currentNedFromEnv({waterV:{x:2,y:3,z:4}}),{N:4,E:2,D:-3});
const wind=new WindLoad({enabled:true,rho:1,frontalArea:2,lateralArea:3,length:4,C_X:1,C_Y:0.5,C_N:0.25});
assert.deepEqual(wind.computeWrench({params:{geometry:{}},state:{velocity:{u:0,v:0},eulerAngles:{yaw:0}},env:{wind:{N:2,E:0}}}),[4,3,6]);
console.log("Environment force migration tests passed.");
