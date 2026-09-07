import assert from "node:assert/strict";
import test from "node:test";
import {VEHICLE_C_CORIOLIS_COMPENSATION_SCALE,VehicleCCoriolisFeedforward} from "./vehicle-c-coriolis-feedforward.ts";

test("Vehicle C feedforward adds only production Coriolis surge and yaw before allocation",()=>{
  const feedforward=new VehicleCCoriolisFeedforward(),base:[number,number,number,number,number,number]=[10,0,0,0,0,-4],state={attitude_rad:[.1,-.05,.7] as [number,number,number],velocity_body_mps:[1.2,.4,0] as [number,number,number],angular_rate_body_rad_s:[0,0,.2] as [number,number,number]},result=feedforward.apply(base,state,[.3,-.1,0]);
  assert.equal(VEHICLE_C_CORIOLIS_COMPENSATION_SCALE,.5);
  assert.equal(result.desired_wrench[0],base[0]+result.compensation_full[0]);
  assert.equal(result.desired_wrench[5],base[5]+result.compensation_full[5]);
  assert.equal(result.compensation_full[0],.5*result.raw_coriolis_full[0]);
  assert.deepEqual(result.desired_wrench.slice(1,5),base.slice(1,5));
  assert.equal(result.desired_wrench[1],0,"computed sway must be reported, not commanded");
});

test("Vehicle C feedforward permits an explicit full-scale diagnostic without changing the retained experimental default",()=>{
  const feedforward=new VehicleCCoriolisFeedforward(1),base:[number,number,number,number,number,number]=[10,0,0,0,0,-4],state={attitude_rad:[.1,-.05,.7] as [number,number,number],velocity_body_mps:[1.2,.4,0] as [number,number,number],angular_rate_body_rad_s:[0,0,.2] as [number,number,number]},result=feedforward.apply(base,state,[.3,-.1,0]);
  assert.equal(result.compensation_full[0],result.raw_coriolis_full[0]);
  assert.equal(result.compensation_full[5],result.raw_coriolis_full[5]);
});
