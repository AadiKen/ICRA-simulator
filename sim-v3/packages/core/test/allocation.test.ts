import assert from "node:assert/strict";
import {allocateDualAzimuth} from "../src/allocation.ts";

const healthy=allocateDualAzimuth([100,20,30],[{x_m:-1,y_m:-0.7,max_thrust_n:200},{x_m:-1,y_m:0.7,max_thrust_n:200}]);
assert.equal(healthy.diagnostics.rank,3);
assert.ok(healthy.diagnostics.residual_wrench.every(Number.isFinite));

const throughCg=allocateDualAzimuth([100,20,30],[{x_m:0,y_m:0,max_thrust_n:200},{x_m:0,y_m:0,max_thrust_n:200}]);
assert.equal(throughCg.diagnostics.degradation,"rank-deficient");
assert.ok(throughCg.diagnostics.active_constraints.includes("rank-deficient"));
assert.ok(throughCg.commands.flatMap((v)=>[v.fx_n,v.fy_n]).every(Number.isFinite));
assert.ok(Math.abs(throughCg.diagnostics.residual_wrench[2])>20);

for(const offset of [1e-1,1e-3,1e-6,0]){
  const result=allocateDualAzimuth([50,10,25],[{x_m:offset,y_m:-offset,max_thrust_n:80},{x_m:offset,y_m:offset,max_thrust_n:80}]);
  assert.ok(result.commands.flatMap((v)=>[v.fx_n,v.fy_n]).every(Number.isFinite));
  assert.ok(result.diagnostics.residual_wrench.every(Number.isFinite));
}
const failedOff=allocateDualAzimuth([50,10,25],[{x_m:-1,y_m:-.7,max_thrust_n:80},{x_m:-1,y_m:.7,max_thrust_n:80}],{unavailable:[true,false]});
assert.equal(failedOff.commands[0].fx_n,0);assert.equal(failedOff.diagnostics.infeasible,true);
const stuck=allocateDualAzimuth([30,0,0],[{x_m:-1,y_m:-.7,max_thrust_n:80},{x_m:-1,y_m:.7,max_thrust_n:80}],{stuck:[{fx_n:20,fy_n:10},null]});
assert.deepEqual(stuck.diagnostics.stuck_actuator_bias,[20,10,4]);assert.ok(stuck.diagnostics.residual_wrench.every(Number.isFinite));
assert.ok(stuck.diagnostics.active_constraints.includes("unachievable-wrench"));
console.log("Allocation tests passed.");
