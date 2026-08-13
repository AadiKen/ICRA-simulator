import assert from "node:assert/strict";
import {resolveExperiment,type ExperimentV1} from "../../../packages/experiment-schema/src/index.ts";
import {HeadlessMarineSimulation} from "../../../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "./legacy-production-engine.ts";
import {buildVehicleBProductionConfiguration} from "./vehicle-b-production.ts";

const raw:ExperimentV1={schema_version:1,experiment:{name:"vehicle-b-production",seed:17,timestep_s:.02,duration_s:8},backend:{type:"node"},vehicle:{preset:"vehicle-b-rudder",plant:"coupled6"},mission:{type:"waypoints",waypoints:[{north_m:40,east_m:12}]}};
const config=resolveExperiment(raw),production=buildVehicleBProductionConfiguration();
assert.equal(production.parameters.maneuveringModel.parameterSetId,"vehicle-b-usv-bootstrap");
assert.equal(production.parameters.maneuveringModel.replacesPlanarDamping,true);
assert.equal(production.mmg.provenance.sourceVessel,undefined);
assert.equal(production.mmg.sourceVessel,"Vehicle-B-USV");
assert.notEqual(production.mmg.provenance.parameterSet,"kvlcc2-yy2015-table3");

const sim=new HeadlessMarineSimulation(new LegacyProductionEngine());sim.reset(config);
let result;for(let step=0;step<50;step++)result=sim.step({actuators:{propeller_rps:15,rudder_rad:.15}});
const info=result!.info as any,truth=sim.getGroundTruth() as any;
assert.equal(info.vehicle_path.vehicle_id,"vehicle-b-rudder");
assert.equal(info.vehicle_path.plant,"coupled6");
assert.equal(info.vehicle_path.parameter_set_id,"vehicle-b-usv-bootstrap");
assert.match(info.vehicle_path.claim_limit,/behaviorally unvalidated/);
assert.ok(info.vehicle_diagnostics.applied_command.propeller_rps>0&&info.vehicle_diagnostics.applied_command.propeller_rps<15);
assert.ok(info.vehicle_diagnostics.applied_command.rudder_rad>0&&info.vehicle_diagnostics.applied_command.rudder_rad<.15);
assert.deepEqual(info.vehicle_diagnostics.force_components.total,info.vehicle_diagnostics.force_components.hull.map((value:number,index:number)=>value+info.vehicle_diagnostics.force_components.propeller[index]+info.vehicle_diagnostics.force_components.rudder[index]));
assert.ok(info.vehicle_diagnostics.full_wrench.every(Number.isFinite));
assert.ok([...truth.position_ned_m,...truth.attitude_rad,...truth.velocity_body_mps,...truth.angular_rate_body_rad_s].every(Number.isFinite));
assert.ok(Math.abs(truth.attitude_rad[0])+Math.abs(truth.attitude_rad[1])>0,"Production coupled6 must leave roll and pitch dynamically free.");

const checkpoint=sim.saveCheckpoint(),expected=[];for(let step=0;step<25;step++){const sample=sim.step({actuators:{propeller_rps:18,rudder_rad:-.1}});expected.push({truth:sim.getGroundTruth(),diagnostics:(sample.info as any).vehicle_diagnostics});}sim.loadCheckpoint(JSON.parse(JSON.stringify(checkpoint)));const replay=[];for(let step=0;step<25;step++){const sample=sim.step({actuators:{propeller_rps:18,rudder_rad:-.1}});replay.push({truth:sim.getGroundTruth(),diagnostics:(sample.info as any).vehicle_diagnostics});}assert.deepEqual(replay,expected,"Vehicle B MMG checkpoint replay must be bit-exact mid-transient");sim.dispose();

const bad=resolveExperiment({...raw,vehicle:{preset:"vehicle-b-rudder",plant:"planar3"}});assert.throws(()=>new HeadlessMarineSimulation(new LegacyProductionEngine()).reset(bad),/requires coupled6/);
console.log("Vehicle B production coupled6 integration tests passed.");
