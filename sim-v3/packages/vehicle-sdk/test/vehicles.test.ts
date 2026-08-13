import assert from "node:assert/strict";
import {VEHICLES,validateVehicle} from "../src/index.ts";
assert.equal(Object.keys(VEHICLES).length,4);
for(const vehicle of Object.values(VEHICLES))validateVehicle(vehicle);
assert.equal(VEHICLES["vehicle-b-rudder"].validation.status,"model-structure-trajectory-scored-fail");
assert.deepEqual(VEHICLES["vehicle-b-rudder"].maneuvering_model,{type:"mmg",parameter_set_id:"vehicle-b-usv-bootstrap",stage:"usv-scale",selectable:true,production_path:"integrated-coupled6"});
assert.deepEqual(VEHICLES["vehicle-b-rudder"].validation.evidence_tiers?.map(({id,status})=>({id,status})),[{id:"kvlcc2-mmg-reference-reproduction",status:"blocked-missing-primary-source"},{id:"kvlcc2-experimental-maneuver-indices",status:"executed-comparison-no-acceptance-gate"},{id:"kvlcc2-marin-trajectory-comparison",status:"experimental-trajectory-scored-fail"},{id:"wpcc-trajectory-comparison",status:"architecture-variant-reference"}]);
assert.match(VEHICLES["vehicle-b-rudder"].validation.limitations.join(" "),/7\.00 m.*1\/45\.7/);
assert.match(VEHICLES["vehicle-c-azimuth"].validation.claim,/behavioral dynamics not independently validated/);
for(const id of ["vehicle-b-rudder","vehicle-c-azimuth"]){const potential=VEHICLES[id].potential_flow;assert.equal(potential?.provenance.kind,"capytaine");assert.equal(potential?.provenance.mesh_status,"analytic-parametric");assert.equal(potential?.status,"analytic-parametric-solver-output");assert.deepEqual(potential?.supplies,["added-mass","radiation-damping","wave-excitation","hydrostatic-stiffness"]);assert.match(potential!.provenance.limitations.join(" "),/Representative design geometry is not physical validation/);assert.equal(VEHICLES[id].geometry.length.provenance.kind,"design-assumption");assert.ok(VEHICLES[id].parametric_hull);}
assert.equal(VEHICLES["vehicle-b-rudder"].validation.status,"model-structure-trajectory-scored-fail");
assert.equal(VEHICLES["vehicle-c-azimuth"].validation.status,"allocation-demonstrated-dynamics-unvalidated");
assert.equal(VEHICLES["vehicle-c-azimuth"].hydrostatics?.gm_transverse.value,3.7232);
assert.equal(VEHICLES["searobotics-surveyor-m1.8"].effectors.length,2);
assert.match(VEHICLES["searobotics-surveyor-m1.8"].validation.limitations.join(" "),/Newton-per-command mapping is unknown/);
const invalid=structuredClone(VEHICLES["vehicle-c-azimuth"]);invalid.damping.linear_viscous.value.fill(0);invalid.damping.quadratic_viscous.value.fill(0);
assert.throws(()=>validateVehicle(invalid),/non-zero viscous/);
console.log("Vehicle SDK tests passed.");
