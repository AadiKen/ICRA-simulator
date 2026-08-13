import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {validateDamping} from "../../packages/core/src/hydrodynamics.ts";
import {assertDissipative,validateCombinedDamping} from "./run.ts";

for(const vehicle of ["vehicle-b","vehicle-c","vehicle-b-parametric","vehicle-c-parametric"]){
  const path=`artifacts/capytaine/${vehicle}-resolved.json`,first=validateCombinedDamping(path),second=validateCombinedDamping(path);
  assert.deepEqual(first,second);
  assert.equal(first.status,"software-gate-passed-physical-validation-blocked");
  assert.equal(first.is_software_validation_evidence,true);
  assert.equal(first.is_physical_validation_evidence,false);
  for(const mode of ["heave","roll","pitch"] as const){
    const result=first.modes[mode];
    assert.ok(result.combined.logarithmic_decrement!>result.radiation_only_diagnostic.logarithmic_decrement!);
    assert.ok(result.viscous_effect.logarithmic_decrement_increase>0);
    assert.ok(result.dissipation_sample.minimum_total_power>=-1e-9);
    assert.equal(result.combined.simulation.steps,15000);
  }
  assert.equal(first.damping_provenance.linearViscous.fit_data,"none");
  assert.match(first.damping_provenance.linearViscous.checksum!,/^[a-f0-9]{64}$/);
  assert.match(first.damping_provenance.quadraticViscous.limitations!.join(" "),/not a full Ikeda|No dual-hull/);
}

const resolved=JSON.parse(readFileSync("artifacts/capytaine/vehicle-b-parametric-resolved.json","utf8")),model=structuredClone(resolved.free_decay.roll.hydrodynamics.damping);
model.linearViscousDamping=model.linearViscousDamping.map((row:number[])=>row.map(()=>0));
model.quadraticViscousDamping=model.quadraticViscousDamping.map((row:number[])=>row.map(()=>0));
assert.throws(()=>validateDamping(model),/non-zero viscous/);
const active=resolved.free_decay.roll.hydrodynamics.damping;
active.linearViscousDamping[0][1]=-1e9;
assert.throws(()=>assertDissipative(active),/not dissipative/);
console.log("Combined-damping validation tests passed.");
