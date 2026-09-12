import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import {renderSurveyorVrxModel} from "./prepare-vrx-surveyor.ts";

const source=JSON.parse(readFileSync("artifacts/rl-campaign/surveyor-vehicle-model.json","utf8"));
const estimate=source.hydrodynamics.linear_out_of_plane_vrx;
assert.equal(estimate.classification.startsWith("reviewed engineering estimate"),true);
assert.match(estimate.method,/zeta=0\.25/);
assert.match(estimate.derivation.pitch,/75% of the continuous-waterplane stiffness/);
const model=renderSurveyorVrxModel();
assert.match(model,/<zW>365<\/zW><kP>40<\/kP><mQ>85<\/mQ>/);
assert.match(model,/<xDotU>0<\/xDotU><yDotV>0<\/yDotV><nDotR>0<\/nDotR>/);
console.log("Reviewed VRX out-of-plane damping provenance and serialization passed.");
