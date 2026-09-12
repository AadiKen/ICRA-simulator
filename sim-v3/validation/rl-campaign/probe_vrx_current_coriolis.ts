import assert from "node:assert/strict";
import { mkdirSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import {
  addedMassCoriolis3,
  rigidBodyCoriolis3,
} from "../../packages/core/src/coriolis.ts";

const mass = 52.3;
const xuDot = -2.615;
const yvDot = -39.225;
const params = {
  massProps: { mass, cg: { x: 0, y: 0, z: 0 } },
  addedMass: { XuDot: xuDot, YvDot: yvDot, NrDot: -1.431297586666667 },
};
const nu = [1.5, 0.3, 0.2];
const current = [1.2, -0.4, 0];
const nuRelative = nu.map((value, index) => value - current[index]);
const multiply = (matrix: number[][], vector: number[]) =>
  matrix.map((row) =>
    row.reduce((sum, value, index) => sum + value * vector[index], 0),
  );
const negative = (value: number[]) => value.map((x) => -x);
const add = (...vectors: number[][]) =>
  vectors[0].map((_, index) =>
    vectors.reduce((sum, vector) => sum + vector[index], 0),
  );

const nodeRigid = negative(
  multiply(rigidBodyCoriolis3(params, nu), nuRelative),
);
const nodeAdded = negative(
  multiply(addedMassCoriolis3(params, nuRelative), nuRelative),
);
const nodeTotal = add(nodeRigid, nodeAdded);
const dartRigid = negative(multiply(rigidBodyCoriolis3(params, nu), nu));
const dartAdded = negative(multiply(addedMassCoriolis3(params, nu), nu));
const existingMunk =
  (yvDot - xuDot) * (nuRelative[0] * nuRelative[1] - nu[0] * nu[1]);
const existingPlugin = [0, 0, existingMunk];
const before = add(dartRigid, dartAdded, existingPlugin);
const requiredCorrection = nodeTotal.map(
  (value, index) => value - before[index],
);
const implementedCorrection = [
  -yvDot * nuRelative[1] * nu[2] - -yvDot * nu[1] * nu[2],
  xuDot * nuRelative[0] * nu[2] - xuDot * nu[0] * nu[2],
  mass * (nu[1] * current[0] - nu[0] * current[1]),
];
const after = add(before, implementedCorrection);
const report = {
  schema_version: 1,
  artifact_kind: "vrx-current-coriolis-static-probe",
  state_body_ned: { nu, current, nu_relative: nuRelative },
  conventions: {
    node: "-C_RB(nu)*nu_r - C_A(nu_r)*nu_r",
    dart: "-C_RB(nu)*nu - C_A(nu)*nu",
    existing_plugin: "added-mass Munk yaw difference only",
  },
  wrench_body_n_nm: {
    node: { rigid: nodeRigid, added_mass: nodeAdded, total: nodeTotal },
    vrx_before: {
      dart_rigid: dartRigid,
      dart_added_mass: dartAdded,
      existing_plugin: existingPlugin,
      total: before,
    },
    node_minus_vrx_before: requiredCorrection,
    implemented_additional_plugin_correction: implementedCorrection,
    vrx_after: after,
    node_minus_vrx_after: nodeTotal.map((value, index) => value - after[index]),
  },
  confirmation: {
    predicted_rigid_yaw_correction_nm:
      mass * (nu[1] * current[0] - nu[0] * current[1]),
    observed_total_yaw_residual_nm: requiredCorrection[2],
    hypothesis_confirmed: Math.abs(requiredCorrection[2]) > 40,
    corrected_match_max_abs: Math.max(
      ...nodeTotal.map((value, index) => Math.abs(value - after[index])),
    ),
  },
};
assert(report.confirmation.hypothesis_confirmed);
assert(report.confirmation.corrected_match_max_abs < 1e-12);
const output = resolve(
  process.argv[2] ??
    "artifacts/rl-campaign/vrx-current-coriolis/static-probe-before.json",
);
mkdirSync(dirname(output), { recursive: true });
writeFileSync(output, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
