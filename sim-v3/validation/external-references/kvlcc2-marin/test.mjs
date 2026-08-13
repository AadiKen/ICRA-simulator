import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import { spawnSync } from "node:child_process";

const readJson = async (path) => JSON.parse(await readFile(path));
const l7Path = "validation/external-references/kvlcc2/kvlcc2-l7.json";
const oldHash = createHash("sha256").update(await readFile(l7Path)).digest("hex");

for (const script of ["import.mjs", "score.mjs"]) {
  const run = spawnSync(
    process.execPath,
    [`validation/external-references/kvlcc2-marin/${script}`],
    { encoding: "utf8" },
  );
  assert.equal(run.status, 0, run.stderr);
}
assert.equal(
  createHash("sha256").update(await readFile(l7Path)).digest("hex"),
  oldHash,
);

const manifest = await readJson("validation/datasets/manifests/kvlcc2-simman2008-third-party.json");
const config = await readJson("validation/external-references/kvlcc2-marin/kvlcc2-marin.json");
const artifact = await readJson("validation/external-references/kvlcc2-marin/artifacts/kvlcc2-marin-trajectory-comparison.json");
const inventory = await readJson("validation/datasets/inventories/kvlcc2-simman2008-third-party.json");
const policy = await readJson("validation/external-references/kvlcc2-marin/trajectory-tolerances.json");
const identificationPlan = await readJson("validation/external-references/kvlcc2-marin/identification-plan.json");
const identification = await readJson("validation/external-references/kvlcc2-marin/artifacts/identification/identification-report.json");
const hsva = await readJson("validation/external-references/kvlcc2-marin/hsva-preprocessing-manifest.json");

assert.equal(manifest.license.redistributionPermitted, false);
assert.equal(manifest.hsva.status, "preprocessed-cache-only-fallback-staged");
assert.equal(manifest.marin.columnMapping.length, 11);
assert.equal(manifest.marin.columnMapping[10].status, "identified-measured-input");
assert.match(manifest.marin.rudderAdapter.modelRudderRadFormula, /column_10/);
assert.match(manifest.marin.rudderAdapter.rateHandling, /no second rate limiter/);
assert.equal(config.principal_particulars.propeller_diameter_m, 0.204);
assert.equal(config.principal_particulars.x_g_m, 0.2436);
assert.equal(config.discrepancy_resolution.kvlcc2_l7_unmodified, true);
assert.equal(config.propeller_operating_point.status, "resolved-measured-per-sample");
assert.equal(config.propeller_operating_point.use_mean, false);

assert.equal(policy.schema_version, 2);
assert.ok(policy.diagnostic_only.includes("full_track_position_rmse_lpp"));
assert.equal(policy.trajectory.bounded_window_heading_rmse_deg_max, 15);
assert.deepEqual(identificationPlan.coefficient_groups.turning_circle_identified.source_runs, [
  "MARIN_FREE_KVLCC2_tc_-35_m",
  "MARIN_FREE_KVLCC2_tc_35_m",
]);
assert.equal(identificationPlan.coefficient_groups.held_out_validation.source_runs.length, 4);
assert.equal(identificationPlan.schema_version, 2);
assert.equal(identificationPlan.coefficient_groups.fixed_added_mass.fit_permitted, false);
assert.deepEqual(identificationPlan.coefficient_groups.fixed_added_mass.coefficients, ["m_x", "m_y", "J_z"]);
assert.equal(identificationPlan.failure_interpretation.precommitted, true);
assert.match(identificationPlan.failure_interpretation.prohibited_response, /Do not move/);
assert.equal(identification.status, "executed-insufficient-yaw-transient-coverage");
assert.ok(Object.values(identification.training).every((run) => run.imo_passed));
assert.ok(Object.values(identification.held_out).every((run) => !run.imo_passed));
assert.deepEqual(Object.keys(identification.fixed_added_mass).sort(), ["J_z", "m_x", "m_y"]);
assert.equal(hsva.status, "preprocessed-cache-only");
assert.equal(hsva.runs.length, 13);
assert.equal(hsva.runs.reduce((sum, run) => sum + run.rows, 0), 42400);
assert.ok(hsva.runs.every((run) => /^[a-f0-9]{64}$/.test(run.processed_sha256)));

assert.equal(artifact.status, "experimental-trajectory-scored-fail");
assert.equal(artifact.runs.length, 6);
assert.equal(artifact.port_starboard_aggregated, false);
assert.equal(artifact.vehicle_b_evidence_tiers.length, 4);
assert.equal(artifact.anonymous_release_included, false);
assert.equal(artifact.simulator_scoring.executed, true);
assert.equal(artifact.simulator_scoring.passed, false);
assert.ok(artifact.simulator_scoring.runs.every((run) =>
  run.measured_inputs.samplewise
  && run.measured_inputs.rudder_rate_handling === "measured-actual-angle-no-second-limiter"
  && run.trajectory.rudder_input_replay_rmse_deg < 1e-12
  && run.trajectory.propeller_input_replay_rmse_rps === 0
  && run.trajectory.bounded_window_heading_rmse_deg <= 15
));
assert.equal(inventory.fileCount, 28);
assert.ok(inventory.files.every((file) => file.release_inclusion === false));
assert.ok(artifact.runs.every((run) => run.column_count === 11 && run.mapping_evidence.passed));
assert.equal(artifact.trajectory_scaling.full_scale_lpp_m, 320);
assert.equal(artifact.published_index_consistency.all_within_10_percent, true);
assert.ok(artifact.runs
  .filter((run) => run.maneuver === "turning-circle")
  .every((run) => run.gating_metrics.advance_lpp > 3 && run.gating_metrics.advance_lpp < 3.5));

console.log("KVLCC2 MARIN cache, conventions, measured inputs, scoring policy, and identification split passed");
