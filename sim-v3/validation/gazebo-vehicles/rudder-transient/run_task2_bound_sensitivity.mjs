#!/usr/bin/env node
import { createHash } from "node:crypto";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { spawnSync } from "node:child_process";
import { pathToFileURL } from "node:url";

const sourcePath = "validation/external-references/kvlcc2-marin/identify.mjs";
const originalReportPath = "validation/external-references/kvlcc2-marin/artifacts/identification/identification-report.json";
const protocolPath = "artifacts/gazebo/vehicle-b/rudder-transient/task2-bound-sensitivity-protocol.json";
const outputDir = "artifacts/gazebo/vehicle-b/rudder-transient/task2-fit";
const reportPath = "artifacts/gazebo/vehicle-b/rudder-transient/task2-bound-sensitivity-report.json";
const terms = ["Y_R", "N_R", "Y_rrr", "N_rrr", "Y_vvr", "Y_vrr", "N_vvr", "N_vrr", "X_vr"];
const relaxed = new Set(["Y_R", "X_vr"]);
const sha256 = (bytes) => createHash("sha256").update(bytes).digest("hex");
const median = (values) => { const sorted = [...values].sort((a, b) => a - b); return (sorted[1] + sorted[2]) / 2; };
const percentReduction = (before, after) => 100 * (before - after) / before;

async function main() {
  const protocolBytes = await readFile(protocolPath);
  const protocol = JSON.parse(protocolBytes);
  if (protocol.status !== "registered-before-task2-results") throw new Error("Task 2 protocol is not preregistered");
  const source = await readFile(sourcePath, "utf8");
  const mmgUrl = pathToFileURL(`${process.cwd()}/packages/core/src/mmg.js`).href;
  let transformed = source
    .replace('from "../../../packages/core/src/mmg.js"', `from "${mmgUrl}"`)
    .replace('const outputDir = `${root}/artifacts/identification`;', `const outputDir = "${outputDir}";`)
    .replace(
      "const candidateQ = q.map((entry, index) => Math.max(-2, Math.min(2, entry + step[index])));",
      "const candidateQ = q.map((entry, index) => { const limit = [0, 8].includes(index) ? 10 : 2; return Math.max(-limit, Math.min(limit, entry + step[index])); });",
    );
  if (transformed === source || !transformed.includes("[0, 8].includes(index)")) throw new Error("Could not apply isolated Task 2 runner transformation");
  await mkdir(outputDir, { recursive: true });
  const execution = spawnSync(process.execPath, ["--input-type=module", "-e", transformed], { cwd: process.cwd(), encoding: "utf8", maxBuffer: 20 * 1024 * 1024 });
  if (execution.status !== 0) throw new Error(`Task 2 fit failed:\n${execution.stderr || execution.stdout}`);

  const original = JSON.parse(await readFile(originalReportPath));
  const rawReportPath = `${outputDir}/identification-report.json`;
  const raw = JSON.parse(await readFile(rawReportPath));
  raw.artifact_kind = "kvlcc2-marin-task2-relaxed-bound-identification";
  raw.status = "executed-task2-bound-sensitivity-diagnostic";
  raw.plan_checksum_input = protocol;
  raw.bound_policy = "Y_R and X_vr use q in [-10,10]; all other fitted terms retain q in [-2,2]";
  raw.supersedes_original_fit = false;
  await writeFile(rawReportPath, `${JSON.stringify(raw, null, 2)}\n`);
  const fittedPath = `${outputDir}/fitted-kvlcc2-marin.json`;
  const fitted = JSON.parse(await readFile(fittedPath));
  fitted.id = "kvlcc2-marin-turning-circle-task2-relaxed-bound-diagnostic";
  fitted.identification.plan = protocolPath;
  fitted.identification.bounds = "Y_R and X_vr constrained to baseline +/-10*max(abs(baseline),0.05); other terms retain +/-2 scales";
  fitted.identification.analysis_role = "new Task 2 bound-sensitivity diagnostic; does not supersede the original fit";
  await writeFile(fittedPath, `${JSON.stringify(fitted, null, 2)}\n`);

  const coefficientRows = terms.map((term) => {
    const baseline = raw.coefficients[term].baseline;
    const identified = raw.coefficients[term].identified;
    const scale = Math.max(Math.abs(baseline), 0.05);
    const q = (identified - baseline) / scale;
    const limit = relaxed.has(term) ? 10 : 2;
    return {
      coefficient: term, baseline, identified, q, normalized_bounds: [-limit, limit],
      coefficient_bounds: [baseline - limit * scale, baseline + limit * scale],
      bound_relaxed: relaxed.has(term),
      at_bound: Math.abs(Math.abs(q) - limit) <= 1e-9,
      stable_away_from_bound: Math.abs(q) <= 0.95 * limit,
    };
  });
  const cases = Object.keys(raw.held_out).map((id) => {
    const before = original.held_out[id], after = raw.held_out[id];
    return {
      id,
      heading_rmse_deg: { original: before.heading_rmse_deg, relaxed: after.heading_rmse_deg, reduction_percent: percentReduction(before.heading_rmse_deg, after.heading_rmse_deg) },
      yaw_rate_rmse_deg_s: { original: before.yaw_rate_rmse_deg_s, relaxed: after.yaw_rate_rmse_deg_s, reduction_percent: percentReduction(before.yaw_rate_rmse_deg_s, after.yaw_rate_rmse_deg_s) },
      imo: after.imo,
      imo_passed: after.imo_passed,
    };
  });
  const originalHeading = median(cases.map((row) => row.heading_rmse_deg.original));
  const relaxedHeading = median(cases.map((row) => row.heading_rmse_deg.relaxed));
  const originalYaw = median(cases.map((row) => row.yaw_rate_rmse_deg_s.original));
  const relaxedYaw = median(cases.map((row) => row.yaw_rate_rmse_deg_s.relaxed));
  const headingReduction = percentReduction(originalHeading, relaxedHeading);
  const yawReduction = percentReduction(originalYaw, relaxedYaw);
  const substantiallyImproved = headingReduction >= 20 && yawReduction >= 20;
  const relaxedRows = coefficientRows.filter((row) => row.bound_relaxed);
  const ranToNewBound = relaxedRows.some((row) => row.at_bound);
  const report = {
    schema_version: 1,
    artifact_kind: "vehicle-b-rudder-transient-bound-sensitivity-result",
    date: "2026-09-04",
    status: "task2-completed-separate-diagnostic",
    protocol: { path: protocolPath, checksum_sha256: sha256(protocolBytes), registered_before_results: true },
    source_fit: { report: originalReportPath, checksum_sha256: sha256(await readFile(originalReportPath)), unchanged: true },
    premise_audit: protocol.source_audit,
    method: { base_runner: sourcePath, base_runner_checksum_sha256: sha256(source), held_constant: ["two turning-circle training runs", "four held-out zig-zags", "residual definitions and normalization", "ridge weight", "zero initialization", "12-iteration deterministic Gauss-Newton schedule"], changed_only: "Y_R and X_vr normalized bound half-width: 2 to 10" },
    optimization: { original_final_cost: original.final_cost, relaxed_final_cost: raw.final_cost, coefficient_rows: coefficientRows, formerly_pinned_ran_to_new_bound: ranToNewBound },
    held_out_zig_zags: cases,
    preregistered_summary: { original_median_heading_rmse_deg: originalHeading, relaxed_median_heading_rmse_deg: relaxedHeading, heading_reduction_percent: headingReduction, original_median_yaw_rate_rmse_deg_s: originalYaw, relaxed_median_yaw_rate_rmse_deg_s: relaxedYaw, yaw_rate_reduction_percent: yawReduction, threshold_percent_each: 20, substantial_improvement: substantiallyImproved },
    interpretation: substantiallyImproved && !ranToNewBound ? "Supports the too-tight-bound explanation for the reproducible bound-hit terms." : ranToNewBound ? "A formerly pinned coefficient ran to the substantially relaxed bound, supporting an underconstrained/underdetermined interpretation rather than a merely too-tight original bound." : "Relaxing the reproducible active bounds did not meet the preregistered substantial-improvement rule; the too-tight-bound explanation is not supported by this check.",
    provenance_finding: "The coefficient baseline values are recorded from Yasukawa & Yoshimura (2015) Table 3. The +/-2 optimization bounds are a repository identification choice, not literature-sourced physical bounds; therefore there is no evidence that the numeric bounds were copied from a different hull scale.",
    claim_limit: "KVLCC2 model-scale Task 2 diagnostic only. This does not change the original Vehicle B protocol, fit, scoped negative result, or Vehicle-B-USV parameters.",
    files: { fitted_parameters: fittedPath, raw_identification_report: rawReportPath },
  };
  await writeFile(reportPath, `${JSON.stringify(report, null, 2)}\n`);
  console.log(JSON.stringify({ report: reportPath, summary: report.preregistered_summary, relaxed_coefficients: relaxedRows, interpretation: report.interpretation }, null, 2));
}

main().catch((error) => { console.error(error.stack ?? error.message); process.exitCode = 1; });
