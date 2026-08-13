#!/usr/bin/env node
import { readFile, mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { createMmgParameters, MmgManeuveringModel, MmgPlanarSimulator } from "../../../packages/core/src/mmg.js";

const root = "validation/external-references/kvlcc2-marin";
const cache = "validation/datasets/raw/kvlcc2-simman2008-third-party/kvlcc2-marin";
const outputDir = `${root}/artifacts/identification`;
const scale = 45.7;
const sqrtScale = Math.sqrt(scale);
const lppFull = 320;
const rad = (degrees) => degrees * Math.PI / 180;
const deg = (radians) => radians * 180 / Math.PI;
const wrapDeg = (value) => deg(Math.atan2(Math.sin(rad(value)), Math.cos(rad(value))));
const parse = (text) => text.split(/\r?\n/).slice(3).filter((line) => line.trim()).map((line) => line.trim().split(/\s+/).map(Number));
const clone = (value) => structuredClone(value);
const value = (entry) => typeof entry === "number" ? entry : entry.value;
const fittedTerms = ["Y_R", "N_R", "Y_rrr", "N_rrr", "Y_vvr", "Y_vrr", "N_vvr", "N_vrr", "X_vr"];
const trainingIds = ["MARIN_FREE_KVLCC2_tc_-35_m", "MARIN_FREE_KVLCC2_tc_35_m"];
const validationIds = ["MARIN_FREE_KVLCC2_zz_-10_m", "MARIN_FREE_KVLCC2_zz_10_m", "MARIN_FREE_KVLCC2_zz_-20_m", "MARIN_FREE_KVLCC2_zz_20_m"];

function crossing(rows, target, start = 1) { for (let index = start; index < rows.length; index++) if ((rows[index - 1].heading - target) * (rows[index].heading - target) <= 0) return index; return -1; }
function interpolate(rows, index, target, key) { const before = rows[index - 1], after = rows[index], fraction = (target - before.heading) / (after.heading - before.heading); return before[key] + fraction * (after[key] - before[key]); }
function imoMetrics(rows, id) {
  const h0 = rows[0].heading;
  if (id.includes("_tc_")) {
    const direction = Math.sign(rows.at(-1).heading - h0) || 1;
    const i90 = crossing(rows, h0 + 90 * direction), i180 = crossing(rows, h0 + 180 * direction);
    if (i90 < 0 || i180 < 0) return null;
    const x0 = rows[0].x, y0 = rows[0].y, c = Math.cos(rad(h0)), s = Math.sin(rad(h0));
    const project = (x, y) => ({ along: (x - x0) * c + (y - y0) * s, cross: -(x - x0) * s + (y - y0) * c });
    const p90 = project(interpolate(rows, i90, h0 + 90 * direction, "x"), interpolate(rows, i90, h0 + 90 * direction, "y"));
    const p180 = project(interpolate(rows, i180, h0 + 180 * direction, "x"), interpolate(rows, i180, h0 + 180 * direction, "y"));
    return { advance_lpp: p90.along / lppFull, tactical_diameter_lpp: Math.abs(p180.cross) / lppFull };
  }
  const angle = Math.abs(Number(/_zz_(-?\d+)/.exec(id)?.[1]));
  const positive = crossing(rows, h0 + angle), negative = crossing(rows, h0 - angle);
  const sign = positive > 0 && (negative < 0 || positive < negative) ? 1 : -1;
  const first = sign > 0 ? positive : negative, opposite = crossing(rows, h0 - sign * angle, Math.max(first, 1));
  if (first < 0 || opposite < 0) return null;
  const headings = rows.map((row) => row.heading);
  const firstPeak = sign > 0 ? Math.max(...headings.slice(first, opposite)) : Math.min(...headings.slice(first, opposite));
  const third = crossing(rows, h0 + sign * angle, opposite);
  const secondPeak = sign > 0 ? Math.min(...headings.slice(opposite, third < 0 ? rows.length : third)) : Math.max(...headings.slice(opposite, third < 0 ? rows.length : third));
  return { first_overshoot_deg: Math.abs(firstPeak - h0) - angle, second_overshoot_deg: Math.abs(secondPeak - h0) - angle };
}

function prepare(reference, stride = 5) {
  const rows = reference.rows;
  const dtFull = (rows.at(-1)[0] - rows[0][0]) / (rows.length - 1);
  return { ...reference, rows, stride, dt: dtFull * stride / sqrtScale };
}

function configuredBase(base, marin, coefficients) {
  const result = clone(base);
  result.id = "kvlcc2-marin-turning-circle-identified";
  result.principal_particulars.propeller_diameter.value = marin.principal_particulars.propeller_diameter_m;
  result.principal_particulars.x_g.value = marin.principal_particulars.x_g_m;
  result.propeller.k0.value = marin.propeller_open_water_fit.k0;
  result.propeller.k1.value = marin.propeller_open_water_fit.k1;
  result.propeller.k2.value = marin.propeller_open_water_fit.k2;
  for (const [term, coefficient] of Object.entries(coefficients)) {
    result.hull_derivatives[term] = {
      value: coefficient,
      unit: "nondimensional",
      provenance: {
        source: "MARIN KVLCC2 turning circles",
        runs: trainingIds,
        role: "identification-not-validation",
        method: "deterministic bounded regularized least squares",
      },
    };
  }
  return result;
}

function simulate(reference, config, collect = false) {
  const params = createMmgParameters(config, { rho: 1000 });
  const rows = reference.rows;
  const first = rows[0];
  const u0 = first[5] / sqrtScale;
  const n0 = first[10] * sqrtScale / 60;
  const force = new MmgManeuveringModel(params);
  const straightResistance = force.computeComponents({ u: u0, v: 0, r: 0 }, { rudder_rad: 0, propeller_rps: n0 }).total[0];
  const sim = new MmgPlanarSimulator(params, { dt: reference.dt, rudderRateRadS: Infinity });
  const state = { x: first[1] / scale, y: first[2] / scale, psi: rad(first[4]), u: u0, v: first[6] / sqrtScale, r: rad(first[8]) * sqrtScale, delta: rad(-first[9]) };
  const residuals = [];
  const trace = [];
  for (let sourceIndex = reference.stride; sourceIndex < rows.length; sourceIndex += reference.stride) {
    const source = rows[Math.min(sourceIndex - reference.stride, rows.length - 1)];
    sim.step(state, {
      rudder_rad: rad(-source[9]),
      propeller_rps: source[10] * sqrtScale / 60,
      straight_resistance_n: straightResistance,
      resistance_reference_speed_mps: u0,
    });
    const measured = rows[sourceIndex];
    const fullTime = measured[0] - first[0];
    if (fullTime <= 900 && sourceIndex % (reference.stride * 20) === 0) {
      residuals.push(wrapDeg(deg(state.psi) - measured[4]) / 5);
      residuals.push((deg(state.r) / sqrtScale - measured[8]) / 0.05);
      residuals.push((state.x * scale - measured[1]) / (0.5 * lppFull));
      residuals.push((state.y * scale - measured[2]) / (0.5 * lppFull));
    }
    if (collect) trace.push({ time_full_scale_s: fullTime, x: state.x * scale, y: state.y * scale, heading: deg(state.psi), heading_error_deg: wrapDeg(deg(state.psi) - measured[4]), yaw_rate_error_deg_s: deg(state.r) / sqrtScale - measured[8] });
  }
  return { residuals, trace };
}

function solve(matrix, rhs) {
  const n = rhs.length;
  const augmented = matrix.map((row, index) => [...row, rhs[index]]);
  for (let column = 0; column < n; column++) {
    let pivot = column;
    for (let row = column + 1; row < n; row++) if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
    const divisor = augmented[column][column];
    if (Math.abs(divisor) < 1e-12) throw new Error("Identification normal matrix is singular");
    for (let entry = column; entry <= n; entry++) augmented[column][entry] /= divisor;
    for (let row = 0; row < n; row++) if (row !== column) {
      const factor = augmented[row][column];
      for (let entry = column; entry <= n; entry++) augmented[row][entry] -= factor * augmented[column][entry];
    }
  }
  return augmented.map((row) => row[n]);
}

async function main() {
  const plan = JSON.parse(await readFile(`${root}/identification-plan.json`));
  if (plan.status !== "precommitted-before-identification") throw new Error("Identification plan is not precommitted");
  const base = JSON.parse(await readFile("validation/external-references/kvlcc2/kvlcc2-l7.json"));
  const marin = JSON.parse(await readFile(`${root}/kvlcc2-marin.json`));
  const referenceArtifact = JSON.parse(await readFile(`${root}/artifacts/kvlcc2-marin-trajectory-comparison.json`));
  const policy = JSON.parse(await readFile(`${root}/trajectory-tolerances.json`));
  const referenceById = Object.fromEntries(referenceArtifact.runs.map((run) => [run.id, run]));
  const references = {};
  for (const id of [...trainingIds, ...validationIds]) references[id] = prepare({ id, rows: parse(await readFile(path.join(cache, `${id}.dat`), "utf8")) });
  const baseline = Object.fromEntries(fittedTerms.map((term) => [term, value(base.hull_derivatives[term])]));
  const scales = Object.fromEntries(fittedTerms.map((term) => [term, Math.max(Math.abs(baseline[term]), 0.05)]));
  const coefficients = (q) => Object.fromEntries(fittedTerms.map((term, index) => [term, baseline[term] + scales[term] * q[index]]));
  const residual = (q) => {
    const config = configuredBase(base, marin, coefficients(q));
    const data = trainingIds.flatMap((id) => simulate(references[id], config).residuals);
    return [...data, ...q.map((entry) => 0.15 * entry)];
  };
  const cost = (r) => r.reduce((sum, entry) => sum + entry * entry, 0) / r.length;
  let q = fittedTerms.map(() => 0);
  let lambda = 0.1;
  let current = residual(q);
  const iterations = [];
  for (let iteration = 0; iteration < 12; iteration++) {
    const epsilon = 0.01;
    const jacobian = Array.from({ length: current.length }, () => Array(fittedTerms.length).fill(0));
    for (let column = 0; column < fittedTerms.length; column++) {
      const perturbed = [...q]; perturbed[column] += epsilon;
      const next = residual(perturbed);
      for (let row = 0; row < current.length; row++) jacobian[row][column] = (next[row] - current[row]) / epsilon;
    }
    const normal = Array.from({ length: fittedTerms.length }, () => Array(fittedTerms.length).fill(0));
    const gradient = Array(fittedTerms.length).fill(0);
    for (let row = 0; row < current.length; row++) for (let a = 0; a < fittedTerms.length; a++) {
      gradient[a] += jacobian[row][a] * current[row];
      for (let b = 0; b < fittedTerms.length; b++) normal[a][b] += jacobian[row][a] * jacobian[row][b];
    }
    for (let index = 0; index < fittedTerms.length; index++) normal[index][index] += lambda;
    const step = solve(normal, gradient.map((entry) => -entry));
    const candidateQ = q.map((entry, index) => Math.max(-2, Math.min(2, entry + step[index])));
    const candidate = residual(candidateQ);
    const accepted = cost(candidate) < cost(current);
    iterations.push({ iteration, cost: cost(current), candidate_cost: cost(candidate), lambda, accepted });
    if (accepted) { q = candidateQ; current = candidate; lambda *= 0.5; } else lambda *= 5;
  }
  const fitted = configuredBase(base, marin, coefficients(q));
  fitted.identification = {
    plan: `${root}/identification-plan.json`,
    training_runs: trainingIds,
    held_out_runs: validationIds,
    fixed_added_mass: clone(base.added_mass),
    objective: "heading, yaw-rate, and position residuals through 900 full-scale seconds plus ridge regularization",
    bounds: "each fitted coefficient constrained to baseline plus/minus twice max(abs(baseline), 0.05)",
  };
  const finalReferences = Object.fromEntries(Object.entries(references).map(([id, reference]) => [id, prepare({ id, rows: reference.rows }, 1)]));
  const training = Object.fromEntries(trainingIds.map((id) => [id, simulate(finalReferences[id], fitted, true).trace]));
  const validation = Object.fromEntries(validationIds.map((id) => [id, simulate(finalReferences[id], fitted, true).trace]));
  const summary = (id, trace) => {
    const actual = imoMetrics([{ x: finalReferences[id].rows[0][1], y: finalReferences[id].rows[0][2], heading: finalReferences[id].rows[0][4] }, ...trace], id);
    const reference = referenceById[id].gating_metrics;
    const kinds = id.includes("_tc_") ? { advance_lpp: "advance", tactical_diameter_lpp: "tactical_diameter" } : { first_overshoot_deg: "first_overshoot", second_overshoot_deg: "second_overshoot" };
    const imo = Object.fromEntries(Object.entries(kinds).map(([metric, kind]) => {
      const error = actual?.[metric] == null ? null : Math.abs(actual[metric] - reference[metric]) / Math.abs(reference[metric]) * 100;
      return [metric, { reference: reference[metric], actual: actual?.[metric] ?? null, absolute_percent_error: error, tolerance_percent: policy.imo_percent[kind], passed: error != null && error <= policy.imo_percent[kind] }];
    }));
    return {
      samples: trace.length,
      heading_rmse_deg: Math.sqrt(trace.reduce((sum, row) => sum + row.heading_error_deg ** 2, 0) / trace.length),
      yaw_rate_rmse_deg_s: Math.sqrt(trace.reduce((sum, row) => sum + row.yaw_rate_error_deg_s ** 2, 0) / trace.length),
      imo,
      imo_passed: Object.values(imo).every((metric) => metric.passed),
    };
  };
  const report = {
    schema_version: 1,
    artifact_kind: "kvlcc2-marin-preregistered-identification",
    status: "executed-insufficient-yaw-transient-coverage",
    plan_checksum_input: plan,
    method: "deterministic bounded regularized Gauss-Newton",
    initial_cost: iterations[0].cost,
    final_cost: cost(current),
    iterations,
    coefficients: Object.fromEntries(fittedTerms.map((term) => [term, { baseline: baseline[term], identified: value(fitted.hull_derivatives[term]), provenance: fitted.hull_derivatives[term].provenance }])),
    fixed_added_mass: Object.fromEntries(Object.entries(base.added_mass).map(([term, entry]) => [term, entry])),
    training: Object.fromEntries(Object.entries(training).map(([id, trace]) => [id, summary(id, trace)])),
    held_out: Object.fromEntries(Object.entries(validation).map(([id, trace]) => [id, summary(id, trace)])),
    interpretation: plan.failure_interpretation,
    claim_limit: "KVLCC2 model-scale identification only; no transfer to Vehicle B USV coefficients.",
  };
  await mkdir(outputDir, { recursive: true });
  await writeFile(`${outputDir}/fitted-kvlcc2-marin.json`, JSON.stringify(fitted, null, 2) + "\n");
  await writeFile(`${outputDir}/identification-report.json`, JSON.stringify(report, null, 2) + "\n");
  console.log(JSON.stringify({ output: outputDir, initial_cost: report.initial_cost, final_cost: report.final_cost, coefficients: report.coefficients, training: report.training, held_out: report.held_out }, null, 2));
}

main().catch((error) => { console.error(error.stack ?? error.message); process.exitCode = 1; });
