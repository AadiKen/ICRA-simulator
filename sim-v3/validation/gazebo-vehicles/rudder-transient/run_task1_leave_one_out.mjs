#!/usr/bin/env node
import { mkdir, readFile, writeFile } from "node:fs/promises";
import path from "node:path";
import { createMmgParameters, MmgManeuveringModel, MmgPlanarSimulator } from "../../../packages/core/src/mmg.js";

const ROOT = "validation/external-references/kvlcc2-marin";
const CACHE = "validation/datasets/raw/kvlcc2-simman2008-third-party/kvlcc2-marin";
const PROTOCOL = "artifacts/gazebo/vehicle-b/rudder-transient/task1-protocol.json";
const OUTPUT = "artifacts/gazebo/vehicle-b/rudder-transient/task1-results.json";
const SCALE = 45.7, SQRT_SCALE = Math.sqrt(SCALE), LPP = 320;
const TERMS = ["Y_R", "N_R", "Y_rrr", "N_rrr", "Y_vvr", "Y_vrr", "N_vvr", "N_vrr", "X_vr"];
const IDS = ["MARIN_FREE_KVLCC2_tc_-35_m", "MARIN_FREE_KVLCC2_tc_35_m", "MARIN_FREE_KVLCC2_zz_-10_m", "MARIN_FREE_KVLCC2_zz_10_m", "MARIN_FREE_KVLCC2_zz_-20_m", "MARIN_FREE_KVLCC2_zz_20_m"];
const rad = d => d * Math.PI / 180, deg = r => r * 180 / Math.PI;
const wrapDeg = d => deg(Math.atan2(Math.sin(rad(d)), Math.cos(rad(d))));
const value = x => typeof x === "number" ? x : x.value;
const clone = structuredClone;
const mse = xs => xs.reduce((s, x) => s + x * x, 0) / xs.length;
const rms = xs => Math.sqrt(mse(xs));
const parse = text => text.split(/\r?\n/).slice(3).filter(x => x.trim()).map(x => x.trim().split(/\s+/).map(Number));

function solve(matrix, rhs) {
  const n = rhs.length, a = matrix.map((row, i) => [...row, rhs[i]]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    [a[col], a[pivot]] = [a[pivot], a[col]];
    if (Math.abs(a[col][col]) < 1e-12) throw new Error("Identification normal matrix is singular");
    const divisor = a[col][col];
    for (let j = col; j <= n; j++) a[col][j] /= divisor;
    for (let row = 0; row < n; row++) if (row !== col) {
      const factor = a[row][col];
      for (let j = col; j <= n; j++) a[row][j] -= factor * a[col][j];
    }
  }
  return a.map(row => row[n]);
}

function configured(base, marin, coefficients, trainingIds) {
  const result = clone(base);
  result.id = "kvlcc2-marin-task1-leave-one-out";
  result.principal_particulars.propeller_diameter.value = marin.principal_particulars.propeller_diameter_m;
  result.principal_particulars.x_g.value = marin.principal_particulars.x_g_m;
  for (const key of ["k0", "k1", "k2"]) result.propeller[key].value = marin.propeller_open_water_fit[key];
  for (const [term, coefficient] of Object.entries(coefficients)) result.hull_derivatives[term] = { value: coefficient, unit: "nondimensional", provenance: { source: "MARIN KVLCC2 five-trajectory leave-one-out fold", runs: trainingIds, role: "new-diagnostic-identification-not-validation", method: "deterministic bounded regularized Gauss-Newton" } };
  return result;
}

function prepare(id, rows, stride) {
  const dtFull = (rows.at(-1)[0] - rows[0][0]) / (rows.length - 1);
  return { id, rows, stride, dt: dtFull * stride / SQRT_SCALE };
}

function simulate(reference, config, collect = false) {
  const params = createMmgParameters(config, { rho: 1000 }), rows = reference.rows, first = rows[0];
  const u0 = first[5] / SQRT_SCALE, n0 = first[10] * SQRT_SCALE / 60;
  const force = new MmgManeuveringModel(params);
  const resistance = force.computeComponents({ u: u0, v: 0, r: 0 }, { rudder_rad: 0, propeller_rps: n0 }).total[0];
  const sim = new MmgPlanarSimulator(params, { dt: reference.dt, rudderRateRadS: Infinity });
  const state = { x: first[1] / SCALE, y: first[2] / SCALE, psi: rad(first[4]), u: u0, v: first[6] / SQRT_SCALE, r: rad(first[8]) * SQRT_SCALE, delta: rad(-first[9]) };
  const residuals = [], heading = [], yawRate = [], position = [], trace = [{ x: first[1], y: first[2], heading: first[4] }];
  for (let i = reference.stride; i < rows.length; i += reference.stride) {
    const source = rows[i - reference.stride];
    sim.step(state, { rudder_rad: rad(-source[9]), propeller_rps: source[10] * SQRT_SCALE / 60, straight_resistance_n: resistance, resistance_reference_speed_mps: u0 });
    const measured = rows[i], h = wrapDeg(deg(state.psi) - measured[4]), r = deg(state.r) / SQRT_SCALE - measured[8], px = state.x * SCALE - measured[1], py = state.y * SCALE - measured[2];
    if (measured[0] - first[0] <= 900 && i % (reference.stride * 20) === 0) residuals.push(h / 5, r / 0.05, px / (0.5 * LPP), py / (0.5 * LPP));
    heading.push(h); yawRate.push(r); position.push(Math.hypot(px, py) / LPP);
    if (collect) trace.push({ x: state.x * SCALE, y: state.y * SCALE, heading: deg(state.psi) });
  }
  return { residuals, trace, heading_rmse_deg: rms(heading), yaw_rate_rmse_deg_s: rms(yawRate), position_rmse_lpp: rms(position) };
}

function crossing(rows, target, start = 1) { for (let i = start; i < rows.length; i++) if ((rows[i - 1].heading - target) * (rows[i].heading - target) <= 0) return i; return -1; }
function interpolate(rows, i, target, key) { const a = rows[i - 1], b = rows[i], f = (target - a.heading) / (b.heading - a.heading); return a[key] + f * (b[key] - a[key]); }
function imoMetrics(rows, id) {
  const h0 = rows[0].heading;
  if (id.includes("_tc_")) {
    const direction = Math.sign(rows.at(-1).heading - h0) || 1, i90 = crossing(rows, h0 + 90 * direction), i180 = crossing(rows, h0 + 180 * direction);
    if (i90 < 0 || i180 < 0) return null;
    const c = Math.cos(rad(h0)), s = Math.sin(rad(h0)), x0 = rows[0].x, y0 = rows[0].y;
    const project = (x, y) => ({ along: (x - x0) * c + (y - y0) * s, cross: -(x - x0) * s + (y - y0) * c });
    const p90 = project(interpolate(rows, i90, h0 + 90 * direction, "x"), interpolate(rows, i90, h0 + 90 * direction, "y"));
    const p180 = project(interpolate(rows, i180, h0 + 180 * direction, "x"), interpolate(rows, i180, h0 + 180 * direction, "y"));
    return { advance_lpp: p90.along / LPP, tactical_diameter_lpp: Math.abs(p180.cross) / LPP };
  }
  const angle = Math.abs(Number(/_zz_(-?\d+)/.exec(id)?.[1])), positive = crossing(rows, h0 + angle), negative = crossing(rows, h0 - angle);
  const sign = positive > 0 && (negative < 0 || positive < negative) ? 1 : -1, first = sign > 0 ? positive : negative, opposite = crossing(rows, h0 - sign * angle, Math.max(first, 1));
  if (first < 0 || opposite < 0) return null;
  const headings = rows.map(x => x.heading), firstPeak = sign > 0 ? Math.max(...headings.slice(first, opposite)) : Math.min(...headings.slice(first, opposite)), third = crossing(rows, h0 + sign * angle, opposite), secondPeak = sign > 0 ? Math.min(...headings.slice(opposite, third < 0 ? rows.length : third)) : Math.max(...headings.slice(opposite, third < 0 ? rows.length : third));
  return { first_overshoot_deg: Math.abs(firstPeak - h0) - angle, second_overshoot_deg: Math.abs(secondPeak - h0) - angle };
}

async function main() {
  const protocol = JSON.parse(await readFile(PROTOCOL, "utf8"));
  if (protocol.status !== "precommitted-before-results" || protocol.case_inventory.count !== 6) throw new Error("Task 1 protocol is absent or not precommitted");
  const base = JSON.parse(await readFile("validation/external-references/kvlcc2/kvlcc2-l7.json", "utf8"));
  const marin = JSON.parse(await readFile(`${ROOT}/kvlcc2-marin.json`, "utf8"));
  const source = JSON.parse(await readFile(`${ROOT}/artifacts/kvlcc2-marin-trajectory-comparison.json`, "utf8"));
  const policy = JSON.parse(await readFile(`${ROOT}/trajectory-tolerances.json`, "utf8"));
  const sourceById = Object.fromEntries(source.runs.map(x => [x.id, x]));
  const raw = Object.fromEntries(await Promise.all(IDS.map(async id => [id, parse(await readFile(path.join(CACHE, `${id}.dat`), "utf8"))])));
  const baseline = Object.fromEntries(TERMS.map(term => [term, value(base.hull_derivatives[term])]));
  const scales = Object.fromEntries(TERMS.map(term => [term, Math.max(Math.abs(baseline[term]), 0.05)]));
  const coefficientValues = q => Object.fromEntries(TERMS.map((term, i) => [term, baseline[term] + scales[term] * q[i]]));
  const folds = [];
  for (const heldOutId of IDS) {
    const trainingIds = IDS.filter(id => id !== heldOutId), references = Object.fromEntries(IDS.map(id => [id, prepare(id, raw[id], 5)]));
    const residual = q => {
      const model = configured(base, marin, coefficientValues(q), trainingIds);
      return [...trainingIds.flatMap(id => simulate(references[id], model).residuals), ...q.map(x => 0.15 * x)];
    };
    let q = TERMS.map(() => 0), lambda = 0.1, current = residual(q); const iterations = [];
    for (let iteration = 0; iteration < 12; iteration++) {
      const jac = Array.from({ length: current.length }, () => Array(TERMS.length).fill(0));
      for (let col = 0; col < TERMS.length; col++) { const p = [...q]; p[col] += 0.01; const next = residual(p); for (let row = 0; row < current.length; row++) jac[row][col] = (next[row] - current[row]) / 0.01; }
      const normal = Array.from({ length: TERMS.length }, () => Array(TERMS.length).fill(0)), gradient = Array(TERMS.length).fill(0);
      for (let row = 0; row < current.length; row++) for (let a = 0; a < TERMS.length; a++) { gradient[a] += jac[row][a] * current[row]; for (let b = 0; b < TERMS.length; b++) normal[a][b] += jac[row][a] * jac[row][b]; }
      for (let i = 0; i < TERMS.length; i++) normal[i][i] += lambda;
      const step = solve(normal, gradient.map(x => -x)), candidateQ = q.map((x, i) => Math.max(-2, Math.min(2, x + step[i]))), candidate = residual(candidateQ), accepted = mse(candidate) < mse(current);
      iterations.push({ iteration, cost: mse(current), candidate_cost: mse(candidate), lambda, accepted });
      if (accepted) { q = candidateQ; current = candidate; lambda *= 0.5; } else lambda *= 5;
    }
    const model = configured(base, marin, coefficientValues(q), trainingIds), held = simulate(prepare(heldOutId, raw[heldOutId], 1), model, true), primary = simulate(prepare(heldOutId, raw[heldOutId], 5), model), actual = imoMetrics(held.trace, heldOutId), reference = sourceById[heldOutId].gating_metrics;
    const kinds = heldOutId.includes("_tc_") ? { advance_lpp: "advance", tactical_diameter_lpp: "tactical_diameter" } : { first_overshoot_deg: "first_overshoot", second_overshoot_deg: "second_overshoot" };
    const imo = Object.fromEntries(Object.entries(kinds).map(([metric, kind]) => { const a = actual?.[metric] ?? null, error = a == null ? null : Math.abs(a - reference[metric]) / Math.abs(reference[metric]) * 100; return [metric, { reference: reference[metric], actual: a, absolute_percent_error: error, tolerance_percent: policy.imo_percent[kind], passed: error != null && error <= policy.imo_percent[kind] }]; }));
    const coefficients = Object.fromEntries(TERMS.map((term, i) => [term, { baseline: baseline[term], fitted: coefficientValues(q)[term], normalized_q: q[i], lower: baseline[term] - 2 * scales[term], upper: baseline[term] + 2 * scales[term], hit_bound: Math.abs(Math.abs(q[i]) - 2) <= 1e-9 }]));
    folds.push({ held_out_id: heldOutId, maneuver: heldOutId.includes("_tc_") ? "turning-circle" : "zig-zag", training_ids: trainingIds, fit_error: { initial_regularized_training_mse: iterations[0].cost, final_regularized_training_mse: mse(current), held_out_primary_normalized_mse: mse(primary.residuals), held_out_heading_rmse_deg: held.heading_rmse_deg, held_out_yaw_rate_rmse_deg_s: held.yaw_rate_rmse_deg_s, held_out_position_rmse_lpp: held.position_rmse_lpp }, coefficients, bound_hits: TERMS.filter(term => coefficients[term].hit_bound), imo, held_out_imo_passed: Object.values(imo).every(x => x.passed), iterations });
    console.log(`${heldOutId}: held-out normalized MSE=${mse(primary.residuals).toFixed(6)}, bounds=${folds.at(-1).bound_hits.join(",") || "none"}`);
  }
  const turning = folds.filter(x => x.maneuver === "turning-circle"), zigzag = folds.filter(x => x.maneuver === "zig-zag"), maxTurning = Math.max(...turning.map(x => x.fit_error.held_out_primary_normalized_mse)), minZigzag = Math.min(...zigzag.map(x => x.fit_error.held_out_primary_normalized_mse)), supported = minZigzag > maxTurning;
  for (const fold of folds) {
    const error = fold.fit_error.held_out_primary_normalized_mse;
    const matched = fold.maneuver === "zig-zag" ? error > maxTurning : error < minZigzag;
    fold.prediction_comparison = {
      expected_ordering: fold.maneuver === "zig-zag" ? "error greater than both turning-circle folds" : "error less than all four zig-zag folds",
      comparison_extreme: fold.maneuver === "zig-zag" ? maxTurning : minZigzag,
      result: matched ? "matched" : "contradicted",
    };
  }
  const report = { schema_version: 1, artifact_kind: "vehicle-b-rudder-transient-leave-one-out-results", status: "completed", protocol: PROTOCOL, case_count_confirmed: folds.length, model_scope: "KVLCC2 MARIN L7 model-scale diagnostic; no Vehicle B USV coefficient validation", folds, prediction_assessment: { rule: protocol.prediction.support_rule, max_turning_circle_primary_error: maxTurning, min_zig_zag_primary_error: minZigzag, strict_separation: supported, result: supported ? "supported" : "not-supported", task3_action: supported ? "eligible-under-task1-condition" : "stop-for-review", statement: supported ? "Every held-out zig-zag error exceeded both held-out turning-circle errors, supporting the prospective coverage-gap prediction." : "The per-fold errors do not strictly separate by maneuver type; Task 1 does not support the coverage-gap hypothesis." }, aggregation_guard: protocol.prediction.aggregation_guard, immutability: protocol.immutability };
  await mkdir(path.dirname(OUTPUT), { recursive: true });
  await writeFile(OUTPUT, JSON.stringify(report, null, 2) + "\n");
  console.log(JSON.stringify(report.prediction_assessment, null, 2));
}

main().catch(error => { console.error(error.stack ?? error.message); process.exitCode = 1; });
