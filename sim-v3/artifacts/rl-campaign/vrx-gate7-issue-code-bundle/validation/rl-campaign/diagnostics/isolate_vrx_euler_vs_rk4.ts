import {readFileSync, writeFileSync} from "node:fs";
import {DynamicsCore} from "../../core/dynamicsCore.js";
import {RigidBodyState} from "../../core/rigidBodyState.js";
import {stepDartSemiImplicitEuler, stepRK4} from "../../core/integrator.js";
import {AddedMassCoriolis, HydrodynamicDamping} from "../../packages/core/src/force-components.js";
import {SURVEYOR_PUBLIC_SPEC} from "../../packages/vehicle-sdk/src/surveyor.ts";
import {VehicleParameters} from "../../core/vehicleParameters.js";

const [telemetryPath, outputPath] = process.argv.slice(2);
if (!telemetryPath || !outputPath) throw new Error("usage: isolate_vrx_euler_vs_rk4.ts telemetry.csv result.json");

const DT = 0.05;
const START_TIMES = [1.9, 2.15, 2.45, 2.75, 3, 3.35, 3.7, 4.05, 4.5, 4.8, 5.15];
const lines = readFileSync(telemetryPath, "utf8").trim().split("\n");
const headers = lines[0].split(",");
const rows = lines.slice(1).map((line) => Object.fromEntries(line.split(",").map((value, index) => [headers[index], Number(value)])));
const nearest = (time: number) => rows.reduce((a, b) => Math.abs(b.time_s - time) < Math.abs(a.time_s - time) ? b : a);
const vrxNu = (row: Record<string, number>) => [row.velocity_body_x, -row.velocity_body_y, -row.angular_velocity_body_z];
const acceleration = (start: number[], end: number[]) => start.map((value, index) => (end[index] - value) / DT);
const subtract = (left: number[], right: number[]) => left.map((value, index) => value - right[index]);
const norm2 = (value: number[]) => Math.hypot(value[0], value[1]);
const pearson = (left: number[], right: number[]) => {
  const leftMean = left.reduce((sum, value) => sum + value, 0) / left.length;
  const rightMean = right.reduce((sum, value) => sum + value, 0) / right.length;
  const centered = left.map((value, index) => [value - leftMean, right[index] - rightMean]);
  const numerator = centered.reduce((sum, [x, y]) => sum + x * y, 0);
  const denominator = Math.sqrt(centered.reduce((sum, [x]) => sum + x * x, 0) * centered.reduce((sum, [, y]) => sum + y * y, 0));
  return denominator ? numerator / denominator : null;
};

const S = SURVEYOR_PUBLIC_SPEC;
const [Ix, Iy, Iz] = S.inertia_diagonal_kg_m2;
const [Xu, Yv, Nr] = S.damping.linear_planar;
const [Xuu, Yvv, Nrr] = S.damping.quadratic_planar;
const params = VehicleParameters.fromGeometry(S.length_m, S.beam_m, S.draft_m, S.mass_kg, {
  id: S.id, height: 0.34, Ix, Iy, Iz, Xu, Yv, Nr, Xuu, Yvv, Nrr,
});
const forceModels = [new AddedMassCoriolis(), new HydrodynamicDamping()];
const core = new DynamicsCore(params, forceModels, "rk4");
const env = {waterV: {x: 0, y: 0, z: 0}};
const command = {};
const stateFromNu = ([uu, vv, r]: number[]) => {
  const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);
  state.velocity = {u: uu, v: vv, w: 0};
  state.angularRate = {p: 0, q: 0, r};
  return state;
};
const stateNu = (state: any) => [state.velocity.u, state.velocity.v, state.angularRate.r];

const comparisons = START_TIMES.map((requestedTime) => {
  const startRow = nearest(requestedTime);
  const endRow = nearest(startRow.time_s + DT);
  if (Math.abs(endRow.time_s - startRow.time_s - DT) > 1e-9) throw new Error(`Samples at ${startRow.time_s} and ${endRow.time_s} are not one step apart.`);
  const nu0 = vrxNu(startRow);
  const nu1 = vrxNu(endRow);
  const rk4State = stateFromNu(nu0);
  const eulerState = stateFromNu(nu0);
  stepRK4(core, rk4State, env, command, DT, startRow.time_s);
  stepDartSemiImplicitEuler(core, eulerState, env, command, DT, startRow.time_s);
  const measured = acceleration(nu0, nu1);
  const rk4 = acceleration(nu0, stateNu(rk4State));
  const euler = acceleration(nu0, stateNu(eulerState));
  const vrxMinusRk4 = subtract(measured, rk4);
  const vrxMinusEuler = subtract(measured, euler);
  const eulerMinusRk4 = subtract(euler, rk4);
  return {
    interval_s: [startRow.time_s, endRow.time_s], nu_0: nu0, nu_1_vrx: nu1,
    acceleration_mps2: {vrx_measured: measured, node_rk4: rk4, node_dart_order_euler: euler},
    residual_mps2: {
      vrx_minus_node_rk4: vrxMinusRk4,
      vrx_minus_node_dart_order_euler: vrxMinusEuler,
      node_dart_order_euler_minus_rk4: eulerMinusRk4,
    },
    planar_residual_l2_mps2: {
      vrx_minus_node_rk4: norm2(vrxMinusRk4),
      vrx_minus_node_dart_order_euler: norm2(vrxMinusEuler),
      node_dart_order_euler_minus_rk4: norm2(eulerMinusRk4),
    },
  };
});
const summary = (key: "vrx_minus_node_rk4" | "vrx_minus_node_dart_order_euler" | "node_dart_order_euler_minus_rk4") => {
  const values = comparisons.map((item) => item.planar_residual_l2_mps2[key]);
  return {min: Math.min(...values), max: Math.max(...values), mean: values.reduce((sum, value) => sum + value, 0) / values.length, rms: Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length)};
};
const rk4Summary = summary("vrx_minus_node_rk4");
const eulerSummary = summary("vrx_minus_node_dart_order_euler");
const directionAnalysis = (key: "vrx_minus_node_rk4" | "vrx_minus_node_dart_order_euler") => {
  const surgeResidual = comparisons.map((item) => item.residual_mps2[key][0]);
  const swayResidual = comparisons.map((item) => item.residual_mps2[key][1]);
  const yawRate = comparisons.map((item) => item.nu_0[2]);
  return {
    all_sway_residuals_negative: swayResidual.every((value) => value < 0),
    surge_residual_vs_yaw_rate_pearson: pearson(surgeResidual, yawRate),
    sway_residual_vs_yaw_rate_pearson: pearson(swayResidual, yawRate),
  };
};
const report = {
  schema_version: 1,
  status: "INTEGRATOR_ISOLATION_COMPLETE",
  method: "Each VRX interval, Node RK4 step, and Node DART-order semi-implicit Euler step starts from the identical recorded VRX nu_0; all accelerations are one-step finite differences over 0.05 s.",
  integrators: {node_production: "classical RK4, order 4", diagnostic_dart_match: "semi-implicit Euler, order 1; velocity then configuration", vrx_runtime: "DART 6.13.2 World::step at max_step_size 0.05 s"},
  comparisons,
  planar_residual_summary_mps2: {vrx_minus_node_rk4: rk4Summary, vrx_minus_node_dart_order_euler: eulerSummary, node_dart_order_euler_minus_rk4: summary("node_dart_order_euler_minus_rk4")},
  direction_analysis: {
    vrx_minus_node_rk4: directionAnalysis("vrx_minus_node_rk4"),
    vrx_minus_node_dart_order_euler: directionAnalysis("vrx_minus_node_dart_order_euler"),
  },
  conclusion: eulerSummary.rms < rk4Summary.rms
    ? "Using DART-order Euler in Node reduces the VRX residual relative to Node RK4; the integrator mismatch explains a measurable portion of the systematic pattern."
    : "Using DART-order Euler in Node does not reduce the VRX residual relative to Node RK4; integrator order alone does not explain the systematic pattern.",
  gate_7_changed: false,
  protected_model_configuration_changed: false,
};
writeFileSync(outputPath, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
