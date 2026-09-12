import assert from "node:assert/strict";
import {execFileSync} from "node:child_process";
import {mkdtempSync, readFileSync} from "node:fs";
import {tmpdir} from "node:os";
import {join} from "node:path";

const output = join(mkdtempSync(join(tmpdir(), "vrx-integrator-isolation-")), "result.json");
execFileSync(process.execPath, [
  "--experimental-strip-types",
  "validation/rl-campaign/isolate_vrx_euler_vs_rk4.ts",
  "artifacts/rl-campaign/vrx-mass-bias-check/velocity-telemetry.csv",
  output,
], {stdio: "ignore"});
const report = JSON.parse(readFileSync(output, "utf8"));
assert.equal(report.comparisons.length, 11);
assert(report.comparisons.every((item: any) => Math.abs(item.interval_s[1] - item.interval_s[0] - 0.05) < 1e-9));
assert(report.comparisons.every((item: any) => item.nu_0.length === 3));
assert.equal(report.direction_analysis.vrx_minus_node_dart_order_euler.all_sway_residuals_negative, true);
assert(report.planar_residual_summary_mps2.vrx_minus_node_dart_order_euler.rms > report.planar_residual_summary_mps2.vrx_minus_node_rk4.rms);
assert.match(report.conclusion, /does not reduce/);
assert.equal(report.gate_7_changed, false);
assert.equal(report.protected_model_configuration_changed, false);
console.log("VRX Euler-vs-RK4 isolation test passed.");
