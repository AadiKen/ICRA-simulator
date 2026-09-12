import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";

const root = resolve("artifacts/rl-campaign/vrx-thrust-step"),
  rows = readFileSync(resolve(root, "raw.jsonl"), "utf8")
    .trim()
    .split(/\n+/)
    .map((line) => JSON.parse(line));
const rho = 1025,
  coefficient = 0.006919495468257212,
  diameter = 0.1;
const angularVelocityToForce = (value: number | null) =>
  value === null
    ? null
    : rho * coefficient * diameter ** 4 * Math.abs(value) * value;
const samples = rows.map((row: any, index: number) => ({
  time_s: index * 0.05,
  node_actuator_force_n: row.thruster_newtons[0],
  vrx_feedback_force_n: angularVelocityToForce(
    row.thruster_feedback_angvel_rad_s[0],
  ),
}));
const comparisons = [0, 1, 2].map((lag) => {
  const pairs = samples
    .slice(0, samples.length - lag)
    .map((sample, index) => ({
      node: sample.node_actuator_force_n,
      vrx: samples[index + lag].vrx_feedback_force_n,
    }))
    .filter((pair): pair is { node: number; vrx: number } => pair.vrx !== null);
  const errors = pairs.map((pair) => Math.abs(pair.node - pair.vrx));
  return {
    vrx_feedback_lag_samples: lag,
    pairs: pairs.length,
    max_abs_error_n: Math.max(...errors),
    mean_abs_error_n:
      errors.reduce((sum, value) => sum + value, 0) / errors.length,
  };
});
const best = comparisons.reduce((a, b) =>
  a.mean_abs_error_n <= b.mean_abs_error_n ? a : b,
);
const target = 17.5,
  tau = 0.35;
const expectedAt = (time: number) =>
  target * (1 - Math.exp(-(time + 0.05) / tau));
const checkpoints = [0, 0.05, 0.1, 0.2, 0.35, 0.5, 0.7, 1, 2, 3.95].map(
  (time) => {
    const index = Math.round(time / 0.05),
      sample = samples[index],
      feedbackIndex = index + best.vrx_feedback_lag_samples;
    return {
      time_s: time,
      analytic_node_n: expectedAt(time),
      node_n: sample.node_actuator_force_n,
      vrx_n:
        feedbackIndex < samples.length
          ? samples[feedbackIndex].vrx_feedback_force_n
          : null,
    };
  },
);
const report = {
  schema_version: 1,
  artifact_kind: "vrx-node-static-thrust-step",
  configuration: {
    environment: "zero current, zero wind",
    initial_velocity_m_s: [0, 0, 0],
    command_normalized_each: 0.25,
    target_force_n_each: target,
    actuator_time_constant_s: tau,
    sample_period_s: 0.05,
  },
  measurement: {
    node: "FrozenActuatorBank state using the same first-order law as Node production effectors",
    vrx: "Thruster /ang_vel feedback inverted with configured rho, thrust coefficient, and diameter",
    gazebo_force_path:
      "Force-command mode applies desiredThrust directly; velocity_control changes only joint angular-velocity control",
  },
  alignment: best,
  all_alignments: comparisons,
  checkpoints,
  conclusion: best.max_abs_error_n < 1e-9 ? "MATCH" : "MISMATCH",
};
mkdirSync(root, { recursive: true });
writeFileSync(
  resolve(root, "result.json"),
  JSON.stringify(report, null, 2) + "\n",
);
console.log(JSON.stringify(report, null, 2));
