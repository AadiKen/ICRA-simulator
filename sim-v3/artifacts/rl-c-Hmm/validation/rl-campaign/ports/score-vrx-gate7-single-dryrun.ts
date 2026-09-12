import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import {
  assembleVrxExporterTrace,
  FULL_STEPS,
  PHYSICS_DT,
} from "./episode-driver.ts";
import { generateNodeReference } from "./generate-node-reference.ts";
import { compareTraces } from "./conformance.ts";

const seed = 20000;
const [
  rootArg = "artifacts/rl-campaign/vrx-gate7-single-dryrun",
  zWArg = "0",
  kPArg = "0",
  mQArg = "0",
] = process.argv.slice(2);
const root = resolve(rootArg);
const expectedDamping = {
  zW: Number(zWArg),
  kP: Number(kPArg),
  mQ: Number(mQArg),
};
const onDir = resolve(root, "on"),
  offDir = resolve(root, "off");
const lineCount = (path: string) =>
  readFileSync(path, "utf8").trim().split(/\n+/).filter(Boolean).length;
const onSamples = lineCount(resolve(onDir, "raw.jsonl"));
const offSamples = lineCount(resolve(offDir, "raw.jsonl"));
const usableSamples = Math.min(onSamples, offSamples);
if (usableSamples <= 600)
  throw new Error(
    `Need sample 600 for the 30 s checkpoint; only ${usableSamples} paired samples exist.`,
  );

const model = readFileSync(resolve(onDir, "models/surveyor/model.sdf"), "utf8");
const required = [
  "<xx>2.615</xx>",
  "<yy>39.224999999999994</yy>",
  "<rr>1.431297586666667</rr>",
  "<xDotU>0</xDotU>",
  "<yDotV>0</yDotV>",
  "<nDotR>0</nDotR>",
  `<zW>${expectedDamping.zW}</zW>`,
  `<kP>${expectedDamping.kP}</kP>`,
  `<mQ>${expectedDamping.mQ}</mQ>`,
];
const modelChecks = {
  generated_added_mass_and_expected_damping: required.every((value) =>
    model.includes(value),
  ),
  planar_lock_absent: !/(planar_x|planar_y|planar_yaw)/.test(model),
};
if (!Object.values(modelChecks).every(Boolean))
  throw new Error(
    "Generated model does not satisfy the dry-run configuration contract.",
  );

const vrxOn = assembleVrxExporterTrace(
  seed,
  resolve(onDir, "raw.jsonl"),
  resolve(onDir, "trace-partial.json"),
  usableSamples,
);
const vrxOff = assembleVrxExporterTrace(
  seed,
  resolve(offDir, "raw.jsonl"),
  resolve(offDir, "trace-partial.json"),
  usableSamples,
);
const nodeOn = generateNodeReference(
  seed,
  resolve(onDir, "node-trace-partial.json"),
  usableSamples,
  { wind: 1, current: 1, action: 1 },
);
const nodeOff = generateNodeReference(
  seed,
  resolve(offDir, "node-trace-partial.json"),
  usableSamples,
  { wind: 0, current: 0, action: 1 },
);
const kinematic = compareTraces(nodeOn, vrxOn);
const calmKinematic = compareTraces(nodeOff, vrxOff);
const wrapAngle = (angle: number) =>
  Math.atan2(Math.sin(angle), Math.cos(angle));
const warmupSummary = (reference: typeof nodeOn, candidate: typeof vrxOn) =>
  Object.fromEntries(
    ["N", "E", "yaw_wrapped", "u", "v", "r"].map((name, axis) => {
      const values = candidate.samples.slice(10).map((sample, index) => {
        const difference =
          sample.state[axis] - reference.samples[index + 10].state[axis];
        return Math.abs(axis === 2 ? wrapAngle(difference) : difference);
      });
      return [
        name,
        {
          max_abs: Math.max(...values),
          mean_abs:
            values.reduce((sum, value) => sum + value, 0) / values.length,
        },
      ];
    }),
  );
const velocityWarmup = Object.fromEntries(
  ([3, 4, 5] as const).map((axis, index) => {
    const name = ["u", "v", "r"][index],
      errors = vrxOn.samples
        .slice(10)
        .map((sample, i) =>
          Math.abs(sample.state[axis] - nodeOn.samples[i + 10].state[axis]),
        );
    return [
      name,
      {
        max_abs: Math.max(...errors),
        mean_abs: errors.reduce((sum, value) => sum + value, 0) / errors.length,
      },
    ];
  }),
);
const norm = (v: number[]) => Math.hypot(...v);
const environmentAt = (index: number) => {
  const node = [
    nodeOn.samples[index].state[0] - nodeOff.samples[index].state[0],
    nodeOn.samples[index].state[1] - nodeOff.samples[index].state[1],
  ];
  const vrx = [
    vrxOn.samples[index].state[0] - vrxOff.samples[index].state[0],
    vrxOn.samples[index].state[1] - vrxOff.samples[index].state[1],
  ];
  const nodeNorm = norm(node),
    vrxNorm = norm(vrx),
    den = Math.max(nodeNorm, 0.5),
    dot = node[0] * vrx[0] + node[1] * vrx[1];
  const direction =
    nodeNorm && vrxNorm
      ? (Math.acos(Math.max(-1, Math.min(1, dot / (nodeNorm * vrxNorm)))) *
          180) /
        Math.PI
      : 180;
  const relative = norm([vrx[0] - node[0], vrx[1] - node[1]]) / den;
  const sign = node.every(
    (value, i) =>
      Math.abs(value) < 0.5 || Math.sign(value) === Math.sign(vrx[i]),
  );
  return {
    checkpoint_s: index * PHYSICS_DT,
    sample_index: index,
    node_environment_displacement_ne_m: node,
    vrx_environment_displacement_ne_m: vrx,
    node_magnitude_m: nodeNorm,
    vrx_magnitude_m: vrxNorm,
    magnitude_divergence: Math.abs(vrxNorm - nodeNorm) / den,
    relative_vector_divergence: relative,
    direction_difference_deg: direction,
    component_sign_consistent: sign,
    pass: relative <= 0.3 && direction <= 15 && sign,
  };
};
const calmMotionAt = (index: number) => {
  const node = [
    nodeOff.samples[index].state[0] - nodeOff.samples[0].state[0],
    nodeOff.samples[index].state[1] - nodeOff.samples[0].state[1],
  ];
  const vrx = [
    vrxOff.samples[index].state[0] - vrxOff.samples[0].state[0],
    vrxOff.samples[index].state[1] - vrxOff.samples[0].state[1],
  ];
  const nodeNorm = norm(node),
    vrxNorm = norm(vrx),
    denominator = Math.max(nodeNorm, 0.5);
  const direction =
    nodeNorm && vrxNorm
      ? (Math.acos(
          Math.max(
            -1,
            Math.min(
              1,
              (node[0] * vrx[0] + node[1] * vrx[1]) / (nodeNorm * vrxNorm),
            ),
          ),
        ) *
          180) /
        Math.PI
      : 180;
  const relative = norm([vrx[0] - node[0], vrx[1] - node[1]]) / denominator;
  return {
    checkpoint_s: index * PHYSICS_DT,
    sample_index: index,
    node_displacement_ne_m: node,
    vrx_displacement_ne_m: vrx,
    node_magnitude_m: nodeNorm,
    vrx_magnitude_m: vrxNorm,
    relative_vector_divergence: relative,
    direction_difference_deg: direction,
    pass: relative <= 0.3 && direction <= 15,
  };
};
const prior = {
  campaign_maximum_relative_vector_divergence: 5.74690862561023,
  campaign_maximum_direction_difference_deg: 77.6830711959181,
  seed_20000_at_30_s: {
    relative_vector_divergence: 0.7553254839370167,
    direction_difference_deg: 36.553325595791584,
  },
};
const checkpoint30 = environmentAt(600);
const checkpoints = [
  { checkpoint_s: 30, sample_index: 600 },
  { checkpoint_s: 60, sample_index: 1200 },
  { checkpoint_s: 119.95, sample_index: 2399 },
];
const available = checkpoints
  .filter(({ sample_index }) => sample_index < usableSamples)
  .map(({ sample_index }) => environmentAt(sample_index));
const unavailable = checkpoints.filter(
  ({ sample_index }) => sample_index >= usableSamples,
);
const singleSeedPass =
  available.length === checkpoints.length && available.every((x) => x.pass);
const report = {
  schema_version: 1,
  artifact_kind: "vrx-gate-7-single-episode-unconstrained-dryrun",
  status:
    onSamples === FULL_STEPS && offSamples === FULL_STEPS
      ? "COMPLETE"
      : "INCOMPLETE_RUNTIME_ABORTED",
  seed,
  execution: {
    requested_samples: FULL_STEPS,
    disturbance_on_samples: onSamples,
    disturbance_off_samples: offSamples,
    paired_samples_scored: usableSamples,
    last_scored_time_s: (usableSamples - 1) * PHYSICS_DT,
    runtime_failure:
      onSamples < FULL_STEPS
        ? {
            simulator: "VRX disturbance-on",
            time_s: (onSamples - 1) * PHYSICS_DT,
            error:
              'ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact && aabbBound < dMaxIntExact" failed in collide() [collision_space.cpp:460]',
          }
        : null,
  },
  generation: {
    path: "prepare-vrx-episode.ts -> renderSurveyorVrxModel()",
    model_checks: modelChecks,
    fluid_added_mass: { xx: 2.615, yy: 39.225, rr: 1.431297586666667 },
    simple_hydrodynamics_added_mass: { xDotU: 0, yDotV: 0, nDotR: 0 },
    simple_hydrodynamics_out_of_plane_damping: expectedDamping,
  },
  gate_7_tolerance: {
    kinematic_abs: 1e-4,
    environment_vector_relative: 0.3,
    environment_direction_deg: 15,
    component_sign_floor_m: 0.5,
  },
  kinematic_partial: {
    ...kinematic,
    velocity_warmup_excluded: velocityWarmup,
    warmup_samples_excluded: 10,
  },
  calm_kinematic: {
    gate_metric: calmKinematic,
    warmup_excluded: warmupSummary(nodeOff, vrxOff),
    checkpoint_candidate_minus_reference: [600, 1200, 2399]
      .filter((index) => index < usableSamples)
      .map((index) => ({
        checkpoint_s: index * PHYSICS_DT,
        error: Object.fromEntries(
          ["N", "E", "yaw_wrapped", "u", "v", "r"].map((name, axis) => {
            const difference =
              vrxOff.samples[index].state[axis] -
              nodeOff.samples[index].state[axis];
            return [name, axis === 2 ? wrapAngle(difference) : difference];
          }),
        ),
      })),
    pass: Object.values(calmKinematic.per_state_max_abs).every(
      (value) => value <= 1e-4,
    ),
  },
  calm_cross_simulator_conformance: {
    metric:
      "Each simulator's displacement from its own initial position, evaluated with Gate 7's 30% vector and 15 degree direction bounds.",
    checkpoints: [600, 1200, 2399]
      .filter((index) => index < usableSamples)
      .map(calmMotionAt),
    tolerance: { relative_vector: 0.3, direction_deg: 15 },
    note: "The 1e-4 kinematic_abs value is hard-coded in finalize-vrx-gate7.ts and is not defined by the frozen task contract as a cross-simulator acceptance threshold.",
  },
  environment_response: {
    available,
    unavailable,
    prior_gate_7_comparison: {
      ...prior,
      seed_20000_at_30_s_change: {
        relative_vector_divergence:
          checkpoint30.relative_vector_divergence -
          prior.seed_20000_at_30_s.relative_vector_divergence,
        direction_difference_deg:
          checkpoint30.direction_difference_deg -
          prior.seed_20000_at_30_s.direction_difference_deg,
      },
    },
  },
  conclusion: {
    single_seed_result: unavailable.length
      ? "NOT_SCORED"
      : singleSeedPass
        ? "PASS"
        : "FAIL",
    reason: unavailable.length
      ? "The episode did not reach every preregistered checkpoint."
      : singleSeedPass
        ? "All environmental checkpoints pass for this single seed."
        : "The episode completed, but at least one environmental checkpoint fails tolerance.",
  },
  comparison_note:
    "The protocol's 40-70% band is a classical-policy success-rate target, not the Gate 7 per-checkpoint conformance tolerance; one deterministic seed cannot be classified against that band.",
};
const out = resolve(root, "result.json");
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
