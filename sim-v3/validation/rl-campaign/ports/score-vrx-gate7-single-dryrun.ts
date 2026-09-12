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
const rawOn = readFileSync(resolve(onDir, "raw.jsonl"), "utf8")
  .trim()
  .split(/\n+/)
  .map(JSON.parse);
const rawOff = readFileSync(resolve(offDir, "raw.jsonl"), "utf8")
  .trim()
  .split(/\n+/)
  .map(JSON.parse);
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
      const values = candidate.samples.map((sample, index) => {
        const difference =
          sample.state[axis] - reference.samples[index].state[axis];
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
      errors = vrxOn.samples.map((sample, i) =>
        Math.abs(sample.state[axis] - nodeOn.samples[i].state[axis]),
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
const surgeForceBoundN = 140,
  yawMomentBoundNm = 46.2;
const surgeRateBound =
  (-6 + Math.sqrt(6 * 6 + 4 * 18 * surgeForceBoundN)) / (2 * 18);
const yawRateBound =
  (-8 + Math.sqrt(8 * 8 + 4 * 12 * yawMomentBoundNm)) / (2 * 12);
const plausibility = (rows: any[]) => {
  const violations = rows
    .map((row, index) => ({
      index,
      time_s: row.time_s,
      horizontal_speed_mps: Math.hypot(row.state[3], row.state[4]),
      yaw_rate_rad_s: row.state[5],
    }))
    .filter(
      (row) =>
        row.horizontal_speed_mps > surgeRateBound ||
        Math.abs(row.yaw_rate_rad_s) > yawRateBound,
    );
  const groups: any[] = [];
  for (const row of violations) {
    const last = groups.at(-1);
    if (!last || row.index !== last.end_index + 1)
      groups.push({
        start_index: row.index,
        end_index: row.index,
        start_time_s: row.time_s,
        end_time_s: row.time_s,
        samples: 1,
      });
    else {
      last.end_index = row.index;
      last.end_time_s = row.time_s;
      last.samples++;
    }
  }
  return {
    samples: rows.length,
    violation_count: violations.length,
    groups,
    max_horizontal_speed_mps: Math.max(
      ...rows.map((row) => Math.hypot(row.state[3], row.state[4])),
    ),
    max_abs_yaw_rate_rad_s: Math.max(
      ...rows.map((row) => Math.abs(row.state[5])),
    ),
  };
};
const rms = (values: number[]) =>
  Math.sqrt(
    values.reduce((sum, value) => sum + value * value, 0) / values.length,
  );
const pointSegmentDistance = (p: number[], a: number[], b: number[]) => {
  const dx = b[0] - a[0],
    dy = b[1] - a[1],
    den = dx * dx + dy * dy,
    t = den
      ? Math.max(
          0,
          Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den),
        )
      : 0;
  return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
};
const pathMetrics = (
  nodePath: number[][],
  vrxPath: number[][],
  purpose: string,
) => {
  const synchronous = vrxPath.map((point, index) =>
    Math.hypot(point[0] - nodePath[index][0], point[1] - nodePath[index][1]),
  );
  const cross: number[] = [],
    along: number[] = [];
  for (let i = 0; i < vrxPath.length; i++) {
    const a = nodePath[Math.max(0, i - 1)],
      b = nodePath[Math.min(nodePath.length - 1, i + 1)],
      length = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1,
      tn = (b[0] - a[0]) / length,
      te = (b[1] - a[1]) / length,
      dn = vrxPath[i][0] - nodePath[i][0],
      de = vrxPath[i][1] - nodePath[i][1];
    along.push(dn * tn + de * te);
    cross.push(-dn * te + de * tn);
  }
  const nearest = vrxPath.map((point) => {
    let best = Infinity;
    for (let i = 1; i < nodePath.length; i++)
      best = Math.min(
        best,
        pointSegmentDistance(point, nodePath[i - 1], nodePath[i]),
      );
    return best;
  });
  const nodeLength = nodePath
    .slice(1)
    .reduce(
      (sum, point, index) =>
        sum +
        Math.hypot(
          point[0] - nodePath[index][0],
          point[1] - nodePath[index][1],
        ),
      0,
    );
  return {
    metric_choice:
      "Nearest distance from every VRX sample to the full Node path; this measures loop geometry without treating phase around the loop as radial divergence. Synchronous along/cross-track errors are retained separately.",
    purpose,
    samples: vrxPath.length,
    node_path_length_m: nodeLength,
    nearest_node_path_error_m: {
      mean: nearest.reduce((a, b) => a + b, 0) / nearest.length,
      rms: rms(nearest),
      max: Math.max(...nearest),
    },
    synchronous_position_error_m: {
      mean: synchronous.reduce((a, b) => a + b, 0) / synchronous.length,
      rms: rms(synchronous),
      max: Math.max(...synchronous),
    },
    synchronous_along_track_error_m: {
      rms: rms(along),
      max_abs: Math.max(...along.map(Math.abs)),
    },
    synchronous_cross_track_error_m: {
      rms: rms(cross),
      max_abs: Math.max(...cross.map(Math.abs)),
    },
  };
};
const calmPathMetrics = () =>
  pathMetrics(
    nodeOff.samples.map((sample) => sample.state.slice(0, 2)),
    vrxOff.samples.map((sample) => sample.state.slice(0, 2)),
    "Calm absolute trajectory",
  );
const disturbedPathMetrics = () =>
  pathMetrics(
    nodeOn.samples.map((sample, index) => [
      sample.state[0] - nodeOff.samples[index].state[0],
      sample.state[1] - nodeOff.samples[index].state[1],
    ]),
    vrxOn.samples.map((sample, index) => [
      sample.state[0] - vrxOff.samples[index].state[0],
      sample.state[1] - vrxOff.samples[index].state[1],
    ]),
    "Environmental response trajectory (disturbed minus calm within each simulator)",
  );
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
  telemetry_plausibility: {
    derived_bounds: {
      surge_force_bound_n: surgeForceBoundN,
      surge_damping_equation: "18*u^2 + 6*u = 140",
      horizontal_speed_bound_mps: surgeRateBound,
      yaw_moment_bound_nm: yawMomentBoundNm,
      yaw_damping_equation: "12*r^2 + 8*r = 46.2",
      yaw_rate_bound_rad_s: yawRateBound,
    },
    raw_odometry_twist: { off: plausibility(rawOff), on: plausibility(rawOn) },
    cleaned_pose_derived_twist: {
      off: plausibility(
        vrxOff.samples.map((sample: any, index: number) => ({
          time_s: index * PHYSICS_DT,
          state: sample.state,
        })),
      ),
      on: plausibility(
        vrxOn.samples.map((sample: any, index: number) => ({
          time_s: index * PHYSICS_DT,
          state: sample.state,
        })),
      ),
    },
    identified_mechanism:
      "Gazebo OdometryPublisher's ten-sample twist estimator differentiates the spawn teleport and wrapped yaw angles. The initial origin-to-reset discontinuity contaminates samples 0-9; each +/-pi yaw crossing contributes approximately 2*pi/(10*0.05)=12.566 rad/s for ten samples. Quaternion pose and IMU rates remain physical.",
    correction:
      "Planar u/v/r are reconstructed from consecutive position and quaternion-derived yaw samples, using a wrapped angular difference. No fixed sample exclusion is used.",
  },
  gate_7_tolerance: {
    kinematic_abs: 1e-4,
    environment_vector_relative: 0.3,
    environment_direction_deg: 15,
    component_sign_floor_m: 0.5,
  },
  kinematic_partial: {
    ...kinematic,
    velocity_all_samples: velocityWarmup,
    excluded_samples: 0,
  },
  calm_kinematic: {
    gate_metric: calmKinematic,
    all_samples: warmupSummary(nodeOff, vrxOff),
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
    note: "Superseded for calm looping trajectories: displacement from the initial point is phase-sensitive and is retained only for historical comparison.",
  },
  calm_path_conformance: calmPathMetrics(),
  environment_response: {
    available,
    unavailable,
    path_conformance: disturbedPathMetrics(),
    tolerance_review: {
      existing_tolerance_applicable_to_path_metric: false,
      existing_definition:
        "At each checkpoint, relative difference between environmental displacement vectors <= 30%, direction difference <= 15 degrees, with component-sign consistency.",
      review_question:
        "Define a path-metric acceptance bound before assigning pass/fail. It must specify the normalization/reference scale for nearest-path, synchronous along-track, and synchronous cross-track errors (for example, reference path length or another preregistered physical scale). No equivalent percentage or angle threshold is inferred here.",
      decision: "UNASSIGNED_PENDING_REVIEW",
    },
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
    "The calm path metric is authoritative for this looping trajectory. Disturbed checkpoint divergence decreases across this run (67.76% to 66.53% to 64.03%; direction 42.28 to 41.58 to 39.80 deg) and must not be described as growing unbounded.",
};
const out = resolve(root, "result.json");
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
