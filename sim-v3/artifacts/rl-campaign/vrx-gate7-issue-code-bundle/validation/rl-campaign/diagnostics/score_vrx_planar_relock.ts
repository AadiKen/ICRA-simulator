import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import {
  assembleVrxExporterTrace,
  FULL_STEPS,
  PHYSICS_DT,
} from "./ports/episode-driver.ts";
import { generateNodeReference } from "./ports/generate-node-reference.ts";

const root = resolve(
  process.argv[2] ?? "artifacts/rl-campaign/vrx-gate7-planar-relock/off",
);
const samples = readFileSync(resolve(root, "raw.jsonl"), "utf8")
  .trim()
  .split(/\n+/)
  .filter(Boolean).length;
if (samples !== FULL_STEPS)
  throw new Error(`Expected ${FULL_STEPS} samples, found ${samples}`);

const model = readFileSync(resolve(root, "models/surveyor/model.sdf"), "utf8");
const required = [
  'joint name="planar_x"',
  'joint name="planar_y"',
  'joint name="planar_yaw"',
  "<xx>2.615</xx>",
  "<yy>39.224999999999994</yy>",
  "<rr>1.431297586666667</rr>",
  "<zW>365</zW>",
  "<kP>40</kP>",
  "<mQ>85</mQ>",
];
if (!required.every((item) => model.includes(item)))
  throw new Error("Relocked model configuration is incomplete");

const vrx = assembleVrxExporterTrace(
  20000,
  resolve(root, "raw.jsonl"),
  resolve(root, "trace.json"),
  samples,
);
const node = generateNodeReference(
  20000,
  resolve(root, "node-trace.json"),
  samples,
  { wind: 0, current: 0, action: 1 },
);
const schedule = JSON.parse(
  readFileSync(resolve(root, "transport.json"), "utf8"),
);
let maxAppliedForceDifferenceN = 0;
for (let step = 0; step < samples; step++) {
  for (let thruster = 0; thruster < 2; thruster++) {
    maxAppliedForceDifferenceN = Math.max(
      maxAppliedForceDifferenceN,
      Math.abs(
        schedule.transport[step][thruster].value -
          node.samples[step].applied_action[thruster] * 70,
      ),
    );
  }
}

const norm = (value: number[]) => Math.hypot(...value);
const checkpoint = (index: number) => {
  const nodeDisplacement = [
    node.samples[index].state[0] - node.samples[0].state[0],
    node.samples[index].state[1] - node.samples[0].state[1],
  ];
  const vrxDisplacement = [
    vrx.samples[index].state[0] - vrx.samples[0].state[0],
    vrx.samples[index].state[1] - vrx.samples[0].state[1],
  ];
  const nodeMagnitude = norm(nodeDisplacement);
  const vrxMagnitude = norm(vrxDisplacement);
  const direction =
    (Math.acos(
      Math.max(
        -1,
        Math.min(
          1,
          (nodeDisplacement[0] * vrxDisplacement[0] +
            nodeDisplacement[1] * vrxDisplacement[1]) /
            (nodeMagnitude * vrxMagnitude),
        ),
      ),
    ) *
      180) /
    Math.PI;
  const relative =
    norm([
      vrxDisplacement[0] - nodeDisplacement[0],
      vrxDisplacement[1] - nodeDisplacement[1],
    ]) / Math.max(nodeMagnitude, 0.5);
  return {
    checkpoint_s: index * PHYSICS_DT,
    sample_index: index,
    node_displacement_ne_m: nodeDisplacement,
    vrx_displacement_ne_m: vrxDisplacement,
    node_magnitude_m: nodeMagnitude,
    vrx_magnitude_m: vrxMagnitude,
    relative_vector_divergence: relative,
    direction_difference_deg: direction,
    pass: relative <= 0.3 && direction <= 15,
  };
};

const checkpoints = [600, 1200, 2399].map(checkpoint);
const report = {
  schema_version: 1,
  artifact_kind: "vrx-gate-7-planar-relock-diagnostic",
  status: "COMPLETE",
  production_change: false,
  seed: 20000,
  configuration: {
    open_loop: true,
    environment: "calm",
    planar_constraint: "joint chain planar_x -> planar_y -> planar_yaw",
    corrected_added_mass: {
      xx: 2.615,
      yy: 39.224999999999994,
      rr: 1.431297586666667,
    },
    accepted_out_of_plane_damping: { zW: 365, kP: 40, mQ: 85 },
  },
  command_identity: {
    max_applied_force_difference_n: maxAppliedForceDifferenceN,
  },
  tolerance: { relative_vector: 0.3, direction_deg: 15 },
  checkpoints,
  classification: checkpoints.every((item) => item.pass)
    ? "PLANAR_RELOCK_NEAR_PARITY"
    : "PLANAR_RELOCK_STILL_LARGE_DIVERGENCE",
  conclusion: checkpoints.every((item) => item.pass)
    ? "Out-of-plane coupling explains the unconstrained divergence."
    : "Out-of-plane coupling does not explain the dominant divergence; the unresolved planar yaw-rate-correlated residual remains the leading suspect.",
};
const out = resolve(dirname(root), "result.json");
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
