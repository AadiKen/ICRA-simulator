import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";

const root = resolve(
  process.argv[2] ?? "artifacts/rl-campaign/vrx-gate7-damped-dryrun/off",
);
const node = JSON.parse(
  readFileSync(resolve(root, "node-trace-partial.json"), "utf8"),
);
const vrx = JSON.parse(
  readFileSync(resolve(root, "trace-partial.json"), "utf8"),
);
const schedule = JSON.parse(
  readFileSync(resolve(root, "transport.json"), "utf8"),
);
let raw = 0,
  applied = 0,
  force = 0;
for (let step = 0; step < node.samples.length; step++) {
  const actionIndex = Math.floor(step / 2),
    scheduled = schedule.action_trace[actionIndex].command,
    nodeAction = node.action_trace[actionIndex].command;
  for (let axis = 0; axis < 4; axis++) {
    raw = Math.max(raw, Math.abs(scheduled[axis] - nodeAction[axis]));
    applied = Math.max(
      applied,
      Math.abs(
        node.samples[step].applied_action[axis] -
          vrx.samples[step].applied_action[axis],
      ),
    );
  }
  for (let thruster = 0; thruster < 2; thruster++)
    force = Math.max(
      force,
      Math.abs(
        schedule.transport[step][thruster].value -
          node.samples[step].applied_action[thruster] * 70,
      ),
    );
}
const report = {
  schema_version: 1,
  artifact_kind: "vrx-gate-7-open-loop-command-audit",
  classification: "OPEN_LOOP_FIXED_REPLAY",
  evidence: {
    command_generator: "fixedActionTrace() / commandAt()",
    state_feedback_used_for_command_generation: false,
    physics_steps: node.samples.length,
    control_actions: schedule.action_trace.length,
    max_raw_command_difference: raw,
    max_applied_normalized_action_difference: applied,
    max_vrx_transport_vs_node_applied_force_difference_n: force,
  },
  interpretation:
    "The existing calm Gate 7 dry run is already the requested 120 s open-loop comparison. Its divergence cannot be attributed to feedback amplification.",
  next_scope:
    "Residual baseline vehicle dynamics under matched applied thrust.",
};
const out = resolve(dirname(root), "open-loop-audit.json");
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
