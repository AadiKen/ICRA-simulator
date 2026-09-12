import assert from "node:assert/strict";
import test from "node:test";
import { compareTraces } from "./conformance.ts";
import type { TraceV2 } from "./trace-schema-v2.ts";

const trace = (simulator: TraceV2["simulator"], yaw: number): TraceV2 => ({
  schema_version: "trace-schema-v2",
  simulator,
  reset: {
    seed: 1,
    initial_state: [0, 0, 0, 0, 0, 0],
    route_ned_m: [[0, 0]],
    disturbance: {
      wind_speed_m_s: 0,
      wind_direction_deg: 0,
      current_speed_m_s: 0,
      current_direction_deg: 0,
    },
  },
  action_trace: [{ step: 0, time_s: 0, command: [0, 0, 0, 0] }],
  samples: [0, 1].map((step) => ({
    step,
    time_s: step * 0.05,
    state: [0, 0, yaw, 0, 0, 0],
    observation: Array(14)
      .fill(0)
      .concat(1 - step),
    applied_action: [0, 0, 0, 0],
  })),
});

test("yaw comparison wraps across +/-pi", () => {
  const result = compareTraces(
    trace("Gazebo Harmonic", Math.PI - 0.01),
    trace("VRX", -Math.PI + 0.01),
  );
  assert.ok(Math.abs(result.per_state_max_abs.yaw - 0.02) < 1e-12);
  assert.equal(result.first_divergence_step.yaw, 0);
});
