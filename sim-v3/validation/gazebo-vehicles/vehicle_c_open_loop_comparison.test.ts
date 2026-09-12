import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
const a = JSON.parse(
  await readFile(
    "artifacts/gazebo/vehicle-c/open-loop-trajectory-comparison-thrust-corrected.json",
    "utf8",
  ),
);
assert.equal(a.provenance.controller, "none; fixed-input open-loop");
assert.equal(a.diagnostics.actuator_lag_exercised, true);
assert.equal(a.diagnostics.straight_ahead_lateral_drift.persists_in_retained_low_thrust_campaign, false);
const wrappedBoundaryError = Math.abs(
  Math.atan2(Math.sin((-Math.PI + 0.01) - (Math.PI - 0.01)), Math.cos((-Math.PI + 0.01) - (Math.PI - 0.01))),
);
assert.ok(Math.abs(wrappedBoundaryError - 0.02) < 1e-12, "heading error must wrap across ±π");
const rotation = a.scenarios.find((scenario: any) => scenario.id === "rotation-in-place");
const allocationChirp = a.scenarios.find((scenario: any) => scenario.id === "allocation-chirp");
assert.equal(rotation.summary.relative_position_error, null);
assert.equal(rotation.summary.heading_convention_normalization.applied, "negate Gazebo task-frame yaw");
assert.ok(rotation.summary.heading_error_rad.max < 3 * Math.PI / 180);
assert.equal(allocationChirp.gazebo_capture_model_sha256, a.provenance.sdf.corrected_sha256);
assert.ok(allocationChirp.summary.heading_error_rad.max < 2 * Math.PI / 180);
assert.deepEqual(
  a.scenarios.map((x: any) => x.id),
  ["straight-ahead", "pure-lateral", "rotation-in-place", "allocation-chirp"],
);
for (const s of a.scenarios) {
  assert.equal(s.bcod_sim.length, 120);
  assert.equal(s.gazebo_harmonic.length, 120);
  assert.equal(s.error_time_series.length, 120);
  assert.equal(s.command_schedule.length, 120);
  assert.deepEqual(
    s.bcod_sim.map((x: any) => x.command),
    s.gazebo_harmonic.map((x: any) => x.command),
  );
  assert.ok(
    s.gazebo_harmonic.every(
      (x: any, i: number) =>
        Number.isFinite(x.delivered.port_azimuth_rad) &&
        Number.isFinite(x.delivered.starboard_azimuth_rad) &&
        Math.abs(x.time_s - (i + 1) * 0.05) < 1e-9 &&
        Number.isInteger(x.gazebo_iteration),
    ),
  );
  assert.equal(s.synchronization.blind_retries, false);
  assert.equal(s.synchronization.asserted_iteration_delta, 5);
}
console.log("Vehicle C open-loop paired artifact passed");
