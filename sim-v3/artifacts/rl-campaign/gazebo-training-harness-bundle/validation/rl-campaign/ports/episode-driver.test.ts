import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { frozenReset, poseDerivedPlanarTwist } from "./episode-driver.ts";
import { loadFrozenTaskContract } from "./frozen-task-contract.ts";

const { task, contentSha256 } = loadFrozenTaskContract();
assert.equal(
  contentSha256,
  "2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36",
);
const reset = frozenReset(30000);
assert.deepEqual(
  reset.initial_state,
  [9999.825020389631, 10000.692335725296, -0.14518976353579438, 0, 0, 0],
);
assert.deepEqual(reset.route_ned_m, [
  [10026.13945536919, 10007.331657956162],
  [10040.895789930712, 10032.04697586398],
  [10063.951277280814, 10023.869250047348],
]);
assert.equal(reset.disturbance.current_speed_m_s, 1.3151809121378515);
assert.equal(reset.disturbance.wind_speed_m_s, 3.2291639726812593);
assert.deepEqual(task.reset_randomization.route_relative_m, [
  [27.139087825103655, 0],
  [47.49340369393139, 20.35431586882774],
  [67.84771956275914, 6.784771956275914],
]);
const generated = JSON.parse(
  readFileSync("artifacts/rl-campaign/vrx-conformance.json", "utf8"),
);
assert.equal(generated.task_contract.content_sha256, contentSha256);
const wrapRows = [
  { state: [0, 0, Math.PI - 0.01, 999, 999, 999] },
  { state: [0.05, 0, -Math.PI + 0.01, 999, 999, 999] },
];
const wrapTwist = poseDerivedPlanarTwist(wrapRows, 0.05);
assert(
  Math.abs(wrapTwist[1][2] - 0.4) < 1e-12,
  "yaw differentiation must unwrap +/-pi",
);
assert(wrapTwist.every((sample) => sample.every(Number.isFinite)));
console.log("Frozen external reset and generated contract provenance passed.");
