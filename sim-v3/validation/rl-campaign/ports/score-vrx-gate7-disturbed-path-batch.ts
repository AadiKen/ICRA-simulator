import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { pathMetrics } from "./score-vrx-gate7-calm-path-batch.ts";
import { frozenReset } from "./episode-driver.ts";
import type { TraceV2 } from "./trace-schema-v2.ts";

const root = resolve("artifacts/rl-campaign/vrx-gate7-full"),
  out = resolve(root, "disturbed-path-result.json");
const read = (path: string) =>
  JSON.parse(readFileSync(path, "utf8")) as TraceV2;
const episodes = Array.from({ length: 20 }, (_, i) => 20000 + i).map((seed) => {
  const on = resolve(root, "on", `seed-${seed}`),
    off = resolve(root, "off", `seed-${seed}`);
  const nodeOn = read(
    resolve("artifacts/rl-campaign/gate-7/reference", `node-${seed}.json`),
  );
  const nodeOff = read(resolve(off, "node-trace.json")),
    vrxOn = read(resolve(on, "trace.json")),
    vrxOff = read(resolve(off, "trace.json"));
  const nodePath = nodeOn.samples.map((x, i) => [
    x.state[0] - nodeOff.samples[i].state[0],
    x.state[1] - nodeOff.samples[i].state[1],
  ]);
  const vrxPath = vrxOn.samples.map((x, i) => [
    x.state[0] - vrxOff.samples[i].state[0],
    x.state[1] - vrxOff.samples[i].state[1],
  ]);
  return {
    seed,
    disturbance: frozenReset(seed).disturbance,
    ...pathMetrics(nodePath, vrxPath),
  };
});
const mean = (pick: (x: (typeof episodes)[number]) => number) =>
  episodes.reduce((s, x) => s + pick(x), 0) / episodes.length;
const worst = episodes.reduce((a, b) =>
  a.nearest_mean_over_path_length > b.nearest_mean_over_path_length ? a : b,
);
const report = {
  schema_version: 1,
  artifact_kind: "vrx-gate-7-disturbed-path-batch",
  metric_choice:
    "Nearest distance from every VRX environmental-response sample to the full Node environmental-response path; response paths are environment-on minus calm within each simulator. Synchronous along/cross-track errors are retained separately.",
  telemetry:
    "All four inputs per seed are corrected TraceV2 artifacts produced by assembleVrxExporterTrace: quaternion-derived yaw, pose-derived u/v/r with wrapped yaw differences, and no fixed startup exclusion.",
  episodes,
  aggregate: {
    mean_nearest_error_m: mean((x) => x.nearest_node_path_error_m.mean),
    mean_nearest_rms_m: mean((x) => x.nearest_node_path_error_m.rms),
    maximum_nearest_error_m: Math.max(
      ...episodes.map((x) => x.nearest_node_path_error_m.max),
    ),
    mean_node_path_length_m: mean((x) => x.node_path_length_m),
    mean_nearest_over_path_length: mean((x) => x.nearest_mean_over_path_length),
    worst_seed_by_normalized_mean: {
      seed: worst.seed,
      value: worst.nearest_mean_over_path_length,
    },
    mean_synchronous_position_rmse_m: mean(
      (x) => x.synchronous_position_error_m.rms,
    ),
    mean_along_track_rmse_m: mean((x) => x.synchronous_along_track_error_m.rms),
    mean_cross_track_rmse_m: mean((x) => x.synchronous_cross_track_error_m.rms),
  },
};
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, JSON.stringify(report, null, 2) + "\n");
console.log(JSON.stringify(report, null, 2));
