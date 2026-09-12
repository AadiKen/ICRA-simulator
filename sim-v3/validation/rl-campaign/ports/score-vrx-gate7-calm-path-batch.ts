import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import type { TraceV2 } from "./trace-schema-v2.ts";

const root = resolve("artifacts/rl-campaign/vrx-gate7-full");
const out = resolve(root, "calm-path-result.json");
const read = (path: string) =>
  JSON.parse(readFileSync(path, "utf8")) as TraceV2;
const rms = (xs: number[]) =>
  Math.sqrt(xs.reduce((s, x) => s + x * x, 0) / xs.length);
const segmentDistance = (p: number[], a: number[], b: number[]) => {
  const dx = b[0] - a[0],
    dy = b[1] - a[1],
    den = dx * dx + dy * dy;
  const t = den
    ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den))
    : 0;
  return Math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy));
};
export const pathMetrics = (nodePath: number[][], vrxPath: number[][]) => {
  const synchronous = vrxPath.map((p, i) =>
    Math.hypot(p[0] - nodePath[i][0], p[1] - nodePath[i][1]),
  );
  const along: number[] = [],
    cross: number[] = [];
  for (let i = 0; i < vrxPath.length; i++) {
    const a = nodePath[Math.max(0, i - 1)],
      b = nodePath[Math.min(nodePath.length - 1, i + 1)];
    const length = Math.hypot(b[0] - a[0], b[1] - a[1]) || 1,
      tn = (b[0] - a[0]) / length,
      te = (b[1] - a[1]) / length;
    const dn = vrxPath[i][0] - nodePath[i][0],
      de = vrxPath[i][1] - nodePath[i][1];
    along.push(dn * tn + de * te);
    cross.push(-dn * te + de * tn);
  }
  const nearest = vrxPath.map((p) => {
    let best = Infinity;
    for (let i = 1; i < nodePath.length; i++)
      best = Math.min(best, segmentDistance(p, nodePath[i - 1], nodePath[i]));
    return best;
  });
  const nodeLength = nodePath
    .slice(1)
    .reduce(
      (s, p, i) => s + Math.hypot(p[0] - nodePath[i][0], p[1] - nodePath[i][1]),
      0,
    );
  return {
    node_path_length_m: nodeLength,
    nearest_node_path_error_m: {
      mean: nearest.reduce((a, b) => a + b, 0) / nearest.length,
      rms: rms(nearest),
      max: Math.max(...nearest),
    },
    nearest_mean_over_path_length:
      nearest.reduce((a, b) => a + b, 0) / nearest.length / nodeLength,
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
const episodes = Array.from({ length: 20 }, (_, i) => 20000 + i).map((seed) => {
  const dir = resolve(root, "off", `seed-${seed}`),
    node = read(resolve(dir, "node-trace.json")),
    vrx = read(resolve(dir, "trace.json"));
  return {
    seed,
    ...pathMetrics(
      node.samples.map((x) => x.state.slice(0, 2)),
      vrx.samples.map((x) => x.state.slice(0, 2)),
    ),
  };
});
const mean = (pick: (x: (typeof episodes)[number]) => number) =>
  episodes.reduce((s, x) => s + pick(x), 0) / episodes.length;
const report = {
  schema_version: 1,
  artifact_kind: "vrx-gate-7-calm-path-batch",
  metric_choice:
    "Nearest distance from every VRX sample to the full Node path; synchronous along/cross-track errors retained separately.",
  telemetry:
    "Both calm traces are the same corrected TraceV2 inputs used by the batch finalizer. VRX trace assembly derives u/v/r from quaternion pose with wrapped yaw differences and no fixed startup exclusion.",
  episodes,
  aggregate: {
    mean_nearest_error_m: mean((x) => x.nearest_node_path_error_m.mean),
    mean_nearest_rms_m: mean((x) => x.nearest_node_path_error_m.rms),
    maximum_nearest_error_m: Math.max(
      ...episodes.map((x) => x.nearest_node_path_error_m.max),
    ),
    mean_node_path_length_m: mean((x) => x.node_path_length_m),
    mean_nearest_over_path_length: mean((x) => x.nearest_mean_over_path_length),
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
