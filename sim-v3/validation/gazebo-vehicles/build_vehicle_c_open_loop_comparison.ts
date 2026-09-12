import { createHash } from "node:crypto";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { gazeboOdomToTask } from "../rl-campaign/ports/task-trace-bridge.ts";
const ROOT = "artifacts/gazebo/vehicle-c",
  IDS = [
    "straight-ahead",
    "pure-lateral",
    "rotation-in-place",
    "allocation-chirp",
  ],
  sha = async (p: string) =>
    createHash("sha256")
      .update(await readFile(p))
      .digest("hex"),
  pct = (a: number[], p: number) =>
    [...a].sort((x, y) => x - y)[
      Math.min(a.length - 1, Math.ceil(p * a.length) - 1)
    ],
  stats = (a: number[]) => ({
    mean: a.reduce((s, x) => s + x, 0) / a.length,
    p95: pct(a, 0.95),
    max: Math.max(...a),
    final: a.at(-1),
  }),
  wrap = (x: number) => Math.abs(Math.atan2(Math.sin(x), Math.cos(x)));
const scenarios = [];
for (const id of IDS) {
  const node = JSON.parse(
      await readFile(`${ROOT}/node-reference/${id}.json`, "utf8"),
    ),
    gz = JSON.parse(
      await readFile(`${ROOT}/open-loop-corrected/${id}.json`, "utf8"),
    ),
    schedule = JSON.parse(
      await readFile(`${ROOT}/schedules/${id}.json`, "utf8"),
    );
  if (
    node.rows.length !== gz.rows.length ||
    node.rows.length !== schedule.samples.length
  )
    throw Error(`${id}: unmatched row counts`);
  if (
    gz.rows.some(
      (r: any, i: number) =>
        r.iteration_delta !== 5 ||
        Math.abs(r.sim_time_delta_s - 0.05) > 1e-12 ||
        Math.abs(r.time_s - (i + 1) * 0.05) > 1e-9,
    )
  )
    throw Error(`${id}: Gazebo synchronization invariant failed`);
  const bcod = node.rows.map((r: any, i: number) => ({
      time_s: (i + 1) * 0.05,
      north_m: r.state.N_m,
      east_m: r.state.E_m,
      heading_rad: r.state.yaw_rad,
      command: schedule.samples[i],
      delivered: {
        port_thrust_n: r.applied.port.thrust_n,
        port_azimuth_rad: r.applied.port.azimuth_rad,
        starboard_thrust_n: r.applied.starboard.thrust_n,
        starboard_azimuth_rad: r.applied.starboard.azimuth_rad,
      },
    })),
    gazebo = gz.rows.map((r: any) => {
      const s = gazeboOdomToTask(r.time_s, r.enu);
      return {
        time_s: r.time_s,
        gazebo_iteration: r.gazebo_iteration,
        gazebo_sim_time_s: r.gazebo_sim_time_s,
        north_m: s.N_m,
        east_m: s.E_m,
        raw_heading_rad: s.yaw_rad,
        heading_rad: id === "rotation-in-place" ? -s.yaw_rad : s.yaw_rad,
        command: r.command,
        delivered: r.delivered,
      };
    }),
    errors = bcod.map((r: any, i: number) => ({
      time_s: r.time_s,
      horizontal_position_error_m: Math.hypot(
        r.north_m - gazebo[i].north_m,
        r.east_m - gazebo[i].east_m,
      ),
      heading_error_rad: wrap(r.heading_rad - gazebo[i].heading_rad),
    })),
    position = errors.map((x: any) => x.horizontal_position_error_m),
    heading = errors.map((x: any) => x.heading_error_rad),
    increments = position.map((x: number, i: number) =>
      i ? x - position[i - 1] : 0,
    ),
    jumpIndex = increments.indexOf(Math.max(...increments)),
    jump = increments[jumpIndex],
    medianIncrement = pct(increments.map(Math.abs), 0.5),
    nodePath = Math.hypot(bcod.at(-1).north_m, bcod.at(-1).east_m),
    gazeboPath = Math.hypot(gazebo.at(-1).north_m, gazebo.at(-1).east_m);
  scenarios.push({
    id,
    dt_s: 0.05,
    gazebo_capture_model_sha256:
      id === "allocation-chirp"
        ? "2c60c0ee527190bbe0afb658d185f0f24053a872c71e770a2827c594abd6fef4"
        : "8e12f93b5bd97800dcf1af894a14030a8ba50e5e8c8e27ed5d0a7b22b7f3fe2e",
    synchronization: gz.synchronization,
    command_schedule: schedule.samples,
    bcod_sim: bcod,
    gazebo_harmonic: gazebo,
    error_time_series: errors,
    summary: {
      horizontal_position_error_m: stats(position),
      heading_error_rad: stats(heading),
      path_length_m: { bcod_sim: nodePath, gazebo_harmonic: gazeboPath },
      ...(id === "rotation-in-place"
        ? {
            heading_convention_normalization: {
              applied: "negate Gazebo task-frame yaw",
              reason:
                "The retained SDF serializes the NED port/starboard lateral signs directly into FLU, reversing differential-thrust yaw handedness.",
              final_heading_magnitude_ratio:
                Math.abs(gazebo.at(-1).raw_heading_rad) /
                Math.max(Math.abs(bcod.at(-1).heading_rad), 1e-12),
            },
          }
        : {}),
      relative_position_error:
        Math.max(nodePath, gazeboPath) < 0.1
          ? null
          : Math.max(...position) / Math.max(nodePath, gazeboPath),
      ...(Math.max(nodePath, gazeboPath) < 0.1
        ? { relative_position_error_exclusion: "Excluded because both translational endpoint path lengths are below 0.1 m." }
        : {}),
      divergence_pattern:
        jump > Math.max(0.05, 10 * medianIncrement)
          ? {
              kind: "sharp-jump",
              time_s: errors[jumpIndex].time_s,
              increment_m: jump,
            }
          : {
              kind: "steady-growth",
              largest_increment_time_s: errors[jumpIndex].time_s,
              largest_increment_m: jump,
            },
    },
  });
}
const straight = scenarios.find((x: any) => x.id === "straight-ahead")!;
const straightFinal = straight.bcod_sim.at(-1);
const commandDeliveryDiff = scenarios.some((scenario: any) =>
  scenario.command_schedule.some(
    (sample: any) =>
      Math.abs(sample.port_commanded_thrust_n - sample.port_thrust_n) > 1e-6 ||
      Math.abs(sample.starboard_commanded_thrust_n - sample.starboard_thrust_n) > 1e-6,
  ),
);
const artifact = {
  schema_version: 2,
  artifact_kind: "vehicle-c-open-loop-fixed-input-trajectory-comparison",
  timing: { physics_timestep_s: 0.05, full_resolution: true },
  diagnostics: {
    actuator_lag_exercised: commandDeliveryDiff,
    straight_ahead_lateral_drift: {
      persists_in_retained_low_thrust_campaign: false,
      bcod_sim_final_east_m: straightFinal.east_m,
      bcod_sim_final_heading_deg: (straightFinal.heading_rad * 180) / Math.PI,
      classification:
        "The high-thrust straight-running equilibrium is physically unstable in both engines; the retained 3% cap case stays below the six-second onset criterion.",
      stability_diagnostic: "artifacts/gazebo/vehicle-c/timestep-diagnostic/stability-diagnostic.json",
    },
  },
  scenarios,
  provenance: {
    generator: {
      path: "validation/gazebo-vehicles/generate_vehicle_c_reference.ts",
      sha256: await sha(
        "validation/gazebo-vehicles/generate_vehicle_c_reference.ts",
      ),
    },
    runner: {
      path: "validation/gazebo-vehicles/run_gazebo_vehicle_c_open_loop.py",
      sha256: await sha(
        "validation/gazebo-vehicles/run_gazebo_vehicle_c_open_loop.py",
      ),
    },
    sdf: {
      path: "gazebo/models/vehicle-c-azimuth/model.sdf",
      capture_sha256: "8e12f93b5bd97800dcf1af894a14030a8ba50e5e8c8e27ed5d0a7b22b7f3fe2e",
      corrected_sha256: await sha("gazebo/models/vehicle-c-azimuth/model.sdf"),
      yaw_handedness_correction: {
        applied_after_capture: true,
        change: "Negated the lateral coordinate of both pod mounts when translating NED definitions into Gazebo FLU.",
        raw_trace_normalization: "Rotation-in-place Gazebo yaw is negated analytically; no physics rerun was required.",
        scenario_evidence: {
          "straight-ahead": "Unchanged pre-fix capture retained because every port/starboard command is symmetric.",
          "pure-lateral": "Unchanged pre-fix capture retained because every port/starboard command is symmetric.",
          "rotation-in-place": "Pre-fix capture retained with analytical yaw-sign normalization; translation is near zero.",
          "allocation-chirp": "Physics rerun against corrected_sha256 because 40 of 120 commands are asymmetric and cannot be repaired analytically.",
        },
      },
      thrust_cap_correction: {
        task: "vehicle-c-gazebo-thrust-fix",
        date: "2026-09-07",
        forward_n: 1340.506075016061,
        reverse_n: 670.2530375080305,
      },
    },
    parameter_map: {
      path: "validation/gazebo-vehicles/vehicle-c-parameter-map.json",
      sha256: await sha(
        "validation/gazebo-vehicles/vehicle-c-parameter-map.json",
      ),
    },
    controller: "none; fixed-input open-loop",
  },
};
await mkdir(ROOT, { recursive: true });
await writeFile(
  `${ROOT}/open-loop-trajectory-comparison-thrust-corrected.json`,
  JSON.stringify(artifact, null, 2) + "\n",
);
console.log(
  JSON.stringify(
    Object.fromEntries(scenarios.map((x: any) => [x.id, x.summary])),
    null,
    2,
  ),
);
