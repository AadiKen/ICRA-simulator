import { mkdir, readFile, writeFile } from "node:fs/promises";
import { dirname } from "node:path";
import {
  DeterministicVectorMarineSimulation,
  HeadlessMarineSimulation,
} from "../../packages/core/src/simulation.ts";
import { SeededRandom } from "../../packages/core/src/random.ts";
import { resolveExperiment } from "../../packages/experiment-schema/src/index.ts";
import { LegacyProductionEngine } from "../../backends/node/src/legacy-production-engine.ts";
import {
  losAction,
  VEHICLE_WRENCH_LIMITS,
} from "./ports/portable-controllers.ts";
const CONTRACT = "artifacts/rl-campaign/surveyor/task-contract-frozen.json",
  OUT =
    process.argv[2] ??
    "artifacts/rl-campaign/vehicle-c-distance-diagnostic.json",
  task = JSON.parse(await readFile(CONTRACT, "utf8")).tasks.find(
    (x: any) => x.task_id === "common-waypoint-transit-v1",
  ),
  dt = task.timing.physics_timestep_s,
  maxSteps = task.timing.episode_length_steps,
  hold = Math.round(task.timing.control_interval_s / dt),
  seedArgument = process.argv.find((value) => value.startsWith("--seed=")),
  seeds = seedArgument
    ? [Number(seedArgument.split("=")[1])]
    : [30000, 30003, 30007, 30013, 30048],
  calm = process.argv.includes("--calm"),
  vehicleA = process.argv.includes("--vehicle-a"),
  continuityArgument = process.argv.find((value) =>
    value.startsWith("--angle-continuity-weight="),
  ),
  angleContinuityWeight = Number(continuityArgument?.split("=")[1] ?? 0),
  traceControlRate = process.argv.includes("--trace-control-rate"),
  vehicle = vehicleA ? "vehicle-a-otter" : "vehicle-c-azimuth",
  plant = vehicleA ? "planar3" : "coupled6",
  wrenchLimits = VEHICLE_WRENCH_LIMITS[vehicle];
const randomization = (seed: number) => {
  const r = new SeededRandom(seed),
    u = (a: number, b: number) => a + (b - a) * r.next(),
    rr = task.reset_randomization,
    angle = (u(...rr.route_rotation_deg) * Math.PI) / 180,
    start: [number, number] = [
      10000 + u(...rr.start_position_offset_m),
      10000 + u(...rr.start_position_offset_m),
    ],
    rot = ([n, e]: number[]): [number, number] => [
      start[0] + n * Math.cos(angle) - e * Math.sin(angle),
      start[1] + n * Math.sin(angle) + e * Math.cos(angle),
    ],
    cs = u(...rr.current_speed_m_s),
    cd = u(0, 2 * Math.PI),
    ws = u(...rr.wind_speed_m_s),
    wd = u(0, 2 * Math.PI);
  return {
    start,
    heading: (u(...rr.start_heading_deg) * Math.PI) / 180,
    route: rr.route_relative_m.map(rot),
    current: [cs * Math.cos(cd), cs * Math.sin(cd), 0],
    wind: [ws * Math.cos(wd), ws * Math.sin(wd), 0],
  };
};
const specs = seeds.map((seed) => ({ seed, random: randomization(seed) })),
  configs = specs.map(({ seed, random }) =>
    resolveExperiment({
      schema_version: 1,
      experiment: {
        name: `${vehicle}-diagnostic-${seed}${calm ? "-calm" : ""}`,
        seed,
        timestep_s: dt,
        duration_s: 120,
      },
      backend: { type: "node" },
      vehicle: { preset: vehicle, plant },
      environment: {
        current_mps: calm ? [0, 0, 0] : random.current,
        wind_mps: calm ? [0, 0, 0] : random.wind,
      },
      initial_state: {
        position_ned_m: [...random.start, 0],
        attitude_rad: [0, 0, random.heading],
      },
      mission: {
        type: "rl-common-waypoint-v1",
        waypoints: [{ north_m: 19000, east_m: 19000 }],
      },
      sensors: [],
    } as any),
  ),
  runs = specs.map(({ seed, random }) => ({
    seed,
    ...random,
    waypoint: 0,
    lastCommand: null as any,
    trace: [] as any[],
  })),
  sim = new DeterministicVectorMarineSimulation(
    runs.length,
    () =>
      new HeadlessMarineSimulation(
        new LegacyProductionEngine(undefined, {
          vehicleC: { angleContinuityWeight },
        }),
      ),
  );
sim.reset(configs);
const geometry = (run: any, truth: any) => {
  const a = run.waypoint ? run.route[run.waypoint - 1] : run.start,
    b = run.route[run.waypoint],
    dn = b[0] - a[0],
    de = b[1] - a[1],
    length = Math.hypot(dn, de),
    cn = dn / length,
    ce = de / length,
    n = truth.position_ned_m[0],
    e = truth.position_ned_m[1],
    along = (n - a[0]) * cn + (e - a[1]) * ce,
    cross = -ce * (n - a[0]) + cn * (e - a[1]);
  return {
    distance: Math.hypot(b[0] - n, b[1] - e),
    cross,
    along,
    leg_length_m: length,
    passed: along >= length && Math.abs(cross) <= 15.361124064575238,
  };
};
for (let step = 0; step < maxSteps; step++) {
  const commands = runs.map((run: any, i) => {
    if (step % hold) return run.lastCommand;
    const truth: any = sim.getGroundTruth(i),
      start = run.waypoint ? run.route[run.waypoint - 1] : run.start,
      goal = run.route[run.waypoint],
      wrench = losAction(
        "LOS-PID-v2",
        {
          north_m: truth.position_ned_m[0],
          east_m: truth.position_ned_m[1],
          heading_rad: truth.attitude_rad[2],
          surge_mps: truth.velocity_body_mps[0],
          yaw_rate_rad_s: truth.angular_rate_body_rad_s[2],
        },
        start,
        goal,
        wrenchLimits,
      );
    return (run.lastCommand = {
      actuators: { desiredWrench: [wrench[0], 0, 0, 0, 0, wrench[1]] },
    });
  });
  const result = sim.step(commands);
  for (let i = 0; i < runs.length; i++) {
    const run: any = runs[i],
      truth: any = sim.getGroundTruth(i),
      g = geometry(run, truth);
    if (run.waypoint < run.route.length - 1 && (g.distance <= 6 || g.passed))
      run.waypoint++;
    if ((traceControlRate && step % hold === hold - 1) || (!traceControlRate && step % 20 === 19) || step === 0) {
      const vd: any = result.infos[i].vehicle_diagnostics,
        goal = run.route.at(-1),
        desired = run.lastCommand.actuators.desiredWrench;
      run.trace.push({
        time_s: (step + 1) * dt,
        waypoint_index: run.waypoint,
        distance_to_current_m: g.distance,
        distance_to_final_m: Math.hypot(
          goal[0] - truth.position_ned_m[0],
          goal[1] - truth.position_ned_m[1],
        ),
        signed_cross_track_m: g.cross,
        along_track_m: g.along,
        leg_length_m: g.leg_length_m,
        position_ned_m: truth.position_ned_m,
        heading_rad: truth.attitude_rad[2],
        surge_mps: truth.velocity_body_mps[0],
        sway_mps: truth.velocity_body_mps[1],
        yaw_rate_rad_s: truth.angular_rate_body_rad_s[2],
        desired_wrench_surge_yaw: [desired[0], desired[5]],
        actual_wrench_surge_sway_yaw: [
          vd.full_wrench[0],
          vd.full_wrench[1],
          vd.full_wrench[5],
        ],
        pods: vd.effectors.map((x: any) => ({
          id: x.id,
          thrust_n: x.thrust,
          azimuth_rad: x.azimuth,
          target: vd.applied_command[x.id],
        })),
      });
    }
  }
}
sim.dispose();
const classify = (trace: any[]) => {
  const distances = trace.map((x) => x.distance_to_final_m),
    closest = Math.min(...distances),
    closestIndex = distances.indexOf(closest),
    improvement = distances[0] - closest,
    rebound = Math.max(...distances.slice(closestIndex)) - closest,
    tail = distances.slice(-20),
    tailSpan = Math.max(...tail) - Math.min(...tail),
    directionChanges = distances
      .slice(2)
      .reduce(
        (n, d, i) =>
          n +
          (Math.sign(d - distances[i + 1]) !==
          Math.sign(distances[i + 1] - distances[i])
            ? 1
            : 0),
        0,
      );
  return {
    classification:
      improvement < 2
        ? "minimal-progress"
        : rebound >= 5 && directionChanges >= 4
          ? "approaches-then-oscillates_or_reverses"
          : tailSpan < 1
            ? "approaches-then-stalls"
            : "approaches-but-does-not-close",
    start_distance_m: distances[0],
    closest_distance_m: closest,
    closest_time_s: trace[closestIndex].time_s,
    final_distance_m: distances.at(-1),
    rebound_after_closest_m: rebound,
    tail_20s_span_m: tailSpan,
    radial_direction_changes: directionChanges,
  };
};
const rows = runs.map((run: any) => ({
    seed: run.seed,
    ...classify(run.trace),
    trace_1hz: run.trace,
  })),
  artifact = {
    schema_version: 1,
    artifact_kind: "los-pid-v2-distance-to-goal-diagnostic",
    status: "DIAGNOSTIC_COMPLETE",
    vehicle,
    plant,
    training_performed: false,
    evaluation_seeds: seeds,
    controller: "shared LOS-PID-v2",
    wrench_limits: wrenchLimits,
    allocator_angle_continuity_weight: angleContinuityWeight,
    protocol: {
      task_contract: CONTRACT,
      physics_timestep_s: dt,
      control_interval_s: task.timing.control_interval_s,
      duration_s: 120,
      trace_interval_s: traceControlRate ? task.timing.control_interval_s : 1,
      environment: calm ? "calm-counterfactual" : "frozen-seeded-disturbances",
    },
    rows,
  };
await mkdir(dirname(OUT), { recursive: true });
await writeFile(OUT, JSON.stringify(artifact, null, 2) + "\n");
console.log(
  JSON.stringify(
    rows.map(({ trace_1hz, ...row }: any) => row),
    null,
    2,
  ),
);
