import {resolveExperiment} from "../packages/experiment-schema/src/index.ts";
import {HeadlessMarineSimulation} from "../packages/core/src/simulation.ts";
import {LegacyProductionEngine} from "../backends/node/src/legacy-production-engine.ts";
import type {ReplayAdapter, RouteFixture, TrajectoryLog, TrajectorySample} from "./types.ts";

const DT = 0.05;
export const bcodAdapter: ReplayAdapter = {
  async runReplay(route: RouteFixture, replayIndex: number): Promise<TrajectoryLog> {
    const origin = 10000;
    const log: TrajectoryLog = {simulator: "bcod-sim", route_id: route.route_id, replay_index: replayIndex, vehicle: route.vehicle, wall_clock_start: new Date().toISOString(), samples: [], metadata: {plant: "planar3", timestep_s: DT, actuation: "requested per-thruster thrust N", internal_ned_origin_m: [origin, origin], logged_frame: "route-local NED; translated to initial (0,0)"}};
    const sim = new HeadlessMarineSimulation(new LegacyProductionEngine());
    let initialized = false;
    try {
      const cfg = resolveExperiment({schema_version: 1, experiment: {name: `determinism-replay-${replayIndex}`, seed: 7319, timestep_s: DT, duration_s: route.duration_s}, backend: {type: "node"}, vehicle: {preset: route.vehicle, plant: "planar3"}, environment: {current_mps: route.environment.current_mps, wind_mps: route.environment.wind_mps}, initial_state: {...route.initial_state, position_ned_m: [origin, origin, 0]}, mission: {type: "rl-common-waypoint-v1", waypoints: [{north_m: 19000, east_m: 19000}]}, sensors: []} as any);
      sim.reset(cfg);
      initialized = true;
      const sample = (): TrajectorySample => {const truth = sim.getGroundTruth(); return {t: truth.time_s, x: truth.position_ned_m[0] - origin, y: truth.position_ned_m[1] - origin, heading_rad: truth.attitude_rad[2]};};
      log.samples.push(sample());
      const steps = Math.round(route.duration_s / DT);
      for (let i = 0; i < steps; i++) {
        const t = i * DT;
        const segment = route.segments.find(s => t >= s.start_s && t < s.end_s);
        if (!segment) throw Error(`No thrust segment at ${t}s`);
        const result = sim.step({actuators: {effectors: {port: {thrust: segment.left_thrust_n}, starboard: {thrust: segment.right_thrust_n}}}});
        log.samples.push(sample());
        if (result.terminated || (result.truncated && i < steps - 1)) throw Error(`Simulation stopped at ${log.samples.at(-1)!.t}s: ${String(result.info?.stop_reason)}`);
      }
    } catch (error) {
      log.crashed_at_t = log.samples.at(-1)?.t ?? 0;
      log.error = String(error);
    } finally {if (initialized) sim.dispose();}
    return log;
  }
};
