export const SIMULATORS = ["bcod-sim", "gazebo", "vrx", "holoocean", "stonefish"] as const;
export type Simulator = typeof SIMULATORS[number];
export interface RouteFixture {
  route_id: string;
  vehicle: "vehicle-a-otter";
  duration_s: number;
  initial_state: {
    position_ned_m: [number, number, number];
    attitude_rad: [number, number, number];
    body_velocity_mps: [number, number, number];
    angular_rate_body_rad_s: [number, number, number];
  };
  environment: {current_mps: [number, number, number]; wind_mps: [number, number, number]; water: "flat"};
  segments: {start_s: number; end_s: number; left_thrust_n: number; right_thrust_n: number}[];
}
export interface TrajectorySample {t: number; x: number; y: number; heading_rad: number}
export interface TrajectoryLog {
  simulator: Simulator;
  route_id: string;
  replay_index: number;
  vehicle: "vehicle-a-otter";
  wall_clock_start: string;
  samples: TrajectorySample[];
  crashed_at_t?: number;
  error?: string;
  metadata?: Record<string, unknown>;
}
export interface ReplayAdapter {
  runReplay(route: RouteFixture, replayIndex: number): Promise<TrajectoryLog>;
}
export function validateRoute(route: RouteFixture): void {
  if (route.route_id !== "determinism-replay-route-v1" || route.vehicle !== "vehicle-a-otter" || route.duration_s !== 90) throw Error("Unexpected replay fixture identity");
  if (route.environment.water !== "flat" || [...route.environment.current_mps, ...route.environment.wind_mps].some(v => v !== 0)) throw Error("Replay environment must be calm and deterministic");
  if ([...route.initial_state.position_ned_m, ...route.initial_state.attitude_rad, ...route.initial_state.body_velocity_mps, ...route.initial_state.angular_rate_body_rad_s].some(v => v !== 0)) throw Error("Replay initial state must be zero");
  if (route.segments.length !== 5) throw Error("Expected five route segments");
  let end = 0;
  for (const segment of route.segments) {
    if (segment.start_s !== end || !(segment.end_s > end) || [segment.left_thrust_n, segment.right_thrust_n].some(v => !Number.isFinite(v) || Math.abs(v) > 95)) throw Error("Invalid or discontinuous thrust schedule");
    end = segment.end_s;
  }
  if (end !== route.duration_s) throw Error("Thrust schedule does not cover duration");
}
