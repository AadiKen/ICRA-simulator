import {createHash} from "node:crypto";

export type Plant = "planar3" | "coupled6";
export type Backend = "browser" | "node" | "tensor-cpu" | "tensor-mps" | "tensor-cuda";
export type Integrator = "rk4" | "semi-implicit-euler";
export type SensorArtifactMode = "summary" | "selected-raw" | "all-raw";
export interface SensorArtifactPolicy {
  mode: SensorArtifactMode;
  raw_plugins?: string[];
  max_bytes_per_run?: number;
}

export interface ExperimentV1 {
  schema_version: 1;
  experiment: {name: string; seed: number; timestep_s: number; duration_s: number};
  backend: {type: Backend; parallel_environments?: number};
  vehicle: {preset: string; plant: Plant; hydrodynamic_reference_speed_mps?: number; hydrodynamics?: {artifact_checksum_sha256: string; frequency_grid_rad_s: number[]; extrapolation_policy: "reject"}};
  environment?: {
    current_mps?: [number, number, number];
    wind_mps?: [number, number, number];
    regular_wave?: {amplitude_m: number; period_s: number; phase_rad?: number; direction_rad: number; water_depth_m?: number};
    data_sources?: {mode:"realtime_forecast"|"historical_replay";ndbc?:{enabled:boolean};coops?:{enabled:boolean};nws?:{enabled:boolean;user_agent?:{application:string;contact:string}};rtofs?:{enabled:boolean};era5?:{enabled:boolean;credentials?:{source:"environment";env_var:string}|{source:"file";path:string}}};
  };
  initial_state?: {position_ned_m?:[number,number,number];attitude_rad?:[number,number,number];body_velocity_mps?: [number, number, number];angular_rate_body_rad_s?:[number,number,number]};
  integrator?: Integrator;
  sensors?: Array<{plugin: string; enabled?: boolean; oracle?: boolean}>;
  mission: {type: string; [key: string]: unknown};
  metrics?: string[];
  outputs?: {directory?: string; state_hz?: number; sensor_artifacts?: Partial<SensorArtifactPolicy>};
}

export interface ResolvedExperimentV1 extends ExperimentV1 {
  integrator: Integrator;
  sensors: Array<{plugin: string; enabled: boolean; oracle: boolean}>;
  metrics: string[];
  outputs: {directory: string; state_hz: number; sensor_artifacts: SensorArtifactPolicy};
  resolution: {
    checksum_sha256: string;
    hydrodynamic_reference_speed_mps: number;
    warnings: string[];
    ground_truth_exposed: boolean;
    hydrodynamics?: {approximation: "constant-coefficient-no-radiation-memory"; selection_method: "wave-encounter"; evaluation_frequency_rad_s: number; intrinsic_wave_frequency_rad_s: number; reference_speed_mps: number; encounter_angle_rad: number; wave_number_rad_m: number; interpolation_bracket_rad_s: [number, number]; artifact_checksum_sha256: string; extrapolation_policy: "reject"};
  };
}

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) throw new Error(`${name} must be finite.`);
}

export function validateExperiment(config: ExperimentV1): void {
  if (config.schema_version !== 1) throw new Error("Only schema_version 1 is supported.");
  if (!config.experiment?.name?.trim()) throw new Error("experiment.name is required.");
  if (!Number.isInteger(config.experiment.seed)) throw new Error("experiment.seed must be an integer.");
  assertFinite("experiment.timestep_s", config.experiment.timestep_s);
  assertFinite("experiment.duration_s", config.experiment.duration_s);
  if (config.experiment.timestep_s <= 0 || config.experiment.duration_s <= 0) throw new Error("Simulation time values must be positive.");
  if (!config.vehicle?.preset || !["planar3", "coupled6"].includes(config.vehicle.plant)) throw new Error("A valid vehicle preset and plant are required.");
  if(config.vehicle.preset==="searobotics-surveyor-m1.8"||config.vehicle.preset==="surveyor"){
    const mission=config.mission as any;
    const validPoint=(point:any)=>Number.isFinite(point?.lat)&&point.lat>=-90&&point.lat<=90&&Number.isFinite(point?.lon)&&point.lon>=-180&&point.lon<=180;
    if(config.vehicle.plant!=="planar3")throw new Error("SeaRobotics Surveyor integration preset currently requires planar3.");
    if(mission?.type==="surveyor-waypoint"){
      if(!validPoint(mission.origin)||!validPoint(mission.erp)||!Array.isArray(mission.waypoints)||mission.waypoints.length===0||!mission.waypoints.every(validPoint))throw new Error("Surveyor waypoint missions require origin, ERP, and non-empty raw lat/lon waypoints.");
      if(mission.max_thrust_command!==undefined&&(!Number.isInteger(mission.max_thrust_command)||mission.max_thrust_command<0||mission.max_thrust_command>70))throw new Error("Surveyor max_thrust_command must be an integer in [0,70].");
    }
  }
  const wave = config.environment?.regular_wave;
  const dataSources=config.environment?.data_sources;
  if(dataSources?.nws?.enabled){const agent=dataSources.nws.user_agent;if(!agent?.application?.trim()||!agent.contact?.trim()||!(/@|https?:\/\//.test(agent.contact)))throw new Error("NWS data sourcing requires environment.data_sources.nws.user_agent with application and email or URL contact.");}
  if(dataSources?.era5?.enabled){const credential=dataSources.era5.credentials;if(dataSources.mode!=="historical_replay")throw new Error("ERA5 is available only in historical_replay mode.");if(!credential||(credential.source==="environment"?!credential.env_var?.trim():!credential.path?.trim()))throw new Error("ERA5 requires environment.data_sources.era5.credentials with a local environment variable or file reference.");}
  if (wave) {
    if (config.vehicle.plant !== "coupled6") throw new Error("Physical regular-wave forcing requires the coupled6 plant.");
    assertFinite("wave amplitude", wave.amplitude_m);
    assertFinite("wave period", wave.period_s);
    if (wave.amplitude_m < 0 || wave.period_s <= 0) throw new Error("Wave amplitude must be non-negative and period must be positive.");
  }
  const hydrodynamics = config.vehicle.hydrodynamics;
  if (hydrodynamics && (!/^[a-f0-9]{64}$/.test(hydrodynamics.artifact_checksum_sha256) || hydrodynamics.extrapolation_policy !== "reject" || hydrodynamics.frequency_grid_rad_s.length < 2 || hydrodynamics.frequency_grid_rad_s.some((value, index, grid) => !Number.isFinite(value) || value <= 0 || (index > 0 && value <= grid[index - 1])))) {
    throw new Error("vehicle hydrodynamics requires a checksum, increasing frequency grid, and reject extrapolation policy.");
  }
  if ((config.backend.parallel_environments ?? 1) < 1) throw new Error("parallel_environments must be at least 1.");
  for(const [name,value] of Object.entries(config.initial_state??{}))if(!Array.isArray(value)||value.length!==3||value.some((x)=>!Number.isFinite(x)))throw new Error(`initial_state.${name} must contain three finite values.`);
  for (const sensor of config.sensors ?? []) {
    if (!sensor.plugin) throw new Error("Every sensor requires a plugin id.");
    if (sensor.oracle && config.backend.type === "browser") throw new Error("Oracle sensors are not permitted in the browser backend.");
  }
  const artifacts = config.outputs?.sensor_artifacts;
  if (artifacts?.mode === "selected-raw" && !(artifacts.raw_plugins?.length)) {
    throw new Error("selected-raw sensor artifacts require at least one raw_plugins entry.");
  }
  if (artifacts?.max_bytes_per_run !== undefined && (!Number.isInteger(artifacts.max_bytes_per_run) || artifacts.max_bytes_per_run <= 0)) {
    throw new Error("sensor_artifacts.max_bytes_per_run must be a positive integer.");
  }
}

function stable(value: unknown): string {
  if (Array.isArray(value)) return `[${value.map(stable).join(",")}]`;
  if (value && typeof value === "object") {
    return `{${Object.entries(value as Record<string, unknown>).sort(([a], [b]) => a.localeCompare(b)).map(([k, v]) => `${JSON.stringify(k)}:${stable(v)}`).join(",")}}`;
  }
  return JSON.stringify(value);
}

export function resolveExperiment(config: ExperimentV1): ResolvedExperimentV1 {
  validateExperiment(config);
  const canonicalVehiclePreset=config.vehicle.preset==="surveyor"?"searobotics-surveyor-m1.8":config.vehicle.preset;
  const referenceSpeed = config.vehicle.hydrodynamic_reference_speed_mps ?? config.initial_state?.body_velocity_mps?.[0] ?? 0;
  const resolved: ResolvedExperimentV1 = {
    ...structuredClone(config),
    integrator: config.integrator ?? "rk4",
    backend: {...config.backend, parallel_environments: config.backend.parallel_environments ?? 1},
    vehicle: {...config.vehicle, preset:canonicalVehiclePreset, hydrodynamic_reference_speed_mps: referenceSpeed},
    sensors: (config.sensors ?? []).map((sensor) => ({...sensor, enabled: sensor.enabled ?? true, oracle: sensor.oracle ?? false})),
    metrics: config.metrics ?? ["completion", "cross_track_rmse", "propulsion_energy"],
    outputs: {
      directory: config.outputs?.directory ?? `runs/${config.experiment.name}`,
      state_hz: config.outputs?.state_hz ?? 10,
      sensor_artifacts: {
        mode: config.outputs?.sensor_artifacts?.mode ?? "summary",
        raw_plugins: config.outputs?.sensor_artifacts?.raw_plugins ?? [],
        max_bytes_per_run: config.outputs?.sensor_artifacts?.max_bytes_per_run ?? 1_000_000_000
      }
    },
    resolution: {checksum_sha256: "", hydrodynamic_reference_speed_mps: referenceSpeed, warnings: [], ground_truth_exposed: false}
  };
  if (config.environment?.regular_wave) {
    const wave = config.environment.regular_wave, intrinsic = 2 * Math.PI / wave.period_s, g = 9.80665;
    let waveNumber = intrinsic * intrinsic / g;
    if (wave.water_depth_m !== undefined) for (let i = 0; i < 30; i++) { const kh = waveNumber * wave.water_depth_m, tanh = Math.tanh(kh), f = g * waveNumber * tanh - intrinsic * intrinsic, df = g * (tanh + kh * (1 - tanh * tanh)); waveNumber = Math.max(waveNumber - f / df, 1e-12); }
    const angle = wave.direction_rad, evaluation = Math.abs(intrinsic - waveNumber * referenceSpeed * Math.cos(angle)), grid = config.vehicle.hydrodynamics?.frequency_grid_rad_s;
    if (!grid) throw new Error("coupled6 regular-wave experiments require a resolved Capytaine hydrodynamics artifact and frequency grid.");
    const hi = grid.findIndex((value) => value >= evaluation);
    if (hi < 0 || evaluation < grid[0]) throw new Error(`Encounter frequency ${evaluation} rad/s is outside the Capytaine grid and extrapolation is rejected.`);
    resolved.resolution.hydrodynamics = {approximation: "constant-coefficient-no-radiation-memory", selection_method: "wave-encounter", evaluation_frequency_rad_s: evaluation, intrinsic_wave_frequency_rad_s: intrinsic, reference_speed_mps: referenceSpeed, encounter_angle_rad: angle, wave_number_rad_m: waveNumber, interpolation_bracket_rad_s: hi === 0 ? [grid[0], grid[0]] : [grid[hi - 1], grid[hi]], artifact_checksum_sha256: config.vehicle.hydrodynamics!.artifact_checksum_sha256, extrapolation_policy: "reject"};
    resolved.resolution.warnings.push("coupled6 uses constant hydrodynamic coefficients at one resolved encounter frequency; Cummins radiation-memory and time-varying encounter-frequency effects are not modeled.");
  }
  resolved.resolution.checksum_sha256 = createHash("sha256").update(stable({...resolved, resolution: {...resolved.resolution, checksum_sha256: ""}})).digest("hex");
  return resolved;
}
