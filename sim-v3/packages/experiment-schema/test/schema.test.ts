import assert from "node:assert/strict";
import {resolveExperiment, validateExperiment, type ExperimentV1} from "../src/index.ts";

const base: ExperimentV1 = {
  schema_version: 1,
  experiment: {name: "regular-wave", seed: 42, timestep_s: 0.02, duration_s: 60},
  backend: {type: "node"},
  vehicle: {preset: "vehicle-b", plant: "coupled6", hydrodynamics: {artifact_checksum_sha256: "a".repeat(64), frequency_grid_rad_s: [0.3, 0.5, 1, 1.5, 2, 2.5, 3], extrapolation_policy: "reject"}},
  initial_state: {body_velocity_mps: [1.2, 0, 0]},
  environment: {regular_wave: {amplitude_m: 0.2, period_s: 4, direction_rad: 0}},
  mission: {type: "waypoint"}
};
const a = resolveExperiment(base);
const b = resolveExperiment(base);
assert.equal(a.resolution.checksum_sha256, b.resolution.checksum_sha256);
assert.equal(a.resolution.hydrodynamic_reference_speed_mps, 1.2);
assert.match(a.resolution.warnings[0], /radiation-memory/);
assert.equal(a.resolution.hydrodynamics?.selection_method, "wave-encounter");
assert.deepEqual(a.resolution.hydrodynamics?.interpolation_bracket_rad_s, [1, 1.5]);
assert.equal(a.outputs.sensor_artifacts.mode, "summary");
assert.equal(a.outputs.sensor_artifacts.max_bytes_per_run, 1_000_000_000);
assert.throws(() => validateExperiment({...base, outputs: {sensor_artifacts: {mode: "selected-raw"}}}), /raw_plugins/);
assert.throws(() => validateExperiment({...base, vehicle: {preset: "otter", plant: "planar3"}}), /coupled6/);
assert.throws(() => resolveExperiment({...base, vehicle: {...base.vehicle, hydrodynamics: undefined}}), /Capytaine/);
assert.throws(()=>validateExperiment({...base,environment:{...base.environment,data_sources:{mode:"realtime_forecast",nws:{enabled:true}}}}),/user_agent/);
assert.doesNotThrow(()=>validateExperiment({...base,environment:{...base.environment,data_sources:{mode:"realtime_forecast",nws:{enabled:true,user_agent:{application:"BCOD",contact:"ops@example.test"}}}}}));
assert.throws(()=>validateExperiment({...base,environment:{...base.environment,data_sources:{mode:"historical_replay",era5:{enabled:true}}}}),/credentials/);
assert.throws(()=>validateExperiment({...base,environment:{...base.environment,data_sources:{mode:"realtime_forecast",era5:{enabled:true,credentials:{source:"environment",env_var:"BCOD_CDS_API_KEY"}}}}}),/historical_replay/);
assert.doesNotThrow(()=>validateExperiment({...base,environment:{...base.environment,data_sources:{mode:"historical_replay",era5:{enabled:true,credentials:{source:"environment",env_var:"BCOD_CDS_API_KEY"}}}}}));
console.log("Experiment schema tests passed.");
