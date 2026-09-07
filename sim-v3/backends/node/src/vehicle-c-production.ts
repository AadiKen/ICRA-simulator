import { readFileSync } from "node:fs";
import { createHash } from "node:crypto";
import { VehicleParameters } from "../../../core/vehicleParameters.js";
import { VEHICLES } from "../../../packages/vehicle-sdk/src/index.ts";
const symmetrize = (matrix: number[][]) => matrix.map((row, i) => row.map((value, j) => (value + matrix[j][i]) / 2));
export const VEHICLE_C_MAX_FORWARD_THRUST_N = 1000;
export const VEHICLE_C_MAX_REVERSE_THRUST_N = 500;
export function buildVehicleCProductionConfiguration(options: { angleContinuityWeight?: number } = {}) {
  const definition = VEHICLES["vehicle-c-azimuth"],
    hydrodynamics = JSON.parse(readFileSync(new URL("../../../artifacts/capytaine/vehicle-c-parametric-resolved.json", import.meta.url), "utf8")),
    runtime = hydrodynamics.runtime_parameters;
  if (hydrodynamics.vehicle_id !== definition.id) throw new Error("Vehicle C production hydrodynamics identity mismatch.");
  const parameters = new VehicleParameters({
    id: definition.id,
    vehicleClass: "surface_coupled6",
    geometry: {
      ...runtime.geometry,
      height: definition.geometry.draft.value * 2,
    },
    massProps: { ...runtime.massProps, cg: { x: 0, y: 0, z: 0 } },
    addedMass: { matrix6: symmetrize(runtime.addedMass.matrix6) },
    damping: { ...runtime.damping },
    hydrodynamics: runtime.hydrodynamics,
    restoring: {
      waterDensity: 1025,
      gravity: 9.80665,
      waterplaneArea: definition.geometry.length.value * definition.geometry.width.value,
      displacementVolume: definition.mass.value / 1025,
      metacentricHeightRoll: definition.hydrostatics!.gm_transverse.value,
      metacentricHeightPitch: definition.hydrostatics!.gm_longitudinal.value,
      hydrostaticStiffnessMatrix6: runtime.restoring.hydrostaticStiffnessMatrix6,
      cob: { x: 0, y: 0, z: -definition.geometry.draft.value / 2 },
    },
    actuator: {
      behaviorVersion: "integrated-v1",
      maxThrust: VEHICLE_C_MAX_FORWARD_THRUST_N,
      beam: definition.geometry.width.value,
      motorTimeConstant: 0.35,
    },
    controlledDOF: ["surge", "sway", "yaw"],
    allocator: {
      mode: "dual-azimuth-minimum-thrust-norm",
      regularization: 0,
      reachabilityTolerance: 1e-5,
      angleContinuityWeight: options.angleContinuityWeight ?? 0,
    },
    effectors: definition.effectors.map((item) => ({
      id: item.id,
      type: "AzimuthThruster",
      pos: item.position_m,
      axis: [1, 0, 0],
      behaviorVersion: "integrated-v1",
      maxForwardThrust: VEHICLE_C_MAX_FORWARD_THRUST_N,
      maxReverseThrust: VEHICLE_C_MAX_REVERSE_THRUST_N,
      dynamics: { tau: item.time_constant_s },
      azimuth: {
        initial: 0,
        tau: item.time_constant_s,
        rateMax: item.rate_limit_per_s,
        min: -Math.PI,
        max: Math.PI,
      },
      conversion: { type: "linear" },
      power: { maxW: item.max_power_w },
    })),
    validation: {
      status: definition.validation.status,
      claim: definition.validation.claim,
      limitations: definition.validation.limitations,
      production_path: "coupled6-dual-azimuth-parametric-hydrodynamics",
      hydrodynamics_status: hydrodynamics.status,
    },
  });
  return {
    definition,
    hydrodynamics,
    parameters,
    manifest: {
      vehicle_id: definition.id,
      plant: "coupled6",
      actuation: "dual-azimuth",
      allocation_strategy: "minimum-thrust-norm",
      actuator_dynamics: {
        thrust_time_constant_s: 0.35,
        thrust_rate_limit_n_s: null,
        azimuth_rate_limit_rad_s: (30 * Math.PI) / 180,
      },
      hydrodynamics_source: "parametric-hull Capytaine",
      roll_viscous_damping_artifact: definition.damping.roll_viscous_decomposition?.artifact_path,
      hydrodynamics_checksum_sha256: createHash("sha256")
        .update(readFileSync(new URL("../../../artifacts/capytaine/vehicle-c-parametric-resolved.json", import.meta.url)))
        .digest("hex"),
      validation_status: definition.validation.status,
      claim_limit: "Control-allocation and composability demonstrated; behavioral dynamics not independently validated.",
    },
  };
}
