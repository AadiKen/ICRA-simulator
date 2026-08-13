import {performance} from "node:perf_hooks";
import {RigidBodyState} from "../core/rigidBodyState.js";
import {DynamicsCore} from "../core/dynamicsCore.js";
import {CoupledSixPlant} from "../core/coupledSixPlant.js";
import {ActuatorModel} from "../core/forces/actuatorModel.js";
import {AddedMassCoriolis} from "../core/forces/addedMassCoriolis.js";
import {HydrodynamicDamping} from "../core/forces/hydrodynamicDamping.js";
import {createOtterParameters} from "../core/vehicles/otter.js";

const count = Number(process.argv[process.argv.indexOf("--vessels") + 1]) || 1;
const steps = Number(process.argv[process.argv.indexOf("--steps") + 1]) || 5000;
const dt = 0.05;
const env = {waterV: {x: 0, y: 0, z: 0}};
const command = {surgeForce: 40, differentialForce: 4};

function percentile(values, p) {
    const sorted = [...values].sort((a, b) => a - b);
    return sorted[Math.min(Math.floor(sorted.length * p), sorted.length - 1)] || 0;
}

function run(mode) {
    const systems = Array.from({length: count}, () => {
        const params = createOtterParameters();
        const plant = mode === "coupled6"
            ? new CoupledSixPlant(params, [new ActuatorModel(params)])
            : new DynamicsCore(params, [new ActuatorModel(params), new AddedMassCoriolis(), new HydrodynamicDamping()]);
        return {plant, state: RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0)};
    });
    const latencies = [];
    const start = performance.now();
    for (let i = 0; i < steps; i += 1) {
        const tick = performance.now();
        systems.forEach(({plant, state}) => plant.step(state, env, command, dt, i * dt));
        latencies.push(performance.now() - tick);
    }
    const elapsedMs = performance.now() - start;
    return {
        mode,
        workload: "dynamics-only",
        vessels: count,
        stepsPerVessel: steps,
        averageFleetTickMs: elapsedMs / steps,
        p95FleetTickMs: percentile(latencies, 0.95),
        plantStepsPerSecond: count * steps / (elapsedMs / 1000)
    };
}

console.log(JSON.stringify({results: ["planar3", "coupled6"].map(run)}, null, 2));
