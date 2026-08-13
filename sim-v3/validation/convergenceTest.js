import {RigidBodyState} from "../core/rigidBodyState.js";
import {DynamicsCore} from "../core/dynamicsCore.js";
import {ActuatorModel} from "../core/forces/actuatorModel.js";
import {AddedMassCoriolis} from "../core/forces/addedMassCoriolis.js";
import {HydrodynamicDamping} from "../core/forces/hydrodynamicDamping.js";
import {createOtterParameters} from "../core/vehicles/otter.js";
import {getParityManeuver, listParityManeuverNames} from "../gazebo/parityManeuvers.js";

function assert(condition, message, details = {}) {
    if (!condition) {
        const error = new Error(message);
        error.details = details;
        throw error;
    }
}

function commandAtFromSpec(spec) {
    const command = spec.command || {type: "constant", surgeForce: 0, differentialForce: 0};
    if (command.type === "zigZag") {
        return (t) => ({
            surgeForce: command.surgeForce || 0,
            differentialForce: (Math.floor(t / command.periodSec) % 2 === 0 ? 1 : -1) * (command.differentialForce || 0)
        });
    }
    if (command.type === "coastDown") {
        return (t) => t < command.thrustDurationSec
            ? {surgeForce: command.surgeForce || 0, differentialForce: command.differentialForce || 0}
            : {surgeForce: 0, differentialForce: 0};
    }
    return () => ({
        surgeForce: command.surgeForce || 0,
        differentialForce: command.differentialForce || 0
    });
}

function buildCore(params = createOtterParameters()) {
    return new DynamicsCore(params, [
        new ActuatorModel(params),
        new AddedMassCoriolis(),
        new HydrodynamicDamping()
    ]);
}

function runManeuverAtDt(spec, dt) {
    const core = buildCore();
    const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);
    const commandAt = commandAtFromSpec(spec);
    const env = {
        waterV: spec.env?.waterV || {x: 0, y: 0, z: 0},
        hullWaterSamples: []
    };
    const duration = spec.dt * spec.steps;
    const steps = Math.round(duration / dt);
    const samples = [];

    for (let i = 0; i < steps; i += 1) {
        const t = i * dt;
        core.step(state, env, commandAt(t), dt, t);
        samples.push({
            t,
            N: state.position.N,
            E: state.position.E,
            yaw: state.eulerAngles.yaw,
            u: state.velocity.u,
            v: state.velocity.v,
            r: state.angularRate.r
        });
    }
    return samples;
}

function sampleFineAtCoarseTimes(fine, ratio) {
    return fine.filter((_, idx) => (idx + 1) % ratio === 0);
}

function rmsDelta(coarse, fineAligned, columns) {
    const count = Math.min(coarse.length, fineAligned.length);
    const rms = {};
    columns.forEach((column) => {
        let sumSq = 0;
        for (let i = 0; i < count; i += 1) {
            const delta = (coarse[i][column] || 0) - (fineAligned[i][column] || 0);
            sumSq += delta * delta;
        }
        rms[column] = Math.sqrt(sumSq / Math.max(count, 1));
    });
    return rms;
}

function maxAbsDelta(coarse, fineAligned, columns) {
    const count = Math.min(coarse.length, fineAligned.length);
    const max = {};
    columns.forEach((column) => {
        let value = 0;
        for (let i = 0; i < count; i += 1) {
            value = Math.max(value, Math.abs((coarse[i][column] || 0) - (fineAligned[i][column] || 0)));
        }
        max[column] = value;
    });
    return max;
}

export function convergenceStudy(name, dtScale = 1) {
    const spec = getParityManeuver(name);
    const dt = spec.dt * dtScale;
    const fineDt = dt / 2;
    const coarse = runManeuverAtDt(spec, dt);
    const fine = runManeuverAtDt(spec, fineDt);
    const fineAligned = sampleFineAtCoarseTimes(fine, 2);
    const columns = ["N", "E", "yaw", "u", "v", "r"];
    const rms = rmsDelta(coarse, fineAligned, columns);
    const max = maxAbsDelta(coarse, fineAligned, columns);
    const tolerance = {
        N: (spec.tolerances?.N || 0.5) * 0.25,
        E: (spec.tolerances?.E || 0.5) * 0.25,
        yaw: (spec.tolerances?.yaw || 0.1) * 0.25,
        u: 0.08,
        v: 0.08,
        r: 0.08
    };
    const passed = columns.every((column) => rms[column] <= tolerance[column]);
    return {
        maneuver: name,
        dt,
        fineDt,
        samples: Math.min(coarse.length, fineAligned.length),
        rms,
        max,
        tolerance,
        passed
    };
}

function parseArgs(argv) {
    const args = {maneuver: null};
    for (let i = 2; i < argv.length; i += 1) {
        if (argv[i] === "--maneuver") {
            args.maneuver = argv[i + 1] || null;
            i += 1;
        }
    }
    return args;
}

if (process.argv[1] && process.argv[1].endsWith("convergenceTest.js")) {
    const args = parseArgs(process.argv);
    const maneuvers = args.maneuver ? [args.maneuver] : listParityManeuverNames();
    const results = maneuvers.map((name) => convergenceStudy(name));
    const failed = results.filter((result) => !result.passed);
    if (failed.length) {
        console.error("Convergence tests failed.");
        console.error(JSON.stringify({failed, results}, null, 2));
        process.exitCode = 1;
    }
    else {
        console.log("Convergence tests passed.");
        console.log(JSON.stringify({results}, null, 2));
    }
}
