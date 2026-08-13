import fs from "node:fs";
import {RigidBodyState} from "../core/rigidBodyState.js";
import {DynamicsCore} from "../core/dynamicsCore.js";
import {AddedMassCoriolis} from "../core/forces/addedMassCoriolis.js";
import {HydrodynamicDamping} from "../core/forces/hydrodynamicDamping.js";
import {CrossFlowDrag} from "../core/forces/crossFlowDrag.js";
import {ActuatorModel} from "../core/forces/actuatorModel.js";
import {createOtterParameters} from "../core/vehicles/otter.js";
import {constantThrustManeuver, runManeuver, turningCircleManeuver, yawTurnManeuver, zigZagManeuver} from "./maneuvers.js";

export function parseGoldenCsv(path) {
    const text = fs.readFileSync(path, "utf8");
    const lines = text.split(/\r?\n/).filter((line) => line.trim() && !line.startsWith("#"));
    const header = lines.shift().split(",");
    return lines.map((line) => {
        const values = line.split(",");
        return Object.fromEntries(header.map((key, idx) => [key.trim(), Number(values[idx])]));
    });
}

export function compareSamples(actual, expected, columns = ["N", "E", "yaw"], tolerances = {}) {
    const count = Math.min(actual.length, expected.length);
    const rms = {};
    columns.forEach((column) => {
        let sumSq = 0;
        for (let i = 0; i < count; i += 1) {
            const expectedValue = expected[i][column] ?? expected[i][column.toLowerCase()];
            const delta = (actual[i][column] || 0) - (expectedValue || 0);
            sumSq += delta * delta;
        }
        rms[column] = Math.sqrt(sumSq / Math.max(count, 1));
    });
    return {
        count,
        rms,
        passed: columns.every((column) => rms[column] <= (tolerances[column] ?? Infinity))
    };
}

export function replayDefaultOpenLoop(params = createOtterParameters()) {
    return replayManeuver("constant-thrust", params);
}

export function createOtterParametersForReplay(options = {}) {
    return options.disableAddedMass
        ? createOtterParameters({addedMass: {XuDot: 0, YvDot: 0, NrDot: 0, YrDot: 0, NvDot: 0}})
        : createOtterParameters();
}

export function replayManeuver(name = "constant-thrust", params = createOtterParameters(), options = {}) {
    const forceModels = [
        new ActuatorModel(params),
        ...(options.disableAddedMass ? [] : [new AddedMassCoriolis()]),
        new HydrodynamicDamping(),
        new CrossFlowDrag()
    ];
    const core = new DynamicsCore(params, forceModels);
    const maneuver = maneuverByName(name);
    return runManeuver(
        core,
        RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0),
        options.env || maneuver.env || {waterV: {x: 0, y: 0, z: 0}, hullWaterSamples: []},
        maneuver.commandAt,
        maneuver.dt,
        maneuver.steps
    );
}

export function maneuverByName(name) {
    if (name === "turning-circle") {
        return {commandAt: () => ({appliedWrench: [65, 0, 18.9]}), dt: 0.05, steps: 1200};
    }
    if (name === "yaw-turn") {
        return {commandAt: yawTurnManeuver(55, 1), dt: 0.05, steps: 180};
    }
    if (name === "zig-zag") {
        return {
            commandAt: (t) => ({appliedWrench: [60, 0, (Math.floor(t / 2) % 2 === 0 ? 1 : -1) * 15.12]}),
            dt: 0.05,
            steps: 1200
        };
    }
    if (name === "coast-down") {
        return {
            commandAt: (t) => ({appliedWrench: [t < 2 ? 60 : 0, 0, 0]}),
            dt: 0.05,
            steps: 1200
        };
    }
    if (name === "current-drift") {
        return {
            commandAt: constantThrustManeuver(0, 0),
            env: {waterV: {x: 0, y: 0, z: 0.3}, hullWaterSamples: []},
            dt: 0.05,
            steps: 1200
        };
    }
    return {
        commandAt: () => ({appliedWrench: [60, 0, 0]}),
        dt: 0.05,
        steps: 1200
    };
}

function parseArgs(argv) {
    const args = {maneuver: "constant-thrust", golden: null, disableAddedMass: false};
    for (let i = 2; i < argv.length; i += 1) {
        if (argv[i] === "--maneuver") {
            args.maneuver = argv[i + 1] || args.maneuver;
            i += 1;
        }
        else if (argv[i] === "--golden") {
            args.golden = argv[i + 1] || null;
            i += 1;
        }
        else if (argv[i] === "--disable-added-mass") {
            args.disableAddedMass = true;
        }
        else if (!args.golden) {
            args.golden = argv[i];
        }
    }
    return args;
}

if (process.argv[1] && process.argv[1].endsWith("goldenLogCompare.js")) {
    const args = parseArgs(process.argv);
    const params = createOtterParametersForReplay(args);
    const actual = replayManeuver(args.maneuver, params, args);
    if (!args.golden) {
        const last = actual[actual.length - 1];
        console.log(JSON.stringify({
            maneuver: args.maneuver,
            samples: actual.length,
            final: last
        }, null, 2));
        process.exit(0);
    }
    const expected = parseGoldenCsv(args.golden);
    const result = compareSamples(actual, expected, ["N", "E", "yaw"], {N: 0.5, E: 0.2, yaw: 0.1});
    console.log(JSON.stringify(result, null, 2));
    if (!result.passed) {
        process.exitCode = 1;
    }
}
