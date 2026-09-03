import fs from "node:fs";
import path from "node:path";
import {nedWorldToBody2D} from "../core/frames.js";
import {enuToNed, yawEnuToNed} from "../core/frameAdapters.js";
import {VehicleParameters} from "../core/vehicleParameters.js";
import {ActuationModel} from "../packages/core/src/actuators.js";
import {bcodUsvCoefficients, otterCoefficients} from "../core/vehicles/coefficients.js";

export const captureColumns = ["t", "N", "E", "yaw", "u", "v", "r"];

const coefficientSets = {
    bcod_usv: bcodUsvCoefficients,
    otter: otterCoefficients
};

export function loadManifest(manifestPath) {
    const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
    return {
        ...manifest,
        manifestPath,
        manifestDir: path.dirname(manifestPath)
    };
}

export function coefficientSetForVehicle(vehicle) {
    const coeffs = coefficientSets[vehicle];
    if (!coeffs) {
        throw new Error(`No coefficient set registered for vehicle '${vehicle}'.`);
    }
    return coeffs;
}

export function commandAt(command = {}, t = 0) {
    const withYawMoment = (output) => Number.isFinite(command.yawMoment)
        ? {...output, yawMoment: command.yawMoment}
        : output;
    if (command.type === "zigZag") {
        return withYawMoment({
            surgeForce: command.surgeForce || 0,
            differentialForce: (Math.floor(t / command.periodSec) % 2 === 0 ? 1 : -1) * (command.differentialForce || 0)
        });
    }
    if (command.type === "coastDown") {
        return t < command.thrustDurationSec
            ? withYawMoment({surgeForce: command.surgeForce || 0, differentialForce: command.differentialForce || 0})
            : withYawMoment({surgeForce: 0, differentialForce: 0});
    }
    return withYawMoment({
        surgeForce: command.surgeForce || 0,
        differentialForce: command.differentialForce || 0
    });
}

export function effectorCommandsForTime(manifest, coeffs = coefficientSetForVehicle(manifest.vehicle), t = 0) {
    const params = VehicleParameters.fromCoefficientSet(coeffs);
    const actuation = new ActuationModel(params);
    actuation.commandWrench(commandAt(manifest.command, t), 0);
    const byEffector = actuation.lastEffectorCommands;
    const topics = manifest.gazebo?.commandTopics || [];
    return (coeffs.effectors || []).map((effector, idx) => {
        const command = byEffector[effector.id] || {};
        return {
            id: effector.id,
            type: effector.type,
            topic: topics[idx] || `/model/${coeffs.id}joint/${effector.id}_joint/cmd_thrust`,
            value: command.thrust ?? command.command ?? command.deflection ?? command.speed ?? 0
        };
    });
}

export function bodyWrenchCommandForTime(manifest, coeffs = coefficientSetForVehicle(manifest.vehicle), t = 0) {
    const params = VehicleParameters.fromCoefficientSet(coeffs);
    const actuation = new ActuationModel(params);
    const wrench = actuation.commandWrench(commandAt(manifest.command, t), 0);
    return {
        id: "plant-wrench",
        type: "BodyWrench",
        topic: manifest.gazebo?.wrenchPersistentTopic || manifest.gazebo?.wrenchTopic || manifest.gazebo?.commandTopics?.[0],
        messageType: "gz.msgs.EntityWrench",
        value: {
            surge: wrench[0] || 0,
            sway: wrench[1] || 0,
            yaw: wrench[2] || 0
        }
    };
}

export function commandSchedule(manifest, coeffs = coefficientSetForVehicle(manifest.vehicle)) {
    if (Array.isArray(manifest.fixedCommandSchedule)) {
        return manifest.fixedCommandSchedule;
    }
    const rows = [];
    for (let i = 0; i < manifest.steps; i += 1) {
        const t = i * manifest.dt;
        const commands = manifest.gazebo?.actuationMode === "bodyWrench"
            ? [bodyWrenchCommandForTime(manifest, coeffs, t)]
            : effectorCommandsForTime(manifest, coeffs, t);
        rows.push({t, commands});
    }
    return rows;
}

export function quaternionYaw(q) {
    const w = q.w ?? 1;
    const x = q.x ?? 0;
    const y = q.y ?? 0;
    const z = q.z ?? 0;
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

export function gazeboPoseToBcodSample(pose, t = 0) {
    const ned = enuToNed({
        x: pose.position?.x || 0,
        y: pose.position?.y || 0,
        z: pose.position?.z || 0
    });
    const yaw = yawEnuToNed(quaternionYaw(pose.orientation || {w: 1, x: 0, y: 0, z: 0}));
    return {
        t,
        N: ned.N,
        E: ned.E,
        yaw,
        u: 0,
        v: 0,
        r: 0
    };
}

export function unwrapAngleDelta(next, prev) {
    let delta = next - prev;
    while (delta > Math.PI) {
        delta -= 2 * Math.PI;
    }
    while (delta < -Math.PI) {
        delta += 2 * Math.PI;
    }
    return delta;
}

export function deriveBodyVelocities(samples) {
    return samples.map((sample, idx) => {
        if (idx === 0) {
            return {...sample, u: 0, v: 0, r: 0};
        }
        const prev = samples[idx - 1];
        const dt = Math.max(sample.t - prev.t, 1e-9);
        const dN = (sample.N - prev.N) / dt;
        const dE = (sample.E - prev.E) / dt;
        const body = nedWorldToBody2D(dN, dE, sample.yaw);
        return {
            ...sample,
            u: body.u,
            v: body.v,
            r: unwrapAngleDelta(sample.yaw, prev.yaw) / dt
        };
    });
}

export function normalizeSamplesToInitialFrame(samples, expectedYaw = 0) {
    if (!samples.length) {
        return [];
    }
    const first = samples[0];
    const yawOffset = unwrapAngleDelta(first.yaw, expectedYaw);
    const cos = Math.cos(-yawOffset);
    const sin = Math.sin(-yawOffset);
    return samples.map((sample) => {
        const dN = sample.N - first.N;
        const dE = sample.E - first.E;
        return {
            ...sample,
            N: dN * cos - dE * sin,
            E: dN * sin + dE * cos,
            yaw: wrapAngle(sample.yaw - yawOffset)
        };
    });
}

export function wrapAngle(angle) {
    let value = angle;
    while (value > Math.PI) {
        value -= 2 * Math.PI;
    }
    while (value <= -Math.PI) {
        value += 2 * Math.PI;
    }
    return value;
}

export function resampleSamples(samples, dt, steps) {
    if (!samples.length) {
        return [];
    }
    const sorted = samples.slice().sort((a, b) => a.t - b.t);
    const output = [];
    for (let i = 0; i < steps; i += 1) {
        const t = i * dt;
        let hi = sorted.findIndex((sample) => sample.t >= t);
        if (hi < 0) {
            hi = sorted.length - 1;
        }
        const lo = Math.max(hi - 1, 0);
        const a = sorted[lo];
        const b = sorted[hi];
        const span = Math.max((b.t - a.t), 1e-9);
        const f = Math.min(Math.max((t - a.t) / span, 0), 1);
        output.push({
            t,
            N: a.N + (b.N - a.N) * f,
            E: a.E + (b.E - a.E) * f,
            yaw: a.yaw + unwrapAngleDelta(b.yaw, a.yaw) * f,
            u: 0,
            v: 0,
            r: 0
        });
    }
    return deriveBodyVelocities(normalizeSamplesToInitialFrame(output, 0));
}

export function samplesToCsv(samples) {
    const rows = [captureColumns.join(",")];
    samples.forEach((sample) => {
        rows.push(captureColumns.map((column) => Number(sample[column] || 0).toPrecision(12)).join(","));
    });
    return `${rows.join("\n")}\n`;
}

export function parseGazeboPoseVectorText(text, modelName) {
    const modelPoses = [];
    const poseRegex = /pose\s*\{([\s\S]*?)(?=\n\s*pose\s*\{|\n\s*\}\s*$|$)/g;
    let match;
    while ((match = poseRegex.exec(text)) !== null) {
        const block = match[1];
        const name = readString(block, "name");
        if (name !== modelName) {
            continue;
        }
        const position = readVectorBlock(block, "position");
        const orientation = readQuaternionBlock(block, "orientation");
        if (position) {
            const pose = {name, position, orientation: orientation || {w: 1, x: 0, y: 0, z: 0}};
            modelPoses.push(pose);
        }
    }
    return modelPoses;
}

function readString(block, key) {
    const match = new RegExp(`${key}:\\s*"([^"]+)"`).exec(block);
    return match ? match[1] : null;
}

function readVectorBlock(block, key) {
    const match = new RegExp(`${key}\\s*\\{([\\s\\S]*?)\\}`).exec(block);
    if (!match) {
        return null;
    }
    return {
        x: readNumber(match[1], "x"),
        y: readNumber(match[1], "y"),
        z: readNumber(match[1], "z")
    };
}

function readQuaternionBlock(block, key) {
    const match = new RegExp(`${key}\\s*\\{([\\s\\S]*?)\\}`).exec(block);
    if (!match) {
        return null;
    }
    return {
        w: readNumber(match[1], "w", 1),
        x: readNumber(match[1], "x"),
        y: readNumber(match[1], "y"),
        z: readNumber(match[1], "z")
    };
}

function readNumber(block, key, fallback = 0) {
    const match = new RegExp(`${key}:\\s*([-+0-9.eE]+)`).exec(block);
    return match ? Number(match[1]) : fallback;
}
