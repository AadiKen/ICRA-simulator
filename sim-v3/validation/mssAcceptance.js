import fs from "node:fs";
import path from "node:path";
import {fileURLToPath} from "node:url";
import {replayManeuver} from "./goldenLogCompare.js";

const root = path.dirname(fileURLToPath(import.meta.url));
const config = JSON.parse(fs.readFileSync(path.join(root, "mss-reference.json"), "utf8"));

function parseCsv(file) {
    const lines = fs.readFileSync(file, "utf8").split(/\r?\n/).filter((line) => line && !line.startsWith("#"));
    const header = lines.shift().split(",");
    return lines.map((line) => Object.fromEntries(line.split(",").map((value, i) => [header[i], Number(value)])));
}

function angleDelta(a, b) {
    return Math.atan2(Math.sin(a - b), Math.cos(a - b));
}

function rms(values) {
    return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / Math.max(values.length, 1));
}

function compare(name) {
    const file = path.join(root, "mss-golden", `${name}.csv`);
    if (!fs.existsSync(file)) {
        throw new Error(`Missing pinned MSS golden trace: ${file}. Generate it from MSS commit ${config.commit}.`);
    }
    const reference = parseCsv(file);
    const actual = replayManeuver(name);
    const n = Math.min(reference.length, actual.length);
    if (reference.length !== actual.length || n === 0) {
        throw new Error(`${name}: sample count mismatch (MSS=${reference.length}, simulator=${actual.length}).`);
    }
    const position = [];
    const heading = [];
    const speed = [];
    for (let i = 0; i < n; i += 1) {
        const ref = reference[i];
        const got = actual[i];
        position.push(Math.hypot(got.N - ref.N, got.E - ref.E));
        heading.push(angleDelta(got.yaw, ref.yaw));
        speed.push(Math.hypot(got.u - ref.u, got.v - ref.v));
    }
    const metrics = {
        positionRmse: rms(position),
        headingRmse: rms(heading),
        headingRmseDegrees: rms(heading) * 180 / Math.PI,
        bodySpeedRmse: rms(speed),
        maxPositionError: Math.max(...position),
        maxHeadingErrorDegrees: Math.max(...heading.map(Math.abs)) * 180 / Math.PI,
        maxBodySpeedError: Math.max(...speed)
    };
    const limits = config.acceptance;
    const passed = metrics.positionRmse <= limits.positionRmseMeters &&
        metrics.headingRmse <= limits.headingRmseDegrees * Math.PI / 180 &&
        metrics.bodySpeedRmse <= limits.bodySpeedRmseMetersPerSecond;
    return {name, samples: n, metrics, passed};
}

try {
    const results = config.maneuvers.map(compare);
    console.log(JSON.stringify({referenceCommit: config.commit, results}, null, 2));
    if (results.some((result) => !result.passed)) process.exitCode = 1;
}
catch (error) {
    console.error(error.message);
    process.exitCode = 1;
}
