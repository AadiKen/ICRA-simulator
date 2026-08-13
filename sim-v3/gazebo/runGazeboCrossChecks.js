import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import {capture} from "./captureGazeboLog.js";
import {replayManeuver} from "../validation/goldenLogCompare.js";

const maneuvers = ["constant-thrust", "turning-circle", "yaw-turn", "zig-zag", "coast-down", "current-drift"];
const outputRoot = path.resolve(process.argv[2] || "artifacts/gazebo/live-cross-check");

function sha(file) {
    return crypto.createHash("sha256").update(fs.readFileSync(file)).digest("hex");
}

function relative(file) {
    return file ? path.relative(process.cwd(), file) : null;
}

function portableRuntime(runtime) {
    return {
        ...runtime,
        world: relative(runtime.world),
        out: relative(runtime.out),
        wrenchLog: relative(runtime.wrenchLog)
    };
}

function loadCsv(file) {
    const [header, ...lines] = fs.readFileSync(file, "utf8").trim().split(/\r?\n/);
    const keys = header.split(",");
    return lines.map((line) => Object.fromEntries(line.split(",").map((value, idx) => [keys[idx], Number(value)])));
}

function angleDelta(a, b) {
    return Math.atan2(Math.sin(a - b), Math.cos(a - b));
}

function compare(nodeRows, gazeboRows, tolerances) {
    const count = Math.min(nodeRows.length, gazeboRows.length);
    const fields = ["N", "E", "yaw"];
    const rmse = {}, maxAbs = {};
    for (const field of fields) {
        let sum = 0, maximum = 0;
        for (let idx = 0; idx < count; idx += 1) {
            const delta = field === "yaw"
                ? angleDelta(nodeRows[idx][field] || 0, gazeboRows[idx][field] || 0)
                : (nodeRows[idx][field] || 0) - (gazeboRows[idx][field] || 0);
            sum += delta * delta;
            maximum = Math.max(maximum, Math.abs(delta));
        }
        rmse[field] = Math.sqrt(sum / Math.max(count, 1));
        maxAbs[field] = maximum;
    }
    return {
        count,
        rowCountMatch: count === gazeboRows.length,
        rmse,
        maxAbs,
        tolerances,
        passed: fields.every((field) => rmse[field] <= tolerances[field])
    };
}

async function main() {
    fs.mkdirSync(outputRoot, {recursive: true});
    const results = [];
    for (const maneuver of maneuvers) {
        const manifestPath = `gazebo/generated/manifests/otter_${maneuver}.json`;
        const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
        const csv = path.join(outputRoot, `${maneuver}.csv`);
        const wrenchLog = path.join(outputRoot, `${maneuver}-wrenches.csv`);
        const runtime = await capture({
            manifest: manifestPath,
            out: csv,
            gzBin: "gz",
            launch: true,
            dryRun: false,
            diagnose: false,
            topic: null,
            timeoutSec: 30,
            wrenchLog,
            rawOut: null
        });
        const gazeboRows = loadCsv(csv);
        const comparison = compare(replayManeuver(maneuver, undefined, {env: manifest.env}).slice(0, gazeboRows.length), gazeboRows, manifest.tolerances);
        results.push({
            maneuver,
            passed: comparison.passed,
            runtime: portableRuntime(runtime),
            comparison,
            artifacts: {
                trajectory: {path: relative(csv), sha256: sha(csv)},
                wrenchLog: {path: relative(wrenchLog), sha256: sha(wrenchLog)}
            }
        });
    }
    const artifact = {
        schema_version: 1,
        artifact_kind: "gazebo-harmonic-live-cross-check",
        status: results.every((result) => result.passed) ? "passed-planar-implementation-cross-check" : "failed",
        gazebo_version: "8.14.0",
        evidence_scope: "Planar maneuver implementation cross-check using a stabilized primitive-buoyancy fixture.",
        is_coupled6_hydrostatics_validation: false,
        supersedes: {
            artifact: "artifacts/external-runtime/2026-08-02-cross-checks.json",
            artifact_sha256: "e660292ef32461dea543e91b82ae95f5e33e9f5fd0a9e45d7b0db4da2f8105df",
            check_id: "gazebo-harmonic-live-constant-thrust",
            reason: "The retained failed campaign exposed invalid uniform-fluid buoyancy and non-isolated Gazebo Transport processes; this campaign executes the repaired fixture."
        },
        limitations: [
            "The fixture reflects the vertical CG below the primitive buoyancy center for stability.",
            "Gazebo primitive buoyancy does not reproduce the Otter metacentric hydrostatics.",
            "Gazebo is an implementation cross-check, not an independent hydrodynamic oracle."
        ],
        results
    };
    const report = path.join(outputRoot, "report.json");
    fs.writeFileSync(report, `${JSON.stringify(artifact, null, 2)}\n`);
    console.log(JSON.stringify({
        report,
        status: artifact.status,
        results: results.map(({maneuver, passed, comparison}) => ({maneuver, passed, rmse: comparison.rmse}))
    }, null, 2));
    if (artifact.status === "failed") process.exitCode = 1;
}

main().catch((error) => {
    console.error(error.stack || error.message);
    process.exitCode = 1;
});
