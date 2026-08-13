import {performance} from "node:perf_hooks";
import {createDemoScenario} from "../scenarioPresets.js";
import {simulator} from "../schema.js";

function numberArg(name, fallback) {
    const idx = process.argv.indexOf(name);
    const value = idx >= 0 ? Number(process.argv[idx + 1]) : fallback;
    if (!Number.isInteger(value) || value <= 0) throw new Error(`${name} must be a positive integer.`);
    return value;
}

function stringArg(name, fallback) {
    const idx = process.argv.indexOf(name);
    return idx >= 0 ? process.argv[idx + 1] : fallback;
}

function percentile(values, p) {
    const sorted = [...values].sort((a, b) => a - b);
    return sorted[Math.min(Math.floor(sorted.length * p), sorted.length - 1)] || 0;
}

function makeSim(mode) {
    const sim = new simulator(createDemoScenario({physicsMode: mode, logEvery: 1e9}));
    sim.durationSec = Infinity;
    sim.goalModel.updateMissionProgress = () => {};
    sim.updateFailureState = () => {};
    return sim;
}

function runMode(mode, steps, vesselCount) {
    const sims = Array.from({length: vesselCount}, () => makeSim(mode));
    for (let warm = 0; warm < 20; warm += 1) sims.forEach((sim) => sim.step());
    const latencies = [];
    const startMemory = process.memoryUsage().heapUsed;
    const start = performance.now();
    for (let i = 0; i < steps; i += 1) {
        const tickStart = performance.now();
        sims.forEach((sim) => sim.step());
        latencies.push(performance.now() - tickStart);
    }
    const elapsedMs = performance.now() - start;
    const endMemory = process.memoryUsage().heapUsed;
    const totalSteps = steps * vesselCount;
    const simulatedSeconds = steps * sims[0].stepTime;
    return {
        mode,
        workload: "complete-headless-loop",
        vessels: vesselCount,
        stepsPerVessel: steps,
        totalPlantSteps: totalSteps,
        elapsedMs,
        fleetTickAverageMs: elapsedMs / steps,
        fleetTickP95Ms: percentile(latencies, 0.95),
        plantStepsPerSecond: totalSteps / (elapsedMs / 1000),
        simulatedSecondsPerWallSecond: simulatedSeconds / (elapsedMs / 1000),
        heapDeltaBytes: endMemory - startMemory
    };
}

try {
    const steps = numberArg("--steps", 1000);
    const vessels = numberArg("--vessels", 1);
    const requested = stringArg("--mode", "both");
    const modes = requested === "both" ? ["planar3", "coupled6"] : [requested];
    if (modes.some((mode) => mode !== "planar3" && mode !== "coupled6")) {
        throw new Error("--mode must be planar3, coupled6, or both.");
    }
    console.log(JSON.stringify({
        generatedAt: new Date().toISOString(),
        node: process.version,
        results: modes.map((mode) => runMode(mode, steps, vessels))
    }, null, 2));
}
catch (error) {
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
