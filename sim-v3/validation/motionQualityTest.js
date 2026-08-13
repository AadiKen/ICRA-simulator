import {
    scenarioConfig,
    simConfig,
    boatConfig,
    sensorConfig,
    envConfig,
    visibility,
    waterFieldConfig,
    waveConfig,
    goalConfig,
    controlConfig,
    simulator,
    vec3
} from "../schema.js";

function assert(condition, message, details = {}) {
    if (!condition) {
        const error = new Error(message);
        error.details = details;
        throw error;
    }
}

function buildScenario({
    waves = [],
    current = new vec3(0, 0, 0),
    strategy = "none",
    simHz = 30,
    durationSec = 30,
    goal = new vec3(80, 0, 80)
} = {}) {
    return new scenarioConfig(
        new simConfig(simHz, durationSec, 42, true, "planar3"),
        new boatConfig(
            2.5,
            0.9,
            0.9,
            0.55,
            0.08,
            0.12,
            new vec3(20, 0, 20),
            new vec3(0, 0, 0),
            new vec3(2.4, 1.0, 4.8),
            120,
            0.22,
            0.35,
            3.2,
            0.32,
            0.04,
            3.8,
            4.2,
            2.8,
            3.0,
            1.2,
            new vec3(0.08, 0, 0.08),
            0.04,
            new vec3(0.16, 0.48, 0.16),
            0.18,
            0.18
        ),
        new sensorConfig({}),
        new envConfig(
            100,
            100,
            [],
            [],
            [],
            new waterFieldConfig(waves, current),
            new visibility(1, 1),
            "day"
        ),
        new goalConfig([goal], 1),
        new controlConfig("local", 5, strategy, 100, "relative")
    );
}

function summarizeSeries(values) {
    const min = Math.min(...values);
    const max = Math.max(...values);
    const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
    const rms = Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length);
    let zeroCrossings = 0;
    let peaks = 0;

    for (let i = 1; i < values.length; i += 1) {
        const prev = values[i - 1];
        const current = values[i];
        if (Math.abs(prev) > 1e-5 && Math.abs(current) > 1e-5 && Math.sign(prev) !== Math.sign(current)) {
            zeroCrossings += 1;
        }
    }

    for (let i = 1; i < values.length - 1; i += 1) {
        const prevSlope = values[i] - values[i - 1];
        const nextSlope = values[i + 1] - values[i];
        if (Math.abs(values[i]) > 1e-4 && Math.sign(prevSlope) !== Math.sign(nextSlope)) {
            peaks += 1;
        }
    }

    return {
        min,
        max,
        mean,
        rms,
        range: max - min,
        maxAbs: Math.max(Math.abs(min), Math.abs(max)),
        zeroCrossings,
        peaks,
        leanRatio: (max - min) > 0 ? Math.abs(mean) / (max - min) : 0
    };
}

function collectMotion(sim, steps) {
    const samples = {
        t: [],
        pitch: [],
        roll: [],
        yaw: [],
        pitchRate: [],
        rollRate: [],
        yawRate: [],
        speed: []
    };

    for (let i = 0; i < steps && sim.simStatus(); i += 1) {
        sim.step();
        samples.t.push(sim.state.time);
        samples.pitch.push(sim.state.boat.orientation.x);
        samples.roll.push(sim.state.boat.orientation.z);
        samples.yaw.push(sim.state.boat.heading);
        samples.pitchRate.push(sim.state.boat.angularVel.x);
        samples.rollRate.push(sim.state.boat.angularVel.z);
        samples.yawRate.push(sim.state.boat.angularVel.y);
        samples.speed.push(Math.hypot(sim.state.boat.velocity.x, sim.state.boat.velocity.z));
    }
    return samples;
}

function testForcedAttitudeCorrectsInCalmWater() {
    const sim = new simulator(buildScenario({waves: []}));
    sim.state.boat.orientation.x = 0.12;
    sim.state.boat.orientation.z = -0.1;
    sim.state.boat.angularVel.x = 0;
    sim.state.boat.angularVel.z = 0;

    const samples = collectMotion(sim, 180);
    const finalPitch = samples.pitch[samples.pitch.length - 1];
    const finalRoll = samples.roll[samples.roll.length - 1];
    const pitchRate = samples.pitchRate[samples.pitchRate.length - 1];
    const rollRate = samples.rollRate[samples.rollRate.length - 1];
    const summary = {
        pitch: summarizeSeries(samples.pitch),
        roll: summarizeSeries(samples.roll),
        finalPitch,
        finalRoll,
        pitchRate,
        rollRate
    };

    assert(Math.abs(finalPitch) < 0.04, "Pitch should correct toward level in calm water.", summary);
    assert(Math.abs(finalRoll) < 0.04, "Roll should correct toward level in calm water.", summary);
    assert(Math.abs(pitchRate) < 0.04, "Pitch correction should settle rather than keep drifting.", summary);
    assert(Math.abs(rollRate) < 0.04, "Roll correction should settle rather than keep drifting.", summary);
    return summary;
}

function testRegularWavesRockInsteadOfOnlyLeaning() {
    const waves = [
        new waveConfig(35, 0.28, 11, 0.34, 0.45),
        new waveConfig(120, 0.14, 6, 0.24, 0.25)
    ];
    const sim = new simulator(buildScenario({waves, durationSec: 24}));
    const samples = collectMotion(sim, 720);
    const pitch = summarizeSeries(samples.pitch);
    const roll = summarizeSeries(samples.roll);
    const pitchRate = summarizeSeries(samples.pitchRate);
    const rollRate = summarizeSeries(samples.rollRate);
    const summary = {pitch, roll, pitchRate, rollRate};

    assert(pitch.range > 0.03, "Regular waves should create visible pitch motion.", summary);
    assert(roll.range > 0.025, "Regular waves should create visible roll motion.", summary);
    assert(pitch.zeroCrossings >= 1, "Pitch should cross level under regular waves, not just lean one way.", summary);
    assert(roll.zeroCrossings >= 1, "Roll should cross level under regular waves, not just lean one way.", summary);
    assert(pitch.peaks >= 2 || roll.peaks >= 2, "Wave response should have turning points that read as rocking.", summary);
    assert(pitch.maxAbs < 0.12, "Pitch should remain bounded under regular waves.", summary);
    assert(roll.maxAbs < 0.12, "Roll should remain bounded under regular waves.", summary);
    assert(Math.max(pitchRate.maxAbs, rollRate.maxAbs) < 0.25, "Rocking rates should remain visually clean.", summary);
    return summary;
}

function testGuidedRunMovesCleanlyWhileRocking() {
    const waves = [
        new waveConfig(35, 0.18, 11, 0.34, 0.4),
        new waveConfig(120, 0.10, 6, 0.24, 0.25)
    ];
    const sim = new simulator(buildScenario({waves, strategy: "heuristic", durationSec: 20}));
    const start = {x: sim.state.boat.pos.x, z: sim.state.boat.pos.z};
    const samples = collectMotion(sim, 600);
    const end = sim.state.boat.pos;
    const distanceMoved = Math.hypot(end.x - start.x, end.z - start.z);
    const speed = summarizeSeries(samples.speed);
    const yawRate = summarizeSeries(samples.yawRate);
    const pitch = summarizeSeries(samples.pitch);
    const roll = summarizeSeries(samples.roll);
    const summary = {distanceMoved, speed, yawRate, pitch, roll};

    assert(distanceMoved > 3, "Guided run should translate through the water, not only rock in place.", summary);
    assert(speed.max < 4, "Guided run should stay within a clean visual speed envelope.", summary);
    assert(yawRate.maxAbs < 1.4, "Guided run yaw rate should remain visually clean.", summary);
    assert(pitch.maxAbs < 0.12 && roll.maxAbs < 0.12, "Guided run rocking should remain bounded.", summary);
    return summary;
}

const tests = [
    testForcedAttitudeCorrectsInCalmWater,
    testRegularWavesRockInsteadOfOnlyLeaning,
    testGuidedRunMovesCleanlyWhileRocking
];

try {
    const summaries = {};
    tests.forEach((test) => {
        summaries[test.name] = test();
    });
    console.log("Motion quality tests passed.");
    console.log(JSON.stringify(summaries, null, 2));
} catch (error) {
    console.error("Motion quality tests failed.");
    console.error(error.message);
    if (error.details) {
        console.error(JSON.stringify(error.details, null, 2));
    }
    process.exitCode = 1;
}
