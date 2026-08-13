import {SensorStreamPublisher} from "../sensorStreamPublisher.js";
import {createDemoScenario} from "../scenarioPresets.js";
import {simulator} from "../schema.js";

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

function approx(value, expected, tolerance, message) {
    assert(Math.abs(value - expected) <= tolerance, `${message}: expected ${expected}, got ${value}`);
}

function testControllerRunsAtExactDeadlines() {
    const sim = new simulator(createDemoScenario());
    const commandTimes = [];
    sim.controlModel.step = (_observation, state) => {
        commandTimes.push(state.time);
        return {activeSensors: []};
    };

    sim.runSteps(13);
    assert(commandTimes.length === 3, `Expected commands at 0, 0.5, and 1.0 seconds; got ${commandTimes.length}.`);
    const startTime = commandTimes[0];
    [0, 0.5, 1].forEach((offset, idx) => {
        approx(commandTimes[idx], startTime + offset, 1e-9, "Controller deadline mismatch.");
    });
}

function testLongRunClockUsesIntegerSteps() {
    const scenario = createDemoScenario();
    scenario.simConfig.simHz = 50;
    scenario.simConfig.durationSec = 300;
    const sim = new simulator(scenario);
    sim.goalModel.updateMissionProgress = () => {};
    sim.updateFailureState = () => {};
    sim.runUntilDone();
    assert(sim.state.steps === 15000, `Expected exactly 15000 steps, got ${sim.state.steps}.`);
    assert(sim.state.time === sim.state.startTime + 15000 * 0.02, "Time must be derived from the integer step index.");
    assert(sim.state.stopReason === "duration_elapsed", "Integer duration budget must terminate the run.");
}

function testSensorCadenceUsesSampleIndices() {
    const scenario = createDemoScenario();
    scenario.simConfig.simHz = 50;
    scenario.simConfig.durationSec = 30;
    const gps = scenario.sensorConfig.sensors.find((sensor) => sensor.id === "gps");
    gps.hz = 7;
    scenario.sensorConfig.sensors = [gps];
    const sim = new simulator(scenario);
    sim.goalModel.updateMissionProgress = () => {};
    sim.updateFailureState = () => {};
    sim.state.activeSensors = ["gps", "GPS"];
    for (let i = 0; i < 1500; i += 1) sim.step({controlCommand: {activeSensors: ["gps", "GPS"]}});
    assert(sim.state.sensors.sampleCounts.gps === 210, `Expected 210 indexed samples, got ${sim.state.sensors.sampleCounts.gps}.`);
}

function testPublisherDisposeClosesEverySocket() {
    const publisher = new SensorStreamPublisher({enabled: true});
    const sockets = Object.keys(publisher.sockets).map(() => ({
        closeCount: 0,
        close() {
            this.closeCount += 1;
        }
    }));
    [publisher.sockets.camera, publisher.sockets.lidar, publisher.sockets.telemetry] = sockets;
    publisher.connectAttempted = true;
    publisher.lastSent.set("gps", "gps:1");

    publisher.dispose();

    sockets.forEach((socket) => assert(socket.closeCount === 1, "dispose() must close each socket exactly once."));
    assert(Object.values(publisher.sockets).every((socket) => socket === null), "dispose() must release socket references.");
    assert(!publisher.connectAttempted, "dispose() must allow a future reconnect.");
    assert(publisher.lastSent.size === 0, "dispose() must clear transmission bookkeeping.");
}

const tests = [
    testControllerRunsAtExactDeadlines,
    testLongRunClockUsesIntegerSteps,
    testSensorCadenceUsesSampleIndices,
    testPublisherDisposeClosesEverySocket
];

try {
    const results = tests.map((test) => {
        test();
        return test.name;
    });
    console.log("Runtime lifecycle tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
}
catch (error) {
    console.error("Runtime lifecycle tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
