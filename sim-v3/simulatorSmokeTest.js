async function loadSchema() {
    return import(`./schema.js?test=${Date.now()}`);
}

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function nearlyEqual(a, b, epsilon = 1e-9) {
    return Math.abs(a - b) <= epsilon;
}

function buildScenario(schema) {
    const {
        scenarioConfig,
        simConfig,
        boatConfig,
        sensorConfig,
        imuSensor,
        gpsSensor,
        envConfig,
        visibility,
        waterFieldConfig,
        waveConfig,
        goalConfig,
        controlConfig,
        Obstacle,
        vec3
    } = schema;
    const sensorDict = {
        sensor_1: new gpsSensor("gps_1", 2, 0.25),
        sensor_2: new imuSensor("imu_1", 10, 0.1)
    };

    const waves = [
        new waveConfig(45, 0.25, 8, 0.4, 0.35)
    ];

    return new scenarioConfig(
        new simConfig(2, 20, 123, true, "planar3"),
        new boatConfig(
            2,
            0.75,
            0.75,
            0.4,
            0.05,
            0.1,
            new vec3(10, 0, 10),
            new vec3(0, 0, 0),
            new vec3(2, 1, 4),
            100,
            0.25,
            0.25,
            9.81,
            0.35
        ),
        new sensorConfig(sensorDict),
        new envConfig(
            100,
            100,
            [new Obstacle(new vec3(90, 0, 90), 2, true)],
            [],
            [],
            new waterFieldConfig(waves, new vec3(0.05, 0, 0.02)),
            new visibility(1, 1),
            "day"
        ),
        new goalConfig(
            [new vec3(40, 0, 40), new vec3(70, 0, 70)],
            1
        ),
        new controlConfig("local", 2, "heuristic", 100)
    );
}

async function runSmokeTest() {
    const schema = await loadSchema();
    const sim = new schema.simulator(buildScenario(schema));
    const initialTime = sim.state.time;
    const initialTick = sim.state.tick;
    const initialSteps = sim.state.steps;
    const initialPos = {
        x: sim.state.boat.pos.x,
        y: sim.state.boat.pos.y,
        z: sim.state.boat.pos.z
    };

    assert(sim.state.isSimulating === true, "Simulator should start active.");
    assert(sim.state.logs === undefined, "Logs should be owned by simulator, not simState.");
    assert(sim.logs.boatStates.length === 0, "Boat logs should start empty.");
    assert(sim.logs.metrics.length === 0, "Metric logs should start empty.");
    assert(sim.state.boat.rigidBody, "Boat should carry a core rigid body state.");

    sim.runSteps(3);

    assert(sim.state.steps === initialSteps + 3, "runSteps(3) should advance three steps.");
    assert(sim.state.tick === initialTick + 3, "Tick should advance once per step.");
    assert(
        nearlyEqual(sim.state.time, initialTime + 3 * sim.stepTime),
        "Time should advance by stepTime per step."
    );
    assert(sim.state.goal.failed === false, "Smoke scenario should not fail.");
    assert(sim.state.stopReason === null, "Smoke scenario should not have a stop reason yet.");
    assert(sim.logs.boatStates.length === sim.state.steps, "Boat log length should match steps.");
    assert(sim.logs.metrics.length === sim.state.steps, "Metric log length should match steps.");
    assert(sim.logs.sensorActivations.length === sim.state.steps, "Sensor activation log length should match steps.");

    const lastMetric = sim.logs.metrics[sim.logs.metrics.length - 1];
    assert(Number.isFinite(lastMetric.totalEnergy), "Metric log should include totalEnergy.");
    assert(Number.isFinite(lastMetric.lastTotalCost), "Metric log should include lastTotalCost.");
    assert(Number.isFinite(lastMetric.lastSpeed), "Metric log should include lastSpeed.");
    assert(sim.state.lastCommand !== null, "Controller should produce a command.");
    assert(Array.isArray(sim.state.lastCommand.waypoints), "Command should use canonical waypoints array.");
    assert(Array.isArray(sim.state.lastCommand.activeSensors), "Command should use canonical activeSensors array.");
    assert(sim.state.localEnv.waterSample, "Local env should include a 3D water sample.");
    assert(sim.state.localEnv.hullWaterSamples.length === 15, "Local env should include fifteen hull water samples.");
    assert(sim.state.localEnv.hullWaterSamples.every((sample) => sample.waterSample), "Each hull sample should include the full water sample.");

    const finalPos = sim.state.boat.pos;
    const moved = finalPos.x !== initialPos.x ||
        finalPos.y !== initialPos.y ||
        finalPos.z !== initialPos.z;
    assert(moved, "Boat position should change after stepping.");

    return {
        steps: sim.state.steps,
        tick: sim.state.tick,
        time: sim.state.time,
        stopReason: sim.state.stopReason,
        boatPos: sim.state.boat.pos,
        lastMetric,
        logs: {
            boatStates: sim.logs.boatStates.length,
            metrics: sim.logs.metrics.length,
            sensorActivations: sim.logs.sensorActivations.length
        }
    };
}

runSmokeTest()
    .then((result) => {
        console.log("Simulator smoke test passed.");
        console.log(JSON.stringify(result, null, 2));
    })
    .catch((error) => {
        console.error("Simulator smoke test failed.");
        console.error(error.stack || error.message);
        process.exitCode = 1;
    });
