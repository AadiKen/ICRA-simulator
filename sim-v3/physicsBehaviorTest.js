let {
    scenarioConfig,
    simConfig,
    boatConfig,
    sensorConfig,
    gpsSensor,
    imuSensor,
    envConfig,
    visibility,
    waterFieldConfig,
    waveConfig,
    goalConfig,
    controlConfig,
    Zone,
    guidanceObj,
    headingForwardVector,
    sensorWorldPose,
    simulator,
    vec3
} = {};

async function loadSchema() {
    const schema = await import(`./schema.js?test=${Date.now()}`);
    ({
        scenarioConfig,
        simConfig,
        boatConfig,
        sensorConfig,
        gpsSensor,
        imuSensor,
        envConfig,
        visibility,
        waterFieldConfig,
        waveConfig,
        goalConfig,
        controlConfig,
        Zone,
        guidanceObj,
        headingForwardVector,
        sensorWorldPose,
        simulator,
        vec3
    } = schema);
}

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function magnitude(v) {
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function horizontalSpeed(v) {
    return Math.sqrt(v.x * v.x + v.z * v.z);
}

function dot2(a, b) {
    return a.x * b.x + a.z * b.z;
}

function isFiniteVec(v) {
    return Number.isFinite(v.x) && Number.isFinite(v.y) && Number.isFinite(v.z);
}

function buildScenario({
    simHz = 30,
    durationSec = 30,
    waves = [],
    current = new vec3(0, 0, 0),
    strategy = "none",
    goal = new vec3(80, 0, 80),
    guidanceMode = "relative",
    sensorDict = {},
    deniedZones = [],
    favoredZones = []
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
        new sensorConfig(sensorDict),
        new envConfig(
            100,
            100,
            [],
            deniedZones,
            favoredZones,
            new waterFieldConfig(waves, current),
            new visibility(1, 1),
            "day"
        ),
        new goalConfig([goal], 1),
        new controlConfig("local", 5, strategy, 100, guidanceMode)
    );
}

function runSteps(sim, steps) {
    for (let i = 0; i < steps && sim.simStatus(); i += 1) {
        sim.step();
        assert(isFiniteVec(sim.state.boat.pos), "Boat position should remain finite.");
        assert(isFiniteVec(sim.state.boat.velocity), "Boat velocity should remain finite.");
        assert(isFiniteVec(sim.state.boat.orientation), "Boat orientation should remain finite.");
        assert(isFiniteVec(sim.state.boat.angularVel), "Boat angular velocity should remain finite.");
    }
}

function testStillWaterNoInputStaysStable() {
    const sim = new simulator(buildScenario({waves: []}));
    const start = {
        pos: {...sim.state.boat.pos},
        orientation: {...sim.state.boat.orientation}
    };

    runSteps(sim, 180);

    const end = sim.state.boat;
    assert(Math.abs(end.pos.x - start.pos.x) < 0.01, "Still water/no input should not drift in x.");
    assert(Math.abs(end.pos.z - start.pos.z) < 0.01, "Still water/no input should not drift in z.");
    assert(Math.abs(end.orientation.x - start.orientation.x) < 0.001, "Still water/no input should not roll.");
    assert(Math.abs(end.orientation.z - start.orientation.z) < 0.001, "Still water/no input should not pitch.");
    assert(magnitude(end.angularVel) < 0.001, "Still water/no input should not build angular velocity.");
}

function testYawDampingDecays() {
    const sim = new simulator(buildScenario({waves: []}));
    sim.state.boat.angularVel.y = 0.4;

    runSteps(sim, 90);

    assert(
        Math.abs(sim.state.boat.angularVel.y) < 0.08,
        "Yaw damping should reduce an initial yaw rate."
    );
}

function testRollRecovery() {
    const sim = new simulator(buildScenario({waves: []}));
    sim.state.boat.orientation.x = 0.12;

    runSteps(sim, 120);

    assert(
        Math.abs(sim.state.boat.orientation.x) < 0.04,
        "Roll restoring physics should bring roll back toward level."
    );
    assert(
        Math.abs(sim.state.boat.angularVel.x) < 0.04,
        "Roll angular velocity should settle after recovery."
    );
}

function testPitchRecovery() {
    const sim = new simulator(buildScenario({waves: []}));
    sim.state.boat.orientation.z = -0.12;

    runSteps(sim, 120);

    assert(
        Math.abs(sim.state.boat.orientation.z) < 0.04,
        "Pitch restoring physics should bring pitch back toward level."
    );
    assert(
        Math.abs(sim.state.boat.angularVel.z) < 0.04,
        "Pitch angular velocity should settle after recovery."
    );
}

function testForwardThrustReachesBoundedSpeed() {
    const sim = new simulator(buildScenario({waves: []}));
    const dt = sim.stepTime;

    for (let i = 0; i < 240; i += 1) {
        const localEnv = sim.envModel.getLocalSample(sim.state);
        sim.boatModel.updatePosEnv(sim.state.boat, localEnv, dt);
        sim.boatModel.updatePosGuidance(sim.state.boat, new guidanceObj(0, 0.6), dt);
        assert(isFiniteVec(sim.state.boat.pos), "Boat position should remain finite under thrust.");
    }

    const speed = horizontalSpeed(sim.state.boat.velocity);
    assert(speed > 0.5, "Forward thrust should accelerate the boat.");
    assert(speed < 5, "Forward thrust should remain bounded by damping and configured limits.");
}

function testForwardThrustMatchesHeadingConvention() {
    const sim = new simulator(buildScenario({waves: []}));
    const dt = sim.stepTime;

    sim.state.boat.heading = 0;
    sim.state.boat.orientation.y = 0;
    sim.boatModel.updatePosEnv(sim.state.boat, sim.envModel.getLocalSample(sim.state), dt);
    sim.boatModel.updatePosGuidance(sim.state.boat, new guidanceObj(0, 0.6), dt);

    assert(
        Math.abs(sim.state.boat.velocity.z) > Math.abs(sim.state.boat.velocity.x),
        "Heading 0 should thrust mostly along world +z, matching the boat mesh bow."
    );
    assert(
        sim.state.boat.velocity.z > 0,
        "Heading 0 should move the boat toward world +z."
    );
}

function testGuidanceAccelerationFollowsPhysicalBow() {
    const sim = new simulator(buildScenario({waves: []}));
    const dt = sim.stepTime;

    sim.state.boat.heading = Math.PI / 2;
    sim.state.boat.orientation.y = Math.PI / 2;
    sim.boatModel.updatePosEnv(sim.state.boat, sim.envModel.getLocalSample(sim.state), dt);
    sim.boatModel.updatePosGuidance(sim.state.boat, new guidanceObj(0, 0.6), dt);

    const bowForward = headingForwardVector(sim.state.boat.heading);
    const guidanceAcceleration = sim.state.boat.guidanceAcceleration;
    const accelerationMag = horizontalSpeed(guidanceAcceleration);
    const projected = dot2(guidanceAcceleration, bowForward);

    assert(accelerationMag > 0, "Guidance acceleration should be nonzero for positive thrust.");
    assert(
        Math.abs(projected - accelerationMag) < 1e-9,
        "Guidance acceleration should be exactly along the physical bow direction."
    );
    assert(
        Math.abs(guidanceAcceleration.z) < 1e-9 && guidanceAcceleration.x > 0,
        "Heading pi/2 should thrust toward world +x with no sideways thrust."
    );
}

function testSkipperTargetsWaypointWhenFarAway() {
    const sim = new simulator(buildScenario({
        waves: [],
        strategy: "heuristic",
        goal: new vec3(80, 0, 80)
    }));

    sim.updateBoatBelief();
    const command = sim.controlModel.step({}, sim.state);
    const guidance = sim.skipperModel.getGuidance(command, sim.state, sim.state.boatBelief);
    const expectedHeading = Math.atan2(
        command.waypoints[0].x - sim.state.boatBelief.pos.x,
        command.waypoints[0].z - sim.state.boatBelief.pos.z
    );

    assert(
        Math.abs(sim.skipperModel.normalizeAngle(guidance.desiredHeading - expectedHeading)) < 0.01,
        "When far from a waypoint, skipper should target the waypoint, not the current boat position."
    );
}

function testWaveResponseIsBounded() {
    const waves = [
        new waveConfig(35, 0.28, 11, 0.34, 0.45),
        new waveConfig(120, 0.14, 6, 0.24, 0.25)
    ];
    const sim = new simulator(buildScenario({waves}));
    let maxAngularVelocity = 0;
    let maxRoll = 0;
    let maxPitch = 0;

    for (let i = 0; i < 360 && sim.simStatus(); i += 1) {
        sim.step();
        maxAngularVelocity = Math.max(maxAngularVelocity, magnitude(sim.state.boat.angularVel));
        maxRoll = Math.max(maxRoll, Math.abs(sim.state.boat.orientation.x));
        maxPitch = Math.max(maxPitch, Math.abs(sim.state.boat.orientation.z));
    }

    assert(maxAngularVelocity < 0.25, "Wave response should not build excessive angular velocity.");
    assert(maxRoll < 0.12, "Wave response roll should stay bounded.");
    assert(maxPitch < 0.12, "Wave response pitch should stay bounded.");
}

function testWavesImpartTranslationalAcceleration() {
    const waves = [
        new waveConfig(35, 0.28, 11, 0.34, 0.45),
        new waveConfig(120, 0.14, 6, 0.24, 0.25)
    ];
    const sim = new simulator(buildScenario({waves}));
    let maxHorizontalEnvAcceleration = 0;
    let maxVerticalEnvAcceleration = 0;

    for (let i = 0; i < 90 && sim.simStatus(); i += 1) {
        const localEnv = sim.envModel.getLocalSample(sim.state);
        sim.boatModel.updatePosEnv(sim.state.boat, localEnv, sim.stepTime);
        sim.boatModel.updatePosGuidance(sim.state.boat, new guidanceObj(0, 0), sim.stepTime);
        maxHorizontalEnvAcceleration = Math.max(
            maxHorizontalEnvAcceleration,
            horizontalSpeed(sim.state.boat.environmentAcceleration)
        );
        maxVerticalEnvAcceleration = Math.max(
            maxVerticalEnvAcceleration,
            Math.abs(sim.state.boat.restoringAngularAcceleration.x) +
                Math.abs(sim.state.boat.restoringAngularAcceleration.z)
        );
        sim.state.steps += 1;
        sim.state.tick += 1;
        sim.state.time = sim.state.startTime + sim.state.steps * sim.stepTime;
    }

    assert(
        maxHorizontalEnvAcceleration > 0.001,
        "Waves should create nonzero horizontal environmental acceleration through force-space wave excitation and water-relative drag."
    );
    assert(
        maxVerticalEnvAcceleration > 0.001,
        "Waves should create nonzero presentation seakeeping response through hull-sampled surface normals."
    );
}

function testSkipperBrakesForTightWaypointArrival() {
    const sim = new simulator(buildScenario({
        waves: [],
        strategy: "heuristic",
        goal: new vec3(24, 0, 20)
    }));

    sim.state.boat.pos = new vec3(22.9, sim.state.boat.pos.y, 20);
    sim.state.boat.heading = 0;
    sim.state.boat.orientation.y = 0;
    sim.state.boat.velocity = new vec3(2.0, 0, 0);
    sim.updateBoatBelief();

    const command = sim.controlModel.step({}, sim.state);
    const guidance = sim.skipperModel.getGuidance(command, sim.state, sim.state.boatBelief);

    assert(
        guidance.a < 0,
        "Skipper should command braking when stopping distance exceeds the remaining waypoint distance."
    );
    assert(
        Math.abs(guidance.w) <= 1,
        "Skipper rudder command should stay normalized."
    );
}

function testWaypointReachUsesBoatFootprint() {
    const sim = new simulator(buildScenario({
        waves: [],
        strategy: "none",
        goal: new vec3(21.6, 0, 20)
    }));

    sim.goalModel.updateMissionProgress(sim.state.goal, sim.state.boat, sim.state.time);

    assert(
        sim.state.goal.completed,
        "Waypoint should register when it lies inside the boat footprint plus configured tolerance."
    );
}

function testTimestepConsistency() {
    const waves = [new waveConfig(45, 0.16, 9, 0.25, 0.25)];
    const sim12 = new simulator(buildScenario({simHz: 12, durationSec: 8, waves}));
    const sim24 = new simulator(buildScenario({simHz: 24, durationSec: 8, waves}));

    runSteps(sim12, 96);
    runSteps(sim24, 192);

    const posDelta = sim12.state.boat.pos.dist(sim24.state.boat.pos);
    const headingDelta = Math.abs(sim12.skipperModel.normalizeAngle(
        sim12.state.boat.heading - sim24.state.boat.heading
    ));

    assert(posDelta < 1.5, "Changing simHz should not drastically change final position.");
    assert(headingDelta < 0.2, "Changing simHz should not drastically change heading.");
}

function testAbsoluteGuidanceCopiesTrueBoatState() {
    const sim = new simulator(buildScenario({guidanceMode: "absolute"}));
    sim.state.boat.orientation.x = 0.11;
    sim.state.boat.orientation.z = -0.09;
    sim.state.boat.angularVel.x = 0.07;
    sim.state.boat.angularVel.z = -0.04;

    sim.updateBoatBelief();

    assert(
        sim.state.boatBelief.orientation.x === sim.state.boat.orientation.x,
        "Absolute guidance belief should copy true roll."
    );
    assert(
        sim.state.boatBelief.orientation.z === sim.state.boat.orientation.z,
        "Absolute guidance belief should copy true pitch."
    );
    assert(
        sim.state.boatBelief.angularVel.x === sim.state.boat.angularVel.x,
        "Absolute guidance belief should copy true roll rate."
    );
}

function testRelativeGuidanceIgnoresRollPitch() {
    const sim = new simulator(buildScenario({guidanceMode: "relative"}));
    sim.state.boat.orientation.x = 0.11;
    sim.state.boat.orientation.z = -0.09;
    sim.state.boat.angularVel.x = 0.07;
    sim.state.boat.angularVel.z = -0.04;

    sim.updateBoatBelief();

    assert(sim.state.boatBelief.orientation.x === 0, "Relative guidance should hide true roll.");
    assert(sim.state.boatBelief.orientation.z === 0, "Relative guidance should hide true pitch.");
    assert(sim.state.boatBelief.angularVel.x === 0, "Relative guidance should hide true roll rate.");
    assert(sim.state.boatBelief.angularVel.z === 0, "Relative guidance should hide true pitch rate.");
    assert(
        sim.state.boatBelief.heading === sim.state.boat.heading,
        "Relative guidance should keep a usable heading belief."
    );
}

function testSensorWorldPoseUsesBoatFrameMount() {
    const sim = new simulator(buildScenario({waves: []}));
    const sensor = {
        mountPosition: new vec3(0, 1, 2),
        mountOrientation: new vec3(0, 0, 0)
    };
    sim.state.boat.pos = new vec3(10, 0, 10);
    sim.state.boat.orientation = new vec3(0, Math.PI / 2, 0);
    sim.state.boat.heading = Math.PI / 2;

    const pose = sensorWorldPose(sensor, sim.state.boat);

    assert(
        Math.abs(pose.position.x - 12) < 1e-9 &&
        Math.abs(pose.position.y - 1) < 1e-9 &&
        Math.abs(pose.position.z - 10) < 1e-9,
        "Sensor mount position should rotate from boat frame into world frame."
    );
    assert(
        Math.abs(pose.forward.x - 1) < 1e-9 &&
        Math.abs(pose.forward.z) < 1e-9,
        "Sensor forward vector should follow boat bow direction."
    );
}

function testSensorFeedsExposeActiveOutputs() {
    const sim = new simulator(buildScenario({
        waves: [],
        strategy: "heuristic",
        sensorDict: {
            gps: new gpsSensor("GPS", 2, 0.25),
            imu: new imuSensor("IMU", 10, 0.1)
        }
    }));

    runSteps(sim, 3);
    const feeds = sim.getSensorFeeds();
    const gpsFeed = feeds.find((feed) => feed.type === "gps");
    const imuFeed = feeds.find((feed) => feed.type === "imu");

    assert(gpsFeed && gpsFeed.active, "GPS feed should be active under heuristic control.");
    assert(imuFeed && imuFeed.active, "IMU feed should be active under heuristic control.");
    assert(gpsFeed.displayType === "position", "GPS feed should use position display format.");
    assert(imuFeed.displayType === "motion", "IMU feed should use motion display format.");
}

function testDeniedZoneForcesSensorOff() {
    const deniedZone = new Zone(
        [
            new vec3(10, 0, 10),
            new vec3(30, 0, 10),
            new vec3(30, 0, 30),
            new vec3(10, 0, 30)
        ],
        "deniedZone",
        ["gps"]
    );
    const sim = new simulator(buildScenario({
        waves: [],
        strategy: "heuristic",
        deniedZones: [deniedZone],
        sensorDict: {
            gps: new gpsSensor("GPS", 2, 0.25),
            imu: new imuSensor("IMU", 10, 0.1)
        }
    }));

    runSteps(sim, 3);

    assert(!sim.state.activeSensors.includes("gps"), "Denied-zone sensors should be forced off.");
    assert(sim.state.activeSensors.includes("imu"), "Sensors not listed in the denied zone should remain available.");
}

const tests = [
    testStillWaterNoInputStaysStable,
    testYawDampingDecays,
    testRollRecovery,
    testPitchRecovery,
    testForwardThrustReachesBoundedSpeed,
    testForwardThrustMatchesHeadingConvention,
    testGuidanceAccelerationFollowsPhysicalBow,
    testSkipperTargetsWaypointWhenFarAway,
    testWaveResponseIsBounded,
    testWavesImpartTranslationalAcceleration,
    testSkipperBrakesForTightWaypointArrival,
    testWaypointReachUsesBoatFootprint,
    testTimestepConsistency,
    testAbsoluteGuidanceCopiesTrueBoatState,
    testRelativeGuidanceIgnoresRollPitch,
    testSensorWorldPoseUsesBoatFrameMount,
    testSensorFeedsExposeActiveOutputs,
    testDeniedZoneForcesSensorOff
];

loadSchema()
    .then(() => {
        const results = tests.map((test) => {
            test();
            return test.name;
        });

        console.log("Physics behavior tests passed.");
        console.log(JSON.stringify({tests: results}, null, 2));
    })
    .catch((error) => {
        console.error("Physics behavior tests failed.");
        console.error(error.stack || error.message);
        process.exitCode = 1;
    });
