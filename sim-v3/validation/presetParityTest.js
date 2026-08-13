import {spawnSync} from "node:child_process";
import {createDemoScenario} from "../scenarioPresets.js";
import {boatModel} from "../schema.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function nearlyEqual(a, b, tolerance = 1e-9) {
    return Math.abs(a - b) <= tolerance;
}

function assertNearlyEqual(actual, expected, label, tolerance = 1e-9) {
    assert(
        nearlyEqual(actual, expected, tolerance),
        `${label} mismatch: simulator=${actual}, MSS=${expected}`
    );
}

function loadMssPreset() {
    const result = spawnSync("python3", ["mssReference.py", "--print_preset_json"], {
        cwd: new URL("..", import.meta.url).pathname,
        encoding: "utf8"
    });
    if (result.status !== 0) {
        throw new Error(`Failed to load MSS preset:\n${result.stderr || result.stdout}`);
    }
    return JSON.parse(result.stdout);
}

function runPresetParityTest() {
    const scenario = createDemoScenario();
    const dynamicsFacade = new boatModel(scenario.boatConfig);
    const vehicle = dynamicsFacade.vehicleParameters;
    const mss = loadMssPreset();
    const boat = scenario.boatConfig;

    assertNearlyEqual(scenario.simConfig.simHz, mss.simHz, "simHz");
    assertNearlyEqual(scenario.simConfig.durationSec, mss.durationSec, "durationSec");
    assertNearlyEqual(1 / scenario.simConfig.simHz, mss.dt, "dt");

    assertNearlyEqual(boat.maxSpeed, mss.boat.maxSpeed, "maxSpeed");
    assertNearlyEqual(boat.maxAcceleration, mss.boat.maxAcceleration, "maxAcceleration");
    assertNearlyEqual(boat.maxDeceleration, mss.boat.maxDeceleration, "maxDeceleration");
    assertNearlyEqual(boat.maxTurn, mss.boat.maxTurn, "maxTurn");
    assertNearlyEqual(boat.mass, mss.boat.mass, "mass");
    assertNearlyEqual(boat.dimensions.x, mss.boat.beam, "beam");
    assertNearlyEqual(boat.dimensions.y, mss.boat.height, "height");
    assertNearlyEqual(boat.dimensions.z, mss.boat.length, "length");
    assertNearlyEqual(boat.startPos.z, mss.boat.startNorth, "startNorth");
    assertNearlyEqual(boat.startPos.x, mss.boat.startEast, "startEast");

    assertNearlyEqual(vehicle.geometry.draft, mss.boat.draft, "draft");
    assertNearlyEqual(vehicle.massProps.inertia.Iz, mss.boat.Iz, "Iz");
    assertNearlyEqual(vehicle.addedMass.XuDot, mss.boat.XuDot, "XuDot");
    assertNearlyEqual(vehicle.addedMass.YvDot, mss.boat.YvDot, "YvDot");
    assertNearlyEqual(vehicle.addedMass.NrDot, mss.boat.NrDot, "NrDot");
    assertNearlyEqual(vehicle.damping.linear.Xu, mss.boat.Xu, "Xu");
    assertNearlyEqual(vehicle.damping.linear.Yv, mss.boat.Yv, "Yv");
    assertNearlyEqual(vehicle.damping.linear.Nr, mss.boat.Nr, "Nr");
    assertNearlyEqual(vehicle.damping.quadratic.Xuu, mss.boat.Xuu, "Xuu");
    assertNearlyEqual(vehicle.damping.quadratic.Yvv, mss.boat.Yvv, "Yvv");
    assertNearlyEqual(vehicle.damping.quadratic.Nrr, mss.boat.Nrr, "Nrr");

    const current = scenario.envConfig.waterFieldConfig.current;
    assertNearlyEqual(current.z, mss.current.north, "currentNorth");
    assertNearlyEqual(current.x, mss.current.east, "currentEast");

    const simWaves = scenario.envConfig.waterFieldConfig.waves;
    assert(simWaves.length === mss.waves.length, "Wave count mismatch.");
    simWaves.forEach((wave, idx) => {
        const mssWave = mss.waves[idx];
        assertNearlyEqual(wave.heading, mssWave.heading, `wave ${idx} heading`);
        assertNearlyEqual(wave.peakHeight, mssWave.peakHeight, `wave ${idx} peakHeight`);
        assertNearlyEqual(wave.wavelength, mssWave.wavelength, `wave ${idx} wavelength`);
        assertNearlyEqual(wave.speed, mssWave.speed, `wave ${idx} speed`);
        assertNearlyEqual(wave.steepness, mssWave.steepness, `wave ${idx} steepness`);
    });

    const simWaypoints = scenario.goalConfig.waypoints.map((wp) => [wp.z, wp.x]);
    assert(simWaypoints.length === mss.waypoints.length, "Waypoint count mismatch.");
    simWaypoints.forEach(([north, east], idx) => {
        assertNearlyEqual(north, mss.waypoints[idx][0], `waypoint ${idx} north`);
        assertNearlyEqual(east, mss.waypoints[idx][1], `waypoint ${idx} east`);
    });
    assertNearlyEqual(scenario.goalConfig.tolerance, mss.tolerance, "goal tolerance");

    return {
        checked: {
            simHz: scenario.simConfig.simHz,
            durationSec: scenario.simConfig.durationSec,
            waypoints: simWaypoints.length,
            waves: simWaves.length,
            mass: boat.mass,
            draft: vehicle.geometry.draft,
            Iz: vehicle.massProps.inertia.Iz
        }
    };
}

try {
    const result = runPresetParityTest();
    console.log("Preset parity test passed.");
    console.log(JSON.stringify(result, null, 2));
} catch (error) {
    console.error("Preset parity test failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
