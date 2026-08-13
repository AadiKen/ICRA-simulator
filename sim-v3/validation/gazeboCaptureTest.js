import {
    bodyWrenchCommandForTime,
    commandAt,
    commandSchedule,
    deriveBodyVelocities,
    gazeboPoseToBcodSample,
    parseGazeboPoseVectorText,
    resampleSamples
} from "../gazebo/captureUtils.js";
import {bodyWrenchToGazeboComponents, capture, commandTransitions, parseArgs, poseTopic, worldPathFromManifest} from "../gazebo/captureGazeboLog.js";
import {validateCapturedSamples} from "../gazebo/captureGazeboLog.js";
import {loadManifest} from "../gazebo/captureUtils.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function approx(value, expected, tolerance, message) {
    assert(Math.abs(value - expected) <= tolerance, `${message}: expected ${expected}, got ${value}`);
}

function manifestPath(name = "constant-thrust") {
    return `gazebo/generated/manifests/otter_${name}.json`;
}

function testCommandScheduleMatchesManifest() {
    const manifest = loadManifest(manifestPath("zig-zag"));
    const first = commandAt(manifest.command, 0);
    const flipped = commandAt(manifest.command, manifest.command.periodSec);
    assert(first.differentialForce === 28, "Zig-zag should start with positive differential command.");
    assert(flipped.differentialForce === -28, "Zig-zag should flip differential command at the configured period.");
    const schedule = commandSchedule(manifest);
    assert(schedule.length === manifest.steps, "Command schedule should have one row per manifest step.");
    assert(schedule[0].commands.length === manifest.gazebo.commandTopics.length, "Schedule should command every manifest command topic.");
    assert(schedule[0].commands[0].type === "BodyWrench", "Generated Gazebo captures should use net body-wrench commands.");
    const firstWrench = bodyWrenchCommandForTime(manifest, undefined, 0);
    approx(firstWrench.value.surge, 60, 1e-12, "Net wrench command should preserve commanded surge force.");
    approx(firstWrench.value.yaw, 15.12, 1e-12, "Net wrench command should preserve the configured thruster-geometry yaw moment.");
    const constantTransitions = commandTransitions(commandSchedule(loadManifest(manifestPath("constant-thrust"))));
    assert(constantTransitions.length === 1, "Constant BodyWrench capture should publish as one continuous transition.");
    const yawTurn = bodyWrenchCommandForTime(loadManifest(manifestPath("yaw-turn")), undefined, 0);
    approx(yawTurn.value.surge, 55, 1e-12, "Yaw-turn should preserve commanded surge force.");
    approx(yawTurn.value.yaw, 1, 1e-12, "Yaw-turn should preserve commanded direct yaw moment.");
    const zigZagTransitions = commandTransitions(schedule);
    assert(zigZagTransitions.length > 1, "Zig-zag BodyWrench capture should retain command flips.");
}

function testBodyWrenchRepublishesInWorldFrameFromCurrentYaw() {
    const manifest = loadManifest(manifestPath("yaw-turn"));
    const command = bodyWrenchCommandForTime(manifest, undefined, 0);
    const north = bodyWrenchToGazeboComponents(command, manifest, 0);
    approx(north.force.x, 0, 1e-12, "At yaw 0, body surge should map to zero Gazebo east force.");
    approx(north.force.y, 55, 1e-12, "At yaw 0, body surge should map to positive Gazebo north force.");
    approx(north.torque.z, -1, 1e-12, "Positive BCOD yaw moment should publish as negative ENU z torque.");
    assert(north.frame === "world_enu_republished_from_body_ned", "Wrench log should identify the world-frame republish mapping.");

    const east = bodyWrenchToGazeboComponents(command, manifest, Math.PI / 2);
    approx(east.force.x, 55, 1e-12, "At yaw pi/2, body surge should rotate into positive Gazebo east force.");
    approx(east.force.y, 0, 1e-12, "At yaw pi/2, body surge should no longer remain fixed north.");
    approx(east.torque.z, -1, 1e-12, "Yaw torque should remain delivered while force rotates.");
}

function testGazeboPoseConvertsToBcodFrame() {
    const sample = gazeboPoseToBcodSample({
        position: {x: 2, y: 3, z: -1},
        orientation: {w: Math.SQRT1_2, x: 0, y: 0, z: Math.SQRT1_2}
    }, 0.5);
    approx(sample.N, 3, 1e-12, "Gazebo ENU y should become BCOD North.");
    approx(sample.E, 2, 1e-12, "Gazebo ENU x should become BCOD East.");
    approx(sample.yaw, 0, 1e-12, "ENU yaw pi/2 should become NED yaw 0.");
}

function testVelocityDerivationAndResampling() {
    const samples = deriveBodyVelocities([
        {t: 0, N: 0, E: 0, yaw: 0},
        {t: 1, N: 2, E: 0, yaw: 0}
    ]);
    approx(samples[1].u, 2, 1e-12, "Northward world motion at yaw 0 should be positive surge.");
    const rotating = deriveBodyVelocities([
        {t: 0, N: 0, E: 0, yaw: 0},
        {t: 1, N: 0, E: 0, yaw: 0.2}
    ]);
    approx(rotating[1].r, 0.2, 1e-12, "Yaw rate should be finite-differenced.");
    const resampled = resampleSamples([
        {t: 0, N: 0, E: 0, yaw: Math.PI / 2},
        {t: 2, N: 0, E: 4, yaw: Math.PI / 2}
    ], 1, 3);
    approx(resampled[1].N, 2, 1e-12, "Resampler should interpolate position.");
    approx(resampled[1].yaw, 0, 1e-12, "Resampler should normalize initial yaw to the BCOD comparison frame.");
    approx(resampled[2].u, 2, 1e-12, "Resampler should derive body velocity after interpolation.");
}

function testPoseVectorTextParser() {
    const text = `
pose {
  name: "otter::base_link"
  position { x: 9 y: 9 z: 9 }
  orientation { w: 1 x: 0 y: 0 z: 0 }
}
pose {
  name: "otter"
  position { x: 1 y: 2 z: 3 }
  orientation { w: 1 x: 0 y: 0 z: 0 }
}
pose {
  name: "other"
  position { x: 9 y: 9 z: 9 }
}
`;
    const poses = parseGazeboPoseVectorText(text, "otter");
    assert(poses.length === 1, "Parser should select the requested model pose.");
    approx(poses[0].position.x, 1, 1e-12, "Parser should extract pose position.");
    assert(poses[0].name === "otter", "Parser should prefer exact model pose over link pose.");
    assert(parseGazeboPoseVectorText('pose { name: "otter::base_link" position { x: 1 y: 2 z: 3 } }', "otter").length === 0, "Parser should reject link-only pose messages.");
}

async function testCaptureDryRun() {
    const args = parseArgs(["node", "capture", "--manifest", manifestPath("constant-thrust"), "--dry-run", "--diagnose", "--raw-out", "/tmp/raw.json"]);
    assert(args.diagnose === true, "Parser should accept --diagnose.");
    assert(args.rawOut === "/tmp/raw.json", "Parser should accept a raw Gazebo pose diagnostic path.");
    const result = await capture(args);
    assert(result.world.endsWith("gazebo/generated/worlds/otter_constant-thrust.sdf"), "Dry run should resolve generated world path.");
    assert(result.topic === "/world/bcod_parity_constant-thrust/pose/info", "Dry run should resolve pose topic.");
    assert(result.env.GZ_SIM_RESOURCE_PATH.includes("gazebo/generated/models"), "Dry run should expose generated model path for Gazebo.");
    assert(result.env.GZ_PARTITION.startsWith("bcod_"), "Each Gazebo run should receive an isolated transport partition.");
    assert(result.firstCommands.length === 1, "Dry run should show the net body-wrench command.");
    assert(result.firstCommands[0].topic === "/world/bcod_parity_constant-thrust/wrench/persistent", "Dry run should target the ApplyLinkWrench persistent wrench topic.");
    const manifest = loadManifest(manifestPath("constant-thrust"));
    assert(worldPathFromManifest(manifest).endsWith(result.world.split("/").slice(-3).join("/")), "World path helper should agree with dry-run.");
    assert(poseTopic(manifest) === result.topic, "Pose topic helper should agree with dry-run.");
}

function testCaptureRejectsZeroMotionStraightThrust() {
    const manifest = loadManifest(manifestPath("constant-thrust"));
    const badSamples = Array.from({length: manifest.steps}, (_, idx) => ({
        t: idx * manifest.dt,
        N: 0,
        E: 0,
        yaw: idx * 0.01,
        u: 0,
        v: 0,
        r: 0.2
    }));
    let rejected = false;
    try {
        validateCapturedSamples(manifest, badSamples, badSamples.length);
    }
    catch {
        rejected = true;
    }
    assert(rejected, "Capture should reject straight-thrust logs with no translation and large yaw drift.");
    const backwardSamples = badSamples.map((sample, idx) => ({
        ...sample,
        N: -idx * 0.1,
        yaw: 0,
        r: 0
    }));
    rejected = false;
    try {
        validateCapturedSamples(manifest, backwardSamples, backwardSamples.length);
    }
    catch {
        rejected = true;
    }
    assert(rejected, "Capture should reject backward motion under positive surge.");
}

function testCaptureRejectsDelayedForceOnset() {
    const manifest = loadManifest(manifestPath("constant-thrust"));
    const delayedSamples = Array.from({length: manifest.steps}, (_, idx) => {
        const moving = idx >= 20;
        return {
            t: idx * manifest.dt,
            N: moving ? (idx - 19) * 0.1 : 0,
            E: 0,
            yaw: 0,
            u: moving ? 2 : 0,
            v: 0,
            r: 0
        };
    });
    let rejected = false;
    try {
        validateCapturedSamples(manifest, delayedSamples, delayedSamples.length);
    }
    catch {
        rejected = true;
    }
    assert(rejected, "Capture should reject delayed first motion under positive surge.");

    const promptSamples = delayedSamples.map((sample, idx) => ({
        ...sample,
        N: idx * 0.1,
        u: idx === 0 ? 0 : 2
    }));
    validateCapturedSamples(manifest, promptSamples, promptSamples.length);
}

const tests = [
    testCommandScheduleMatchesManifest,
    testBodyWrenchRepublishesInWorldFrameFromCurrentYaw,
    testGazeboPoseConvertsToBcodFrame,
    testVelocityDerivationAndResampling,
    testPoseVectorTextParser,
    testCaptureDryRun,
    testCaptureRejectsZeroMotionStraightThrust,
    testCaptureRejectsDelayedForceOnset
];

try {
    const results = [];
    for (const test of tests) {
        await test();
        results.push(test.name);
    }
    console.log("Gazebo capture tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Gazebo capture tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
