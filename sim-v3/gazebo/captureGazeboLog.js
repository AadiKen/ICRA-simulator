import fs from "node:fs";
import path from "node:path";
import {spawn, spawnSync} from "node:child_process";
import {fileURLToPath} from "node:url";
import {
    commandSchedule,
    gazeboPoseToBcodSample,
    loadManifest,
    parseGazeboPoseVectorText,
    resampleSamples,
    samplesToCsv
} from "./captureUtils.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));

function parseArgs(argv) {
    const args = {
        manifest: null,
        out: null,
        gzBin: "gz",
        launch: true,
        dryRun: false,
        diagnose: false,
        topic: null,
        timeoutSec: null,
        wrenchLog: null,
        rawOut: null
    };
    for (let i = 2; i < argv.length; i += 1) {
        if (argv[i] === "--manifest") {
            args.manifest = argv[i + 1] || null;
            i += 1;
        }
        else if (argv[i] === "--out") {
            args.out = argv[i + 1] || null;
            i += 1;
        }
        else if (argv[i] === "--gz-bin") {
            args.gzBin = argv[i + 1] || args.gzBin;
            i += 1;
        }
        else if (argv[i] === "--no-launch") {
            args.launch = false;
        }
        else if (argv[i] === "--dry-run") {
            args.dryRun = true;
        }
        else if (argv[i] === "--diagnose") {
            args.diagnose = true;
        }
        else if (argv[i] === "--topic") {
            args.topic = argv[i + 1] || null;
            i += 1;
        }
        else if (argv[i] === "--timeout-sec") {
            args.timeoutSec = Number(argv[i + 1] || 0) || null;
            i += 1;
        }
        else if (argv[i] === "--wrench-log") {
            args.wrenchLog = argv[i + 1] || null;
            i += 1;
        }
        else if (argv[i] === "--raw-out") {
            args.rawOut = argv[i + 1] || null;
            i += 1;
        }
    }
    return args;
}

function assertGazeboAvailable(gzBin) {
    const result = spawnSync(gzBin, ["--version"], {encoding: "utf8"});
    if (result.error) {
        throw new Error(`Could not execute '${gzBin}'. Install Gazebo Harmonic or pass --gz-bin.`);
    }
}

function generatedRootFromManifest(manifest) {
    return path.resolve(manifest.manifestDir, "..");
}

function worldPathFromManifest(manifest) {
    return path.join(generatedRootFromManifest(manifest), manifest.gazebo.world);
}

function worldName(manifest) {
    return `bcod_parity_${manifest.maneuver}`;
}

function gazeboEnv(manifest) {
    const modelsDir = path.join(generatedRootFromManifest(manifest), "models");
    const current = process.env.GZ_SIM_RESOURCE_PATH || "";
    const parts = current ? [modelsDir, current] : [modelsDir];
    return {
        ...process.env,
        GZ_SIM_RESOURCE_PATH: parts.join(path.delimiter),
        GZ_PARTITION: process.env.BCOD_GZ_PARTITION || `bcod_${process.pid}_${Date.now()}`
    };
}

function poseTopic(manifest) {
    return `/world/${worldName(manifest)}/pose/info`;
}

function publishCommand(gzBin, topic, value, env = process.env) {
    return spawnSync(gzBin, [
        "topic",
        "-t",
        topic,
        "-m",
        "gz.msgs.Double",
        "-d",
        "0.25",
        "-p",
        `data: ${Number(value || 0)}`
    ], {encoding: "utf8", env, timeout: 3000});
}

function startCommandPublisher(gzBin, topic, value, durationSec, env = process.env, diagnose = false) {
    const child = spawn(gzBin, [
        "topic",
        "-t",
        topic,
        "-m",
        "gz.msgs.Double",
        "-d",
        Math.max(durationSec, 0.05).toString(),
        "-p",
        `data: ${Number(value || 0)}`
    ], {stdio: ["ignore", "pipe", "pipe"], env});
    if (diagnose) {
        child.stderr.on("data", (chunk) => {
            process.stderr.write(`Publisher ${topic}: ${chunk}`);
        });
    }
    return child;
}

function startMessagePublisher(gzBin, topic, messageType, payload, durationSec, env = process.env, diagnose = false) {
    const child = spawn(gzBin, [
        "topic",
        "-t",
        topic,
        "-m",
        messageType,
        "-d",
        Math.max(durationSec, 0.05).toString(),
        "-p",
        payload
    ], {stdio: ["ignore", "pipe", "pipe"], env});
    if (diagnose) {
        child.stderr.on("data", (chunk) => {
            process.stderr.write(`Publisher ${topic}: ${chunk}`);
        });
    }
    return child;
}

function bodyWrenchToGazeboComponents(command, manifest, yawNed = 0) {
    const wrench = command.value || {};
    const surge = wrench.surge || 0;
    const sway = wrench.sway || 0;
    const yaw = wrench.yaw || 0;
    const cos = Math.cos(yawNed);
    const sin = Math.sin(yawNed);
    const north = surge * cos - sway * sin;
    const east = surge * sin + sway * cos;
    const entity = manifest.gazebo?.wrenchEntity || {name: manifest.vehicle, type: "MODEL"};
    return {
        entity,
        frame: "world_enu_republished_from_body_ned",
        yawNed,
        body: {surge, sway, yaw},
        force: {x: east, y: north, z: 0},
        torque: {x: 0, y: 0, z: -yaw}
    };
}

function bodyWrenchToGazeboEntityWrench(command, manifest, yawNed = 0) {
    const components = bodyWrenchToGazeboComponents(command, manifest, yawNed);
    const {entity, force, torque} = components;
    return [
        `entity: {name: '${entity.name}', type: ${entity.type || "MODEL"}}`,
        `wrench: {force: {x: ${force.x}, y: ${force.y}, z: ${force.z}}, torque: {x: ${torque.x}, y: ${torque.y}, z: ${torque.z}}}`
    ].join(", ");
}

function publishOneShotCommand(gzBin, command, manifest, yawNed = 0, env = process.env) {
    const payload = command.type === "BodyWrench"
        ? bodyWrenchToGazeboEntityWrench(command, manifest, yawNed)
        : `data: ${Number(command.value || 0)}`;
    const messageType = command.messageType || "gz.msgs.Double";
    return spawnSync(gzBin, [
        "topic",
        "-t",
        command.topic,
        "-m",
        messageType,
        "-p",
        payload
    ], {encoding: "utf8", env, timeout: 3000});
}

function isPersistentBodyWrenchCommand(command) {
    return command.type === "BodyWrench" && command.topic?.endsWith("/wrench/persistent");
}

function publishPersistentBodyWrenchCommands(gzBin, commands, manifest, yawNed = 0, env = process.env, appliedWrenches = [], t = 0) {
    commands.forEach((command) => {
        const result = publishOneShotCommand(gzBin, command, manifest, yawNed, env);
        if (result.status !== 0) {
            throw new Error(`Could not publish persistent Gazebo wrench to ${command.topic}:\n${result.stderr || result.stdout}`);
        }
        if (command.type === "BodyWrench") {
            appliedWrenches.push({
                t,
                topic: command.topic,
                ...bodyWrenchToGazeboComponents(command, manifest, yawNed)
            });
        }
    });
}

function startBodyWrenchTransportPublisher(env = process.env, diagnose = false) {
    const script = path.join(__dirname, "transportBodyWrenchPublisher.py");
    const child = spawn("python3", [script], {stdio: ["pipe", "pipe", "pipe"], env});
    const stderr = [];
    child.stderr.on("data", (chunk) => {
        stderr.push(chunk.toString());
        if (diagnose) {
            process.stderr.write(`BodyWrenchPublisher: ${chunk}`);
        }
    });
    if (diagnose) {
        child.stdout.on("data", (chunk) => {
            process.stderr.write(`BodyWrenchPublisher: ${chunk}`);
        });
    }
    return {child, stderr};
}

function publishBodyWrenchViaTransport(publisher, command, manifest, yawNed = 0, appliedWrenches = [], t = 0) {
    const components = bodyWrenchToGazeboComponents(command, manifest, yawNed);
    const topic = manifest.gazebo?.wrenchTopic || command.topic;
    const payload = {
        t,
        topic,
        clearTopic: null,
        entity: components.entity,
        force: components.force,
        torque: components.torque
    };
    publisher.child.stdin.write(`${JSON.stringify(payload)}\n`);
    appliedWrenches.push({
        t,
        topic,
        ...components
    });
}

function publishPersistentBodyWrenchCommandsViaTransport(publisher, commands, manifest, yawNed = 0, appliedWrenches = [], t = 0) {
    commands.forEach((command) => {
        if (command.type !== "BodyWrench") {
            return;
        }
        publishBodyWrenchViaTransport(publisher, command, manifest, yawNed, appliedWrenches, t);
    });
}

function writeAppliedWrenchLog(pathname, appliedWrenches) {
    if (!pathname) {
        return;
    }
    const header = [
        "t",
        "yaw_ned",
        "frame",
        "body_surge",
        "body_sway",
        "body_yaw",
        "force_x_enu",
        "force_y_enu",
        "force_z_enu",
        "torque_x_enu",
        "torque_y_enu",
        "torque_z_enu",
        "topic"
    ];
    const rows = appliedWrenches.map((entry) => [
        entry.t,
        entry.yawNed,
        entry.frame,
        entry.body.surge,
        entry.body.sway,
        entry.body.yaw,
        entry.force.x,
        entry.force.y,
        entry.force.z,
        entry.torque.x,
        entry.torque.y,
        entry.torque.z,
        entry.topic
    ].map((value) => typeof value === "number" ? Number(value || 0).toPrecision(12) : value).join(","));
    fs.mkdirSync(path.dirname(pathname), {recursive: true});
    fs.writeFileSync(pathname, `${header.join(",")}\n${rows.join("\n")}\n`);
}

function clearPersistentWrench(gzBin, manifest, env = process.env) {
    if (manifest.gazebo?.actuationMode !== "bodyWrench" || !manifest.gazebo?.wrenchClearTopic) {
        return;
    }
    const entity = manifest.gazebo?.wrenchEntity || {name: manifest.vehicle, type: "MODEL"};
    spawnSync(gzBin, [
        "topic",
        "-t",
        manifest.gazebo.wrenchClearTopic,
        "-m",
        "gz.msgs.Entity",
        "-p",
        `name: '${entity.name}', type: ${entity.type || "MODEL"}`
    ], {encoding: "utf8", env, timeout: 3000});
}

function commandsEqual(a = [], b = []) {
    if (a.length !== b.length) {
        return false;
    }
    return a.every((command, idx) => {
        const other = b[idx];
        if (command.topic !== other.topic || command.type !== other.type) {
            return false;
        }
        if (command.type === "BodyWrench") {
            const value = command.value || {};
            const otherValue = other.value || {};
            return Math.abs((value.surge || 0) - (otherValue.surge || 0)) < 1e-9 &&
                Math.abs((value.sway || 0) - (otherValue.sway || 0)) < 1e-9 &&
                Math.abs((value.yaw || 0) - (otherValue.yaw || 0)) < 1e-9;
        }
        return Math.abs((command.value || 0) - (other.value || 0)) < 1e-9;
    });
}

function commandTransitions(schedule) {
    const transitions = [];
    let previous = null;
    schedule.forEach((row) => {
        if (!previous || !commandsEqual(row.commands, previous.commands)) {
            transitions.push(row);
        }
        previous = row;
    });
    return transitions;
}

function usesPersistentBodyWrench(transitions) {
    return transitions.length > 0 &&
        transitions.every((row) => row.commands.length > 0 && row.commands.every(isPersistentBodyWrenchCommand));
}

function listTopics(gzBin, env = process.env) {
    const result = spawnSync(gzBin, ["topic", "-l"], {encoding: "utf8", env});
    if (result.status !== 0) {
        throw new Error(`Could not list Gazebo topics:\n${result.stderr || result.stdout}`);
    }
    return result.stdout.split(/\r?\n/).map((line) => line.trim()).filter(Boolean);
}

function topicInfo(gzBin, topic, env = process.env) {
    const result = spawnSync(gzBin, ["topic", "-i", "-t", topic], {encoding: "utf8", env});
    return {
        status: result.status,
        stdout: result.stdout,
        stderr: result.stderr
    };
}

function publishConfiguredCurrent(gzBin, manifest, env = process.env) {
    const current = manifest.gazebo?.currentEnu;
    if (!current || !Object.values(current).some((value) => Math.abs(Number(value) || 0) > 0)) {
        return null;
    }
    const topic = manifest.gazebo?.currentTopic || "/ocean_current";
    const result = spawnSync(gzBin, [
        "topic", "-t", topic, "-m", "gz.msgs.Vector3d",
        "-d", "0.5",
        "-p", `x: ${Number(current.x || 0)}, y: ${Number(current.y || 0)}, z: ${Number(current.z || 0)}`
    ], {encoding: "utf8", env, timeout: 5000});
    if (result.status !== 0) {
        throw new Error(`Could not publish configured Gazebo current to ${topic}:\n${result.stderr || result.stdout}`);
    }
    return {topic, ...current};
}

async function sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
}

async function waitFor(predicate, timeoutMs, label) {
    const start = Date.now();
    while (!predicate()) {
        if (Date.now() - start > timeoutMs) {
            throw new Error(`Timed out waiting for ${label}.`);
        }
        await sleep(25);
    }
}

async function stopChild(child, signal = "SIGTERM", timeoutMs = 1500) {
    if (!child || child.exitCode !== null) {
        return;
    }
    child.kill(signal);
    await new Promise((resolve) => {
        const timer = setTimeout(() => {
            if (child.exitCode === null) {
                child.kill("SIGKILL");
            }
            resolve();
        }, timeoutMs);
        child.once("exit", () => {
            clearTimeout(timer);
            resolve();
        });
    });
}

async function capture(args) {
    if (!args.manifest) {
        throw new Error("Missing --manifest path.");
    }
    const manifest = loadManifest(args.manifest);
    const out = args.out || path.join(generatedRootFromManifest(manifest), manifest.expectedGoldenCsv);
    const world = worldPathFromManifest(manifest);
    const topic = args.topic || poseTopic(manifest);
    const schedule = commandSchedule(manifest);
    const env = gazeboEnv(manifest);

    if (args.dryRun) {
        return {
            manifest: args.manifest,
            world,
            topic,
            out,
            durationSec: manifest.dt * manifest.steps,
            steps: manifest.steps,
            env: {
                GZ_SIM_RESOURCE_PATH: env.GZ_SIM_RESOURCE_PATH,
                GZ_PARTITION: env.GZ_PARTITION
            },
            firstCommands: schedule[0]?.commands || []
        };
    }

    assertGazeboAvailable(args.gzBin);
    if (!fs.existsSync(world)) {
        throw new Error(`World file does not exist: ${world}`);
    }

    let gzSim = null;
    const simStderr = [];
    if (args.launch) {
        const maximumIterations = Math.ceil((manifest.dt * manifest.steps + 10) / manifest.dt);
        gzSim = spawn(args.gzBin, ["sim", "-r", "-s", "--iterations", String(maximumIterations), world], {
            stdio: ["ignore", "pipe", "pipe"],
            env
        });
        gzSim.stderr.on("data", (chunk) => {
            simStderr.push(chunk.toString());
            if (args.diagnose) {
                process.stderr.write(chunk);
            }
        });
        await sleep(2500);
    }

    const topics = listTopics(args.gzBin, env);
    const firstCommands = schedule[0]?.commands || [];
    const missingCommandTopics = firstCommands
        .map((command) => command.topic)
        .filter((commandTopic) => !topics.includes(commandTopic));
    if (args.diagnose) {
        console.log(JSON.stringify({
            world,
            poseTopic: topic,
            expectedCommandTopics: firstCommands.map((command) => command.topic),
            advertisedCommandTopics: topics.filter((candidate) => candidate.includes("cmd") || candidate.includes("thrust")),
            poseTopicInfo: topicInfo(args.gzBin, topic, env),
            commandTopicInfo: firstCommands.map((command) => ({
                topic: command.topic,
                info: topicInfo(args.gzBin, command.topic, env)
            }))
        }, null, 2));
    }
    if (missingCommandTopics.length) {
        if (gzSim) {
            gzSim.kill("SIGTERM");
        }
        throw new Error(`Gazebo is not advertising command topics: ${missingCommandTopics.join(", ")}\nAdvertised cmd/thrust topics: ${topics.filter((candidate) => candidate.includes("cmd") || candidate.includes("thrust")).join(", ") || "(none)"}\nGazebo stderr:\n${simStderr.join("").slice(-4000)}`);
    }
    const configuredCurrent = publishConfiguredCurrent(args.gzBin, manifest, env);
    if (configuredCurrent) {
        await sleep(100);
    }

    const transitions = commandTransitions(schedule);
    const persistentBodyWrench = usesPersistentBodyWrench(transitions);
    const durationMs = manifest.dt * manifest.steps * 1000;
    const appliedWrenches = [];

    const rawSamples = [];
    const rawGazeboPoses = [];
    let poseStreamReady = false;
    let timeOriginWallTime = null;
    let bodyWrenchPublisher = null;
    const echo = spawn(args.gzBin, ["topic", "-e", "-t", topic], {
        stdio: ["ignore", "pipe", "pipe"],
        env
    });
    echo.stdout.on("data", (chunk) => {
        const now = Date.now();
        parseGazeboPoseVectorText(chunk.toString(), manifest.vehicle).forEach((pose) => {
            poseStreamReady = true;
            if (timeOriginWallTime === null || now < timeOriginWallTime) {
                return;
            }
            const t = (now - timeOriginWallTime) / 1000;
            rawGazeboPoses.push({t, pose});
            rawSamples.push(gazeboPoseToBcodSample(pose, t));
        });
    });

    if (persistentBodyWrench) {
        bodyWrenchPublisher = startBodyWrenchTransportPublisher(env, args.diagnose);
        await waitFor(() => poseStreamReady, 3000, "initial Gazebo pose sample before command publication");
        const warmCommand = {
            ...firstCommands[0],
            value: {surge: 0, sway: 0, yaw: 0}
        };
        publishBodyWrenchViaTransport(bodyWrenchPublisher, warmCommand, manifest, 0, appliedWrenches, -1);
        appliedWrenches.length = 0;
        await sleep(150);
        timeOriginWallTime = Date.now();
        const startPublish = timeOriginWallTime;
        for (let idx = 0; idx < schedule.length; idx += 1) {
            const row = schedule[idx];
            const targetElapsed = row.t * 1000;
            const remaining = targetElapsed - (Date.now() - startPublish);
            await sleep(Math.max(remaining, 0));
            const lastPoseSample = rawSamples[rawSamples.length - 1];
            const yawNed = lastPoseSample?.yaw || 0;
            publishPersistentBodyWrenchCommandsViaTransport(bodyWrenchPublisher, row.commands, manifest, yawNed, appliedWrenches, row.t);
        }
    }
    else {
        timeOriginWallTime = Date.now();
        const startPublish = Date.now();
        for (let idx = 0; idx < transitions.length; idx += 1) {
            const row = transitions[idx];
            const nextT = transitions[idx + 1]?.t ?? manifest.dt * manifest.steps;
            const segmentSec = Math.max(nextT - row.t, manifest.dt);
            const lastPoseSample = rawSamples[rawSamples.length - 1];
            const yawNed = lastPoseSample?.yaw || 0;
            const publishers = row.commands.map((command) => {
                if (command.type === "BodyWrench") {
                    return startMessagePublisher(
                        args.gzBin,
                        command.topic,
                        command.messageType || "gz.msgs.EntityWrench",
                        bodyWrenchToGazeboEntityWrench(command, manifest, yawNed),
                        segmentSec,
                        env,
                        args.diagnose
                    );
                }
                    return startCommandPublisher(args.gzBin, command.topic, command.value, segmentSec, env, args.diagnose);
            });
            const targetElapsed = nextT * 1000;
            const remaining = targetElapsed - (Date.now() - startPublish);
            await sleep(Math.max(remaining, 0));
            await Promise.all(publishers.map((publisher) => stopChild(publisher, "SIGTERM", 500)));
        }
    }
    const recordingStart = timeOriginWallTime || Date.now();
    const remaining = durationMs - (Date.now() - recordingStart);
    if (remaining > 0) {
        await sleep(remaining);
    }

    await sleep(500);
    clearPersistentWrench(args.gzBin, manifest, env);
    if (bodyWrenchPublisher) {
        bodyWrenchPublisher.child.stdin.end();
        await stopChild(bodyWrenchPublisher.child, "SIGTERM", 500);
        if (bodyWrenchPublisher.child.exitCode && bodyWrenchPublisher.child.exitCode !== 0) {
            throw new Error(`Body wrench transport publisher failed:\n${bodyWrenchPublisher.stderr.join("").slice(-4000)}`);
        }
    }
    await stopChild(echo);
    await stopChild(gzSim);

    if (args.rawOut) {
        fs.mkdirSync(path.dirname(args.rawOut), {recursive: true});
        fs.writeFileSync(args.rawOut, `${JSON.stringify(rawGazeboPoses, null, 2)}\n`);
    }

    const samples = resampleSamples(rawSamples, manifest.dt, manifest.steps);
    if (samples.length !== manifest.steps) {
        throw new Error(`Captured ${samples.length} samples, expected ${manifest.steps}.`);
    }
    validateCapturedSamples(manifest, samples, rawSamples.length);
    fs.mkdirSync(path.dirname(out), {recursive: true});
    fs.writeFileSync(out, samplesToCsv(samples));
    writeAppliedWrenchLog(args.wrenchLog, appliedWrenches);
    return {
        manifest: args.manifest,
        world,
        topic,
        out,
        rawSamples: rawSamples.length,
        samples: samples.length,
        commandTransitions: transitions.length,
        appliedWrenchSamples: appliedWrenches.length,
        wrenchLog: args.wrenchLog || null,
        commandTopics: firstCommands.map((command) => command.topic),
        configuredCurrent
    };
}

function validateCapturedSamples(manifest, samples, rawCount) {
    if (rawCount === 0) {
        throw new Error("No Gazebo pose samples were captured. Check the pose topic and PosePublisher plugin.");
    }
    const first = samples[0];
    const last = samples[samples.length - 1];
    const distance = Math.hypot(last.N - first.N, last.E - first.E);
    const yawChange = Math.abs(last.yaw - first.yaw);
    const command = manifest.command || {};
    const shouldTranslate = Math.abs(command.surgeForce || 0) > 0;
    const shouldGoStraight = command.type === "constant" &&
        Math.abs(command.surgeForce || 0) > 0 &&
        Math.abs(command.differentialForce || 0) < 1e-9 &&
        Math.abs(command.yawMoment || 0) < 1e-9;

    if (shouldTranslate && distance < 0.05) {
        throw new Error(`Captured '${manifest.maneuver}' has only ${distance.toFixed(4)} m translation. This usually means thruster commands are not reaching Gazebo or the wrong pose entity is being captured.`);
    }
    if (shouldTranslate && (last.N - first.N) <= 0.05) {
        throw new Error(`Captured '${manifest.maneuver}' moved ${(last.N - first.N).toFixed(4)} m in N under positive surge. This indicates reversed Gazebo actuation or an unstable added-mass backend.`);
    }
    if (shouldTranslate) {
        const firstMotionIndex = samples.findIndex((sample) => Math.hypot(sample.u || 0, sample.v || 0) > 0.05);
        if (firstMotionIndex < 0 || firstMotionIndex > 2) {
            const firstMotionTime = firstMotionIndex < 0 ? "never" : samples[firstMotionIndex].t.toFixed(4);
            throw new Error(`Captured '${manifest.maneuver}' first motion occurred at t=${firstMotionTime}; expected force onset within 2 samples of trajectory origin.`);
        }
    }
    if (shouldGoStraight && yawChange > 0.2) {
        throw new Error(`Captured straight-thrust maneuver yaw changed by ${yawChange.toFixed(4)} rad. This indicates asymmetric Gazebo actuation or the wrong pose entity was captured.`);
    }
}

if (process.argv[1] && process.argv[1].endsWith("captureGazeboLog.js")) {
    capture(parseArgs(process.argv))
        .then((result) => {
            console.log(JSON.stringify(result, null, 2));
        })
        .catch((error) => {
            console.error(error.stack || error.message);
            process.exitCode = 1;
        });
}

export {
    capture,
    parseArgs,
    bodyWrenchToGazeboComponents,
    bodyWrenchToGazeboEntityWrench,
    generatedRootFromManifest,
    gazeboEnv,
    commandTransitions,
    listTopics,
    publishConfiguredCurrent,
    poseTopic,
    stopChild,
    topicInfo,
    validateCapturedSamples,
    worldPathFromManifest
};
