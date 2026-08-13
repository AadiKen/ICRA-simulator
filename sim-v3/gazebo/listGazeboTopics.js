import {spawn, spawnSync} from "node:child_process";
import {loadManifest} from "./captureUtils.js";
import {gazeboEnv, stopChild, worldPathFromManifest} from "./captureGazeboLog.js";

function parseArgs(argv) {
    const args = {
        manifest: null,
        gzBin: "gz",
        launch: true
    };
    for (let i = 2; i < argv.length; i += 1) {
        if (argv[i] === "--manifest") {
            args.manifest = argv[i + 1] || null;
            i += 1;
        }
        else if (argv[i] === "--gz-bin") {
            args.gzBin = argv[i + 1] || args.gzBin;
            i += 1;
        }
        else if (argv[i] === "--no-launch") {
            args.launch = false;
        }
    }
    return args;
}

function sleep(ms) {
    return new Promise((resolve) => setTimeout(resolve, ms));
}

async function main(args) {
    if (!args.manifest) {
        throw new Error("Missing --manifest path.");
    }
    const manifest = loadManifest(args.manifest);
    const env = gazeboEnv(manifest);
    let server = null;
    if (args.launch) {
        server = spawn(args.gzBin, ["sim", "-r", "-s", worldPathFromManifest(manifest)], {
            stdio: ["ignore", "pipe", "pipe"],
            env
        });
        await sleep(2500);
    }
    const result = spawnSync(args.gzBin, ["topic", "-l"], {encoding: "utf8", env});
    if (server) {
        await stopChild(server);
    }
    const topics = result.stdout.split(/\r?\n/).filter(Boolean).sort();
    const expected = manifest.gazebo?.commandTopics || [];
    console.log(JSON.stringify({
        world: worldPathFromManifest(manifest),
        expectedCommandTopics: expected,
        missingExpectedCommandTopics: expected.filter((topic) => !topics.includes(topic)),
        poseTopic: `/world/bcod_parity_${manifest.maneuver}/pose/info`,
        topics
    }, null, 2));
}

if (process.argv[1] && process.argv[1].endsWith("listGazeboTopics.js")) {
    main(parseArgs(process.argv)).catch((error) => {
        console.error(error.stack || error.message);
        process.exitCode = 1;
    });
}
