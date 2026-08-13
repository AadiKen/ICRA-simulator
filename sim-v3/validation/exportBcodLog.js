import fs from "node:fs";
import path from "node:path";
import {replayManeuver} from "./goldenLogCompare.js";

const columns = ["t", "N", "E", "yaw", "u", "v", "r"];

function parseArgs(argv) {
    const args = {
        maneuver: "constant-thrust",
        out: null
    };
    for (let i = 2; i < argv.length; i += 1) {
        if (argv[i] === "--maneuver") {
            args.maneuver = argv[i + 1] || args.maneuver;
            i += 1;
        }
        else if (argv[i] === "--out") {
            args.out = argv[i + 1] || null;
            i += 1;
        }
    }
    return args;
}

export function samplesToCsv(samples) {
    const lines = [columns.join(",")];
    samples.forEach((sample) => {
        lines.push(columns.map((column) => Number(sample[column] || 0).toPrecision(12)).join(","));
    });
    return `${lines.join("\n")}\n`;
}

if (process.argv[1] && process.argv[1].endsWith("exportBcodLog.js")) {
    const args = parseArgs(process.argv);
    const csv = samplesToCsv(replayManeuver(args.maneuver));
    if (args.out) {
        fs.mkdirSync(path.dirname(args.out), {recursive: true});
        fs.writeFileSync(args.out, csv);
        console.log(JSON.stringify({maneuver: args.maneuver, out: args.out}, null, 2));
    }
    else {
        process.stdout.write(csv);
    }
}
