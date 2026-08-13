import fs from "node:fs";
import path from "node:path";
import {replayManeuver} from "./goldenLogCompare.js";

const maneuvers = [
    "constant-thrust",
    "coast-down",
    "turning-circle",
    "zig-zag",
    "current-drift"
];

const outputDir = path.resolve(process.argv[2] || "validation/mss-replay");
fs.mkdirSync(outputDir, {recursive: true});

for (const maneuver of maneuvers) {
    const samples = replayManeuver(maneuver);
    const lines = [
        "t,N,E,yaw,u,v,r",
        ...samples.map((sample) => [
            sample.t,
            sample.N,
            sample.E,
            sample.yaw,
            sample.u,
            sample.v,
            sample.r
        ].map((value) => Number(value).toPrecision(16)).join(","))
    ];
    fs.writeFileSync(path.join(outputDir, `${maneuver}.csv`), `${lines.join("\n")}\n`);
}

console.log(`Wrote ${maneuvers.length} simulator traces to ${outputDir}`);
