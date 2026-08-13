#!/usr/bin/env node

import fs from "node:fs";
import path from "node:path";
import {fileURLToPath} from "node:url";

import {LegacyProductionEngine} from "../../sim-v3/backends/node/src/legacy-production-engine.ts";
import {HeadlessMarineSimulation} from "../../sim-v3/packages/core/src/simulation.ts";
import {resolveExperiment} from "../../sim-v3/packages/experiment-schema/src/index.ts";

const REQUIRED_COLUMNS = ["time_s", "propeller_rps", "rudder_rad"];
const OUTPUT_COLUMNS = ["t", "N", "E", "yaw", "u", "v", "r"];
const DEFAULT_TIMESTEP_S = 0.02;

function parseArgs(argv) {
    const args = {commands: null, out: null, timestep: null};
    for (let index = 2; index < argv.length; index += 1) {
        const value = argv[index];
        if (value === "--commands") args.commands = argv[++index];
        else if (value === "--out") args.out = argv[++index];
        else if (value === "--timestep") args.timestep = Number(argv[++index]);
        else throw new Error(`Unknown argument: ${value}`);
    }
    if (!args.commands || !args.out) {
        throw new Error(
            "Usage: node --experimental-strip-types exportCoupled6Log.js " +
            "--commands commands.csv --out trajectory.csv [--timestep 0.02]"
        );
    }
    if (args.timestep !== null && (!Number.isFinite(args.timestep) || args.timestep <= 0)) {
        throw new Error("--timestep must be a finite positive number");
    }
    return args;
}

function loadCommands(filePath) {
    const lines = fs.readFileSync(filePath, "utf8")
        .split(/\r?\n/)
        .map((line) => line.trim())
        .filter((line) => line && !line.startsWith("#"));
    if (lines.length < 2) throw new Error(`${filePath} contains no command rows`);

    const header = lines.shift().split(",").map((column) => column.trim());
    const indices = Object.fromEntries(
        REQUIRED_COLUMNS.map((column) => [column, header.indexOf(column)])
    );
    const missing = REQUIRED_COLUMNS.filter((column) => indices[column] < 0);
    if (missing.length) {
        throw new Error(`${filePath} is missing required columns: ${missing.join(", ")}`);
    }

    const commands = lines.map((line, rowIndex) => {
        const values = line.split(",");
        const command = Object.fromEntries(REQUIRED_COLUMNS.map((column) => [
            column, Number(values[indices[column]])
        ]));
        if (!REQUIRED_COLUMNS.every((column) => Number.isFinite(command[column]))) {
            throw new Error(`${filePath} row ${rowIndex + 2} contains a non-finite value`);
        }
        if (command.time_s < 0) {
            throw new Error(`${filePath} row ${rowIndex + 2} has a negative time_s`);
        }
        return command;
    });
    if (commands.some((command, index) => index > 0 && command.time_s <= commands[index - 1].time_s)) {
        throw new Error(`${filePath} time_s values must be strictly increasing`);
    }
    return commands;
}

function inferTimestep(commands) {
    if (commands.length < 2) return DEFAULT_TIMESTEP_S;
    return commands[1].time_s - commands[0].time_s;
}

function validateCommandCadence(commands, timestep) {
    const tolerance = Math.max(1e-9, timestep * 1e-6);
    for (let index = 1; index < commands.length; index += 1) {
        const interval = commands[index].time_s - commands[index - 1].time_s;
        if (Math.abs(interval - timestep) > tolerance) {
            throw new Error(
                `Command rows must be spaced one simulation step apart; ` +
                `row ${index + 2} interval is ${interval}, expected ${timestep}`
            );
        }
    }
}

function projectTruth(truth) {
    return [
        truth.time_s,
        truth.position_ned_m[0],
        truth.position_ned_m[1],
        truth.attitude_rad[2],
        truth.velocity_body_mps[0],
        truth.velocity_body_mps[1],
        truth.angular_rate_body_rad_s[2],
    ];
}

export function runCoupled6Replay(commands, timestep) {
    validateCommandCadence(commands, timestep);
    const duration = commands.length * timestep;
    const config = resolveExperiment({
        schema_version: 1,
        experiment: {
            name: "unity-validation-vehicle-b-coupled6",
            seed: 2027,
            timestep_s: timestep,
            duration_s: duration,
        },
        backend: {type: "node"},
        vehicle: {preset: "vehicle-b-rudder", plant: "coupled6"},
        mission: {type: "unity-validation-command-replay"},
        outputs: {state_hz: 1 / timestep},
    });
    const simulation = new HeadlessMarineSimulation(new LegacyProductionEngine());
    const rows = [];

    simulation.reset(config);
    try {
        for (const command of commands) {
            const result = simulation.step({
                actuators: {
                    propeller_rps: command.propeller_rps,
                    rudder_rad: command.rudder_rad,
                },
            });
            if (
                result.info.vehicle_path?.vehicle_id !== "vehicle-b-rudder" ||
                result.info.vehicle_path?.plant !== "coupled6" ||
                result.info.vehicle_path?.maneuvering_model !== "mmg"
            ) {
                throw new Error("Production Vehicle B coupled6/MMG path was not selected");
            }
            rows.push(projectTruth(simulation.getGroundTruth()));
        }
    }
    finally {
        simulation.dispose();
    }
    return rows;
}

function writeTrajectory(filePath, rows) {
    const lines = [OUTPUT_COLUMNS.join(",")];
    for (const row of rows) {
        lines.push(row.map((value) => Number(value).toPrecision(12)).join(","));
    }
    fs.mkdirSync(path.dirname(path.resolve(filePath)), {recursive: true});
    fs.writeFileSync(filePath, `${lines.join("\n")}\n`);
}

function main() {
    const args = parseArgs(process.argv);
    const commands = loadCommands(args.commands);
    const timestep = args.timestep ?? inferTimestep(commands);
    const rows = runCoupled6Replay(commands, timestep);
    if (rows.some((row) => row.some((value) => !Number.isFinite(value)))) {
        throw new Error("Simulation produced a non-finite trajectory value");
    }
    writeTrajectory(args.out, rows);
    process.stderr.write(JSON.stringify({
        vehicle: "vehicle-b-rudder",
        plant: "coupled6",
        timestep_s: timestep,
        command_rows: commands.length,
        trajectory_rows: rows.length,
        duration_s: rows.at(-1)[0],
        out: path.resolve(args.out),
    }, null, 2) + "\n");
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
    try {
        main();
    }
    catch (error) {
        process.stderr.write(`${error instanceof Error ? error.message : error}\n`);
        process.exitCode = 1;
    }
}
