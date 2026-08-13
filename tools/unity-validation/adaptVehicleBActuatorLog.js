#!/usr/bin/env node

import fs from "node:fs";
import path from "node:path";
import {fileURLToPath} from "node:url";

const OUTPUT_COLUMNS = ["time_s", "propeller_rps", "rudder_rad"];

function parseArgs(argv) {
    const args = {input: null, out: null};
    for (let index = 2; index < argv.length; index += 1) {
        if (argv[index] === "--input") args.input = argv[++index];
        else if (argv[index] === "--out") args.out = argv[++index];
        else throw new Error(`Unknown argument: ${argv[index]}`);
    }
    if (!args.input || !args.out) {
        throw new Error(
            "Usage: node adaptVehicleBActuatorLog.js " +
            "--input actuator.jsonl --out commands.csv"
        );
    }
    return args;
}

function parseDiagnostics(record, rowNumber) {
    let diagnostics = record.diagnostics_json;
    if (typeof diagnostics === "string") {
        try {
            diagnostics = JSON.parse(diagnostics);
        }
        catch (error) {
            throw new Error(`Row ${rowNumber} diagnostics_json is not valid JSON: ${error.message}`);
        }
    }
    const applied = diagnostics?.applied_command;
    if (!applied) throw new Error(`Row ${rowNumber} has no diagnostics_json.applied_command`);
    return applied;
}

function loadActuatorRows(filePath) {
    const lines = fs.readFileSync(filePath, "utf8")
        .split(/\r?\n/)
        .map((line) => line.trim())
        .filter(Boolean);
    if (!lines.length) throw new Error(`${filePath} contains no actuator records`);

    return lines.map((line, index) => {
        const rowNumber = index + 1;
        let record;
        try {
            record = JSON.parse(line);
        }
        catch (error) {
            throw new Error(`Row ${rowNumber} is not valid JSON: ${error.message}`);
        }
        const applied = parseDiagnostics(record, rowNumber);
        const row = {
            time_s: Number(record.time_s),
            propeller_rps: Number(applied.propeller_rps),
            rudder_rad: Number(applied.rudder_rad),
        };
        if (!OUTPUT_COLUMNS.every((column) => Number.isFinite(row[column]))) {
            throw new Error(`Row ${rowNumber} has a missing or non-finite command value`);
        }
        return row;
    });
}

function validateUniformTimestep(rows) {
    if (rows.length < 2) throw new Error("At least two actuator records are required");
    const timestep = rows[1].time_s - rows[0].time_s;
    if (!Number.isFinite(timestep) || timestep <= 0) {
        throw new Error("Actuator timestamps must be strictly increasing");
    }
    const tolerance = Math.max(1e-9, timestep * 1e-6);
    for (let index = 1; index < rows.length; index += 1) {
        const interval = rows[index].time_s - rows[index - 1].time_s;
        if (Math.abs(interval - timestep) > tolerance) {
            throw new Error(
                `Non-uniform timestep at row ${index + 1}: ` +
                `${interval} s; expected ${timestep} s (tolerance ${tolerance} s)`
            );
        }
    }
    return timestep;
}

function writeCommands(filePath, rows) {
    const lines = [OUTPUT_COLUMNS.join(",")];
    for (const row of rows) {
        lines.push(OUTPUT_COLUMNS.map((column) => Number(row[column]).toPrecision(12)).join(","));
    }
    fs.mkdirSync(path.dirname(path.resolve(filePath)), {recursive: true});
    fs.writeFileSync(filePath, `${lines.join("\n")}\n`);
}

export function adaptVehicleBActuatorLog(inputPath, outputPath) {
    const rows = loadActuatorRows(inputPath);
    const timestep = validateUniformTimestep(rows);
    writeCommands(outputPath, rows);
    return {rows: rows.length, timestep};
}

function main() {
    const args = parseArgs(process.argv);
    const result = adaptVehicleBActuatorLog(args.input, args.out);
    process.stderr.write(JSON.stringify({
        input: path.resolve(args.input),
        out: path.resolve(args.out),
        command_rows: result.rows,
        timestep_s: result.timestep,
        schema: OUTPUT_COLUMNS.join(","),
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
