import {spawnSync} from "node:child_process";
import {createHash} from "node:crypto";
import {mkdirSync, readFileSync, writeFileSync} from "node:fs";
import {join, resolve} from "node:path";

const root = resolve(new URL("../../", import.meta.url).pathname);
const golden = join(root, "validation/pre-migration/golden");
const policy = JSON.parse(readFileSync(join(root, "validation/pre-migration/acceptance-policy.json"), "utf8"));
const sha = (data) => createHash("sha256").update(data).digest("hex");
const deltas = [];
const rows = [];
const goldenManifest = JSON.parse(readFileSync(join(golden, "manifest.json"), "utf8"));
for (const record of goldenManifest.records) {
  const actualHash = sha(readFileSync(join(golden, record.file)));
  if (actualHash !== record.sha256) throw new Error(`Immutable golden checksum mismatch: ${record.file}`);
}
const supplementalDirectory=join(root,"validation/pre-migration/supplemental");
const supplementalManifest=JSON.parse(readFileSync(join(supplementalDirectory,"manifest.json"),"utf8"));
for(const record of supplementalManifest.records){const actualHash=sha(readFileSync(join(supplementalDirectory,record.file)));if(actualHash!==record.sha256)throw new Error(`Immutable supplemental golden checksum mismatch: ${record.file}`);}

function runJson(args) {
  const result = spawnSync(process.execPath, args, {cwd: root, encoding: "utf8"});
  if (result.status !== 0) throw new Error(`${args[0]} failed:\n${result.stdout}\n${result.stderr}`);
  return JSON.parse(result.stdout.slice(result.stdout.indexOf("{")));
}

function compare(expected, actual, path, tolerance) {
  if (typeof expected === "number" && typeof actual === "number") {
    const absolute = Math.abs(actual - expected);
    const relative = absolute / Math.max(Math.abs(expected), 1e-15);
    if (absolute > tolerance.absolute && relative > (tolerance.relative ?? Infinity)) {
      deltas.push({path, expected, actual, absolute, relative, tolerance});
    }
    return;
  }
  if (Array.isArray(expected)) {
    if (!Array.isArray(actual) || expected.length !== actual.length) {
      deltas.push({path, expected: `array[${expected.length}]`, actual: Array.isArray(actual) ? `array[${actual.length}]` : typeof actual});
      return;
    }
    expected.forEach((value, index) => compare(value, actual[index], `${path}[${index}]`, tolerance));
    return;
  }
  if (expected && typeof expected === "object") {
    for (const key of Object.keys(expected)) compare(expected[key], actual?.[key], `${path}.${key}`, tolerance);
    return;
  }
  if (expected !== actual) deltas.push({path, expected, actual, requirement: "bit-exact"});
}

const commands = {
  sixdof: {args: ["validation/sixDofPlantTest.js"], tolerance: policy.tolerance_bounded.coupled6_invariants},
  waves: {args: ["validation/waveParityTest.js"], tolerance: policy.tolerance_bounded.wave_response},
  convergence: {args: ["validation/convergenceTest.js"], tolerance: policy.tolerance_bounded.convergence_metrics}
};
for (const [name, command] of Object.entries(commands)) {
  const before = deltas.length;
  compare(JSON.parse(readFileSync(join(golden, `${name}.json`), "utf8")), runJson(command.args), name, command.tolerance);
  rows.push({surface: name, metric_delta_count: deltas.length - before, checksum_changes: 0});
}
const hydroBefore=deltas.length;
compare(JSON.parse(readFileSync(join(supplementalDirectory,"hydrostatics-large-angle.json"),"utf8")),runJson(["validation/pre-migration/hydrostatics-large-angle.mjs"]),"hydrostatics-large-angle",policy.tolerance_bounded.coupled6_invariants);
rows.push({surface:"hydrostatics-large-angle",metric_delta_count:deltas.length-hydroBefore,checksum_changes:0});
const actuatorBefore=deltas.length;
compare(JSON.parse(readFileSync(join(supplementalDirectory,"actuator-legacy-trace.json"),"utf8")),runJson(["validation/pre-migration/actuator-legacy-trace.mjs"]),"actuator-command-state-thrust-wrench",policy.tolerance_bounded.actuator_trace);
rows.push({surface:"actuator-command-state-thrust-wrench",metric_delta_count:deltas.length-actuatorBefore,checksum_changes:0});
const replay = spawnSync(process.execPath, ["--experimental-strip-types", "backends/node/src/checkpoint-replay.test.ts"], {cwd: root, encoding: "utf8"});
if (replay.status !== 0) throw new Error(`Checkpoint replay identity failed:\n${replay.stdout}\n${replay.stderr}`);
rows.push({surface: "checkpoint-replay-mid-actuator-lag", metric_delta_count: 0, checksum_changes: 0});

const mssBefore = deltas.length;
const expectedMss = JSON.parse(readFileSync(join(golden, "mss.json"), "utf8"));
const actualMss = runJson(["validation/mssAcceptance.js"]);
for (let i = 0; i < expectedMss.results.length; i += 1) {
  const expected = expectedMss.results[i], actual = actualMss.results[i];
  compare(expected.name, actual?.name, `mss[${i}].name`, {absolute: 0});
  compare(expected.samples, actual?.samples, `mss[${i}].samples`, {absolute: 0});
  compare(expected.metrics.positionRmse, actual?.metrics.positionRmse, `mss[${i}].positionRmse`, policy.tolerance_bounded.mss_position_rmse_m_delta);
  compare(expected.metrics.headingRmse, actual?.metrics.headingRmse, `mss[${i}].headingRmse`, policy.tolerance_bounded.mss_heading_rmse_rad_delta);
  compare(expected.metrics.bodySpeedRmse, actual?.metrics.bodySpeedRmse, `mss[${i}].bodySpeedRmse`, policy.tolerance_bounded.mss_body_speed_rmse_mps_delta);
  compare(true, actual?.passed, `mss[${i}].passed`, {absolute: 0});
}
rows.unshift({surface: "mss-planar3", metric_delta_count: deltas.length - mssBefore, checksum_changes: 0});

const gazeboRun = spawnSync(process.execPath, ["gazebo/generateGazeboParity.js", "--all"], {cwd: root, encoding: "utf8"});
if (gazeboRun.status !== 0) throw new Error("Gazebo generation failed");
const gazeboExpected = JSON.parse(readFileSync(join(golden, "gazebo-generation.json"), "utf8"));
const gazeboSupersessionPath = "validation/pre-migration/supersessions/gazebo-live-runtime-fix.json";
const gazeboSupersession = JSON.parse(readFileSync(join(root, gazeboSupersessionPath), "utf8"));
for (const retained of [gazeboSupersession.original_golden, gazeboSupersession.golden_manifest,
  gazeboSupersession.superseded_runtime_artifact, gazeboSupersession.acceptance_runtime_artifact]) {
  const actual = sha(readFileSync(join(root, retained.path)));
  if (actual !== retained.sha256) throw new Error(`Gazebo supersession checksum mismatch: ${retained.path}`);
}
const liveGazeboReport = JSON.parse(readFileSync(join(root, gazeboSupersession.acceptance_runtime_artifact.path), "utf8"));
if (liveGazeboReport.status !== gazeboSupersession.acceptance_runtime_artifact.required_status ||
    !Array.isArray(liveGazeboReport.results) || liveGazeboReport.results.length === 0 ||
    liveGazeboReport.results.some((result) => result.passed !== true)) {
  throw new Error("Gazebo supersession requires a fully passing retained live-runtime report");
}
const permittedGazeboChanges = new Map(gazeboSupersession.checksum_changes.map((change) => [change.path, change]));
let gazeboChanges = 0;
let gazeboSupersededChanges = 0;
for (const file of gazeboExpected.files) {
  const actual = sha(readFileSync(join(root, "gazebo/generated", file.path)));
  if (actual !== file.sha256) {
    const permitted = permittedGazeboChanges.get(file.path);
    if (permitted?.old_sha256 === file.sha256 && permitted.new_sha256 === actual) {
      gazeboSupersededChanges += 1;
      permittedGazeboChanges.delete(file.path);
    } else {
      gazeboChanges += 1;
      deltas.push({path: `gazebo.${file.path}`, expected: file.sha256, actual, requirement: "bit-exact-or-reviewed-supersession"});
    }
  }
}
if (permittedGazeboChanges.size > 0) {
  for (const [path, change] of permittedGazeboChanges) {
    deltas.push({path: `gazebo.${path}`, expected: change.new_sha256, actual: "unchanged-or-missing", requirement: "supersession-must-be-exact-and-complete"});
  }
}
rows.push({surface: "gazebo-generation", metric_delta_count: 0, checksum_changes: gazeboChanges,
  superseded_checksum_changes: gazeboSupersededChanges, supersession: gazeboSupersessionPath});

const report = {passed: deltas.length === 0, acceptance_policy: "validation/pre-migration/acceptance-policy.json", metric_delta_count: deltas.length, rows, deltas};
if(process.argv.includes("--post-migration-report")){
  const reportDirectory=join(root,"validation/pre-migration/reports");mkdirSync(reportDirectory,{recursive:true});
  const retained={...report,artifact_kind:"final-post-migration-comparison",migration_components:["mass-inertia","coriolis","damping","hydrostatics","actuators","wind-current","waves"],production_core_wiring:"schema.js, DynamicsCore, CoupledSixPlant, and Gazebo actuation import typed-core physics directly",compatibility_facades_scope:"retained only for immutable golden and historical validation harnesses; not used by production",golden_manifest_sha256:sha(readFileSync(join(golden,"manifest.json"))),supplemental_manifest_sha256:sha(readFileSync(join(supplementalDirectory,"manifest.json")))};
  writeFileSync(join(reportDirectory,"08-post-migration.json"),`${JSON.stringify(retained,null,2)}\n`);
}
const reportIndex = process.argv.indexOf("--report");
if (reportIndex >= 0) {
  const component = process.argv[reportIndex + 1];
  const sequence = ["mass-inertia", "coriolis", "damping", "hydrostatics", "actuators", "wind-current", "waves"];
  const sequenceIndex = sequence.indexOf(component);
  if (sequenceIndex < 0) throw new Error(`Unknown migration component report: ${component}`);
  const reportDirectory = join(root, "validation/pre-migration/reports");
  mkdirSync(reportDirectory, {recursive: true});
  const retained = {...report, component, sequence_index: sequenceIndex + 1, golden_manifest_sha256: sha(readFileSync(join(golden, "manifest.json"))), supplemental_manifest_sha256:sha(readFileSync(join(supplementalDirectory,"manifest.json"))), ...(component==="hydrostatics"?{model_scope:"coupled6 uses constant linear stiffness from GM_T/GM_L; the sampled-geometry path recomputes displaced volume and center of buoyancy nonlinearly. Large-angle parity is not nonlinear coupled6 validation."}:{}), ...(component==="actuators"?{migration_scope:"Behavior-preserving move into @bcod/core with the legacy import path retained only as a compatibility facade.",behavior_mapping:{dead_zone:"configuration remains ignored for exact legacy parity",failure_mode:"command metadata remains ignored for exact legacy parity",lag_rate_saturation_order:"unchanged",checkpoint_state:"unchanged and replayed mid-transient"},capability_gate:"Functional dead zones, failed-off/stuck behavior, events, energy, and integrated allocation remain a separate intentional behavior change."}:{}), ...(component==="waves"?{evidence_scope:"Migration parity only. A green gate does not validate wave physics; radiation damping remains zero until resolved Capytaine coefficients, followed by free-decay and frequency-response evidence."}:{})};
  writeFileSync(join(reportDirectory, `${String(sequenceIndex + 1).padStart(2, "0")}-${component}.json`), `${JSON.stringify(retained, null, 2)}\n`);
}
console.log(JSON.stringify(report, null, 2));
if (deltas.length) process.exitCode = 1;
