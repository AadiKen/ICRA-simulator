# Full monorepo test triage

Run date: 2026-08-17 (America/Los_Angeles)  
Runtime: Node.js 24.13.0, npm 11.6.2, Python 3.12.4  
Full output: [`test-logs/npm-test-full-20260817T205559Z.log`](test-logs/npm-test-full-20260817T205559Z.log)

The canonical `npm test` completed twice. The second, explicitly timed run took 55.08 seconds and exited successfully. Additional gates not wired into `npm test` were then run independently so one result could not suppress another. A bucket is assigned only to a failed acceptance outcome; passing gates use `—`.

| gate name | pass/fail | runtime | bucket | notes |
|---|---:|---:|:---:|---|
| Canonical `npm test` | Pass | 55.08 s | — | Full chain completed: legacy physics/validation, offline runtime, focused ICRA, Python, external preparation, thin slice, mission artifact, release, and migration. |
| Legacy simulator and physics validations | Pass | Included in 55.08 s | — | Smoke, behavior, hydrostatics, actuator, wave, Gazebo generation/capture, convergence, motion, lifecycle, and six-DOF plant checks passed. |
| Offline runtime and USV-Bench tests | Pass | Included in 55.08 s | — | Worker runtime, typed positioning, Vehicle C allocation, USV-Bench execution, and agent evaluation passed. |
| Surveyor guidance standalone test | Pass | <1 s | — | This test file was not named by the root script, so it was run explicitly. Mission validation, integer hardware-command bounds, nonzero propulsion, and differential steering passed. |
| Focused ICRA TypeScript suite | Pass | Included in 55.08 s | — | Schema, core, vehicle, sensor, environment, metrics, MCP, reference, and benchmark tests passed. |
| Python clients, tensor backend, and baselines | Pass | 13.58 s reported by unittest | — | 3 client, 11 tensor, and 1 baseline tests passed. Maximum reported tensor error was `4.97e-14`; no NaN was reported. |
| External-data preparation | Pass | Included in 55.08 s | — | Capytaine integration, hull convergence, combined damping, dataset manifests, KVLCC2, MARIN fixtures/scoring policy, wPCC, MMG, and AERO4River passed. Required restricted/cache-only datasets were already provisioned locally and checksum verification passed. |
| Thin-slice artifact | Pass | Included in 55.08 s | — | Generated Parquet state and summary artifacts. |
| Mission-length artifact check | Pass | Included in 55.08 s | — | Existing artifact contained 15,000 rows and 8 row groups. PyArrow printed sandbox-denied CPU-cache `sysctlbyname` warnings, but read and assertions completed successfully. |
| Release gate | Pass | Included in 55.08 s | — | Replay test, TypeScript build, Vite production build, UI artifact checks, and anonymous release generation passed. Vite's large-chunk message is a warning, not a failure. |
| Migration gate | Pass | Included in 55.08 s | — | Production core wiring passed with zero metric deltas and zero checksum changes on preserved surfaces; the audited Gazebo supersession remained explicit. |
| Policy campaign | Pass | <1 s shell resolution | — | 324 configurations completed; all policies had finite rate 1. The result remains software evaluation, not physical validation. |
| Ikeda unit gate | Pass | 0.093 s | — | One test passed. It does not promote bootstrap viscous coefficients to a complete Ikeda validation. |
| Coupled6 wave-response software campaign | Pass | 3 s | — | Four 6,000-step Vehicle B/C traces passed determinism and boundedness checks; every trace reported `finite: true`. |
| Coupled6 physical validation | Fail | 3 s campaign evidence | (C) | Substantive accepted limitation, not a runtime failure: the generated campaign explicitly reports `software-campaign-passed-physical-validation-blocked` and `is_physical_validation_evidence: false`. All four traces are finite and bounded (`max |heave| < 0.14 m`, `max |roll| < 0.35 rad`, `max |pitch| < 0.026 rad`), so this is not a crash, NaN, timeout, or missing-file result. The documented unmet criterion is agreement with independent measured RAO / regular-wave or free-decay evidence; the current coefficients and hulls are unvalidated engineering estimates. |
| VRX construction comparison | Pass | <1 s | — | Construction-effort comparison tests passed. This is not the Docker runtime benchmark. |
| Vehicle B production integration | Pass | 1 s | — | Coupled6 trace was finite, out-of-plane DOFs were free, and MMG force components were present. Artifact status remains `integration-passed-dynamics-unvalidated`. |
| Vehicle B MARIN physical acceptance | Fail | <1 s | (C) | Confirmed substantive tolerance failure by running `npm run score:kvlcc2-marin`. All six trajectories completed with finite numeric metrics; measured propeller replay error was exactly 0 and rudder replay RMSE was approximately `1e-15`. Failures are against precommitted 5% turning and 10% zig-zag limits—not crashes or missing data. Examples: port turning advance error 6.46% > 5%; starboard turning advance/tactical errors 8.33%/8.00% > 5%; port 10° first overshoot error 148.81% > 10%; port 20° first/second errors 212.86%/54.74% > 10%. Artifact status is `experimental-trajectory-scored-fail`. |
| Mission-length regeneration | Pass | 6 s | — | Regenerated the complete 15,000-step, 300-second artifact; `success: true`, finite summary metrics, and fixed-step termination. Follow-up Parquet assertions passed in 1 s. |
| MSS acceptance | Pass | <1 s | — | Five pinned planar reference maneuvers passed their position, heading, and body-speed criteria. |
| Behavior supersession | Pass | <1 s | — | Default-forbidden behavior and legacy checksum checks passed. |

## Failure summary

- Bucket A: none.
- Bucket B: none. No missing credentials, runtimes, datasets, or files blocked an executed gate.
- Bucket C: two physical-evidence outcomes: Vehicle B MARIN maneuver acceptance and general coupled6 physical validation. Both were inspected at the metric/artifact level. Neither conceals a crash, NaN, timeout, or missing file.

## Execution note

The first independent-gate harness stopped after the passing policy campaign because it assigned to zsh's read-only `status` variable. The log contains `zsh:6: read-only variable: status`. This was an orchestration-script error outside the repository, not a monorepo gate result. The harness was corrected to use `gate_rc`, and every remaining gate then ran to completion.
