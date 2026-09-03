# Sensor and vehicle extensibility discipline report

Date: 2026-08-24

## Core-boundary method

The content hash was computed mechanically with:

```text
git ls-files -z packages/core core | xargs -0 shasum -a 256 | shasum -a 256
```

The pre-task and post-task hashes are both `730802b648386b6400dff2a2fbe06a9d763997825ab8c54da216cdb7def6221f`. Therefore the task modified zero tracked core files. This comparison deliberately uses the pre-task working-tree contents as its baseline, preserving the user's pre-existing `packages/core/src/simulation.ts` edit without attributing it to this work.

## Regression runs

| Checkpoint | Result | Wall time | Log |
| --- | --- | ---: | --- |
| Untouched baseline | PASS | 72.55 s | `baseline-full-regression-20260824.log` |
| After scalar hygrometer | PASS | 63.34 s | `after-hygrometer-full-regression-20260824.log` |
| First fan-rangefinder attempt | FAIL (plugin schema declaration) | 25.03 s | `after-fan-rangefinder-full-regression-20260824.log` |
| Corrected fan-rangefinder | PASS | 66.54 s | `after-fan-rangefinder-full-regression-rerun-20260824.log` |
| After fourth-vehicle attempt | PASS | 73.66 s | `after-fourth-vehicle-full-regression-20260824.log` |

The failed fan-rangefinder attempt declared `ray_count` as JSON Schema type `integer`, which the SDK's intentionally small validator does not recognize as a JavaScript runtime type. The plugin already performs the integer constraint mechanically. Changing only the plugin declaration to schema type `number` fixed the integration; no runtime or core change was needed.

## Sensor additions

### `hygrometer` — simple scalar

- `packages/sensor-sdk/src/plugins/hygrometer.ts`: 18 lines added.
- `packages/sensor-sdk/src/plugins/index.ts`: 1 export line added.
- `packages/sensor-sdk/src/index.ts`: 2 registry/import lines added.
- `packages/sensor-sdk/test/remaining-sensors.test.ts`: 2 replacement lines added, 2 removed.
- Total additions: 23 lines; total removals: 2 lines.
- Behavior: seeded bounded relative-humidity scalar, explicit metadata/schema, lifecycle handling, checkpoint state, and bit-identical replay coverage.
- Core files modified by this addition: 0.

### `fan-rangefinder` — geometric/raycast class

- `packages/sensor-sdk/src/plugins/fan-rangefinder.ts`: 18 lines added.
- `packages/sensor-sdk/src/plugins/index.ts`: 1 export line added.
- `packages/sensor-sdk/src/index.ts`: 2 registry/import lines added.
- `packages/sensor-sdk/test/remaining-sensors.test.ts`: 1 replacement line added, 1 removed.
- Total additions: 22 lines; total removals: 1 line.
- Behavior: configurable horizontal fan geometry, one controlled-service raycast per bearing, clipped ranges and hit mask, large-payload checkpoint policy, lifecycle handling, state validation, and bit-identical replay coverage.
- Core files modified by this addition: 0.

## Minimal fourth vehicle attempt

The repository already contains a fourth registered vehicle relative to primary Vehicles A/B/C: `searobotics-surveyor-m1.8`. No new fifth vehicle was invented. Its existing minimal public configuration was executed with `backends/node/src/surveyor-guidance.test.ts`.

- Result: PASS in 0.18 s.
- Output: bounded `set_thruster_mode(thrust, thrust_diff)` commands and finite surge/yaw wrench trace.
- Files modified for the attempt: 0.
- Core files modified for the attempt: 0.
- Evidence: `fourth-vehicle-minimal-attempt-20260824.log`.
- Scope: integration-only, explicitly uncalibrated; this does not add physical-validation claims.

The existing fourth-vehicle path is implemented outside `packages/core`, in the vehicle SDK, experiment resolver, and Node production adapter. Consequently the requested stop condition was not reached: the attempt did not require a core modification.
