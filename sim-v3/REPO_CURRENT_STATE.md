# BCOD Simulator Repo Current State

This document explains what this repo is, how it currently works, what it can do today, what it cannot do yet, and what the broader product/engineering direction appears to be from the current code and planning notes.

The repo is a local, browser-first autonomous surface vessel simulator for the BCOD demo. It combines a headless JavaScript simulation core, Three.js visualization, simplified marine dynamics, water/current/wave modeling, sensor simulation, optional local sensor streaming, mission metrics, CSV export, and validation tooling.

The current system is not a full robotics simulator, not a Gazebo replacement, and not a production training-data pipeline. It is a practical demo and validation environment whose long-term direction is to stay lightweight in the browser while becoming physically credible enough to support mission demos, controller integration, and later headless rollouts.

## Entry Points

Use a local HTTP server from the repo root:

```bash
python3 -m http.server 8000
```

Then open:

- `http://localhost:8000/threeSim.html` for the developer/debug simulator.
- `http://localhost:8000/demo.html` for the guided mission demo.

Run the validation suite with:

```bash
npm test
```

Useful targeted scripts:

```bash
npm run test:smoke
npm run test:physics
npm run test:validation
npm run test:gazebo
npm run test:gazebo-capture
npm run gazebo:generate
npm run gazebo:capture
npm run gazebo:plot
```

## High-Level Architecture

The repo is organized around four practical layers.

### 1. Simulation Core

`schema.js` owns the main runtime model:

- scenario configuration classes
- simulation state classes
- boat facade/model
- controller command model
- skipper/guidance model
- sensor model
- environment model
- water field model
- mission progress and failure checks
- metrics
- logging
- the `simulator` class

`core/` owns the pure physics pieces:

- rigid-body state
- NED/body frame helpers
- vehicle parameter construction
- 3-DOF dynamics
- RK4 and semi-implicit Euler integrators
- hydrodynamic damping
- added-mass Coriolis
- current coupling
- actuator models
- hydrostatics/wave helpers
- partial 6-DOF math helpers

The core is designed to run without the browser renderer. Node tests instantiate the same simulator and physics modules directly.

### 2. Browser Runtime

`webSimRunner.js` drives `threeSim.html`. It creates the default scenario, owns start/pause/reset/step behavior, updates debug panels, exports CSV logs, renders sensor feed cards, and can stream sensor outputs over local WebSockets.

`demoRunner.js` drives `demo.html`. It manages the guided UI flow, converts UI choices into a `scenarioConfig`, starts the simulator, attaches the Three.js renderer-backed sensor provider, updates live stats, and renders final result graphs.

### 3. Rendering And Sensor Provider

`threeStateRenderer.js` renders the developer view. `demoRenderer.js` renders the guided demo world. Both use vendored Three.js from `vendor/three.module.js`.

`threeSensorProvider.js` provides renderer-backed observations for:

- day camera
- night camera
- LiDAR

It uses mounted sensor poses from the simulator state, not the UI camera. Camera frames are rendered through an offscreen `THREE.PerspectiveCamera`; LiDAR is generated through Three.js raycasting against scene targets.

### 4. Validation And Gazebo Parity Tooling

`validation/` contains behavior, physics, parity, convergence, wave, hydrostatics, actuator, motion-quality, and Gazebo tooling tests.

`gazebo/` can generate Gazebo SDF worlds/models/manifests for Otter-style parity maneuvers, capture pose logs from Gazebo when Gazebo is installed, normalize those logs into BCOD/NED columns, and compare against standalone BCOD replays.

Gazebo is treated as an offline calibration oracle, not as a runtime dependency for the browser simulator.

## Runtime Flow

A simulator run starts with a `scenarioConfig`, which includes:

- `simConfig`: simulation rate, duration, seed, ground-truth permission
- `boatConfig`: limits, mass, geometry, damping, hydrodynamics, buoyancy, actuator-related values, start pose
- `sensorConfig`: installed sensor objects
- `envConfig`: bounds, obstacles, denied/favored zones, water field, visibility, time of day
- `goalConfig`: waypoints and tolerance
- `controlConfig`: controller mode, frequency, strategy, timeout, guidance mode

`createInitialSimState()` builds the mutable runtime state. A `simulator` instance then owns the live models and logs.

Each `simulator.step()` does roughly this:

1. Stop immediately if the mission is already complete, failed, or past duration.
2. Sample the local environment around the boat, including current, waves, surface height, normals, and hull sample points.
3. Collect observations from active sensors when their sample cadence is due.
4. Update the controller command when the controller cadence is due.
5. Build the belief state used by guidance.
6. Convert command waypoints into desired heading, desired speed, acceleration demand, and yaw command.
7. Convert guidance into actuator force demand.
8. Apply actuator lag/saturation and build body-frame wrench.
9. Integrate the 3-DOF dynamics core.
10. Apply presentation seakeeping for visual roll/pitch.
11. Update waypoint progress and failure state.
12. Update metrics, logs, time, and sensor feed state.

## Coordinate Conventions

The browser-facing world is Three.js-style, right-handed, and Y-up:

- `x` = east/lateral
- `y` = up
- `z` = north/forward
- `heading` = yaw around world `y`

The physics core uses marine NED/body conventions:

- position: `N`, `E`, `D`
- body velocity: `u`, `v`, `r`
- yaw: `psi`

The bridge between these conventions lives in `schema.js` and `core/frameAdapters.js`. CSV exports map browser state into MSS/Gazebo-friendly columns such as `north`, `east`, `up`, `u`, `v`, and `heading`.

## What It Currently Does

### Mission Simulation

The simulator can:

- run a fixed-step mission loop
- follow waypoint missions
- start, pause, reset, step once, or run until done in the browser
- mark waypoints complete
- fail missions on coarse obstacle collision
- fail missions when the boat leaves bounds
- stop at configured duration
- log boat truth, belief, active sensors, and metrics

The guidance is a local heuristic skipper, not the real BCOD planner. It chooses heading/speed behavior toward waypoints and activates configured sensors through a simple strategy.

### Boat Dynamics

The current physical core is a force-space, Fossen-style 3-DOF marine craft model with:

- surge
- sway
- yaw
- current-relative hydrodynamic damping
- added-mass Coriolis coupling
- twin-thruster style actuation
- actuator lag and saturation
- wave excitation approximation
- hull water sampling
- RK4 integration

Roll and pitch are mostly presentation/sensor-pose effects, not full physics states. The visual boat rocks with water normals through a spring-damper seakeeping approximation.

### Water And Environment

The water field supports:

- configurable waves
- current vector
- surface height sampling
- water velocity
- water acceleration
- water normal
- submerged/depth information at sample points
- hull sample grids

The environment also supports:

- rectangular world bounds
- circular obstacles
- denied sensor zones
- favored sensor zones as represented configuration/UI regions
- visibility presets for weather-like effects

### Sensors

State-derived sensors:

- GPS reports pose, position, and velocity.
- IMU reports pose, acceleration, angular velocity, angular acceleration, and orientation.

Renderer-derived sensors, when a `ThreeSensorProvider` is attached:

- day camera renders mounted camera frames
- night camera renders mounted camera frames with post-processing/weather-style effects
- LiDAR raycasts against Three.js scene targets and returns ranges, hit points, hit count, and min range

Without the Three.js provider, camera and LiDAR observations intentionally become placeholders.

EXO2 exists as a sensor config/type, but its behavior is not defined yet.

### Browser Interfaces

`threeSim.html` currently provides:

- developer-oriented simulation controls
- debug panels
- water visualization modes
- live sensor cards
- CSV export
- optional local sensor streaming

`demo.html` currently provides a guided workflow for:

- vehicle and sensor setup
- obstacle placement
- sensor-zone placement
- weather/current/wave configuration
- route creation
- live mission run
- result summary graphs

### Sensor Streaming

Sensor streaming can be enabled with:

```text
?streamSensors=1
```

or local storage:

```js
localStorage.setItem("bcodStreamSensors", "1")
```

Streams are sent to:

- camera: `ws://127.0.0.1:8765`
- LiDAR: `ws://127.0.0.1:8766`
- telemetry: `ws://127.0.0.1:8767`

Receiver examples live in `data recievers/`. The folder name is misspelled and contains a space, so shell commands need quoting.

### Metrics And Results

The simulator tracks:

- active sensor cost
- movement cost
- total energy/cost
- speed
- per-step and cumulative values
- power/result samples used by the demo graphs

The guided demo computes result displays such as energy used, estimated reduction, sensor activation timeline, and sensor energy usage.

### CSV And Comparison

The developer view can export CSV logs. The validation tooling can also export BCOD parity logs from Node.

`csvCompare.py` compares JavaScript simulator CSV exports with an MSS baseline and can generate:

- `comparison_report.png`
- `comparison_report.txt`

### Gazebo Parity

The repo contains tooling to:

- define parity maneuvers
- generate Gazebo model/world SDF files
- generate manifests
- publish plant-level body wrench commands
- convert Gazebo ENU pose logs into BCOD NED samples
- derive body velocities
- resample logs
- reject invalid captures
- compare frozen golden CSV logs against standalone BCOD replays

Current generator tests explicitly document that the Gazebo plant parity target validates integration, frames, thrust mapping, and drag, but does not currently rely on DART `<fluid_added_mass>` because of pose-reset instability concerns in the current generated setup.

## What It Currently Cannot Do Yet

### It Does Not Use The Real BCOD Controller

The controller path is still local/heuristic. There is no live remote BCOD network/API integration, no timeout/retry protocol, no command validation layer, and no real short-horizon trajectory contract implemented end to end.

### It Is Not A Full 6-DOF Marine Simulator

The operational plant is still 3-DOF for horizontal vessel motion. There are 6-DOF helper functions and roadmap direction, but the current mission simulator does not fully integrate heave, roll, pitch, depth, submarines, aerial drones, or multi-body dynamics.

### Roll/Pitch Are Not Fully Coupled Physics

Roll and pitch are used for presentation and sensor pose. They are not fully part of the force-integrated rigid-body plant. This is acceptable for the current surface-vessel demo, but it is not enough for high-fidelity seakeeping, capsizing, or underwater vehicle simulation.

### Contacts Are Coarse

Obstacle interaction is a mission failure check, not a physical contact solver. The boat does not bounce, scrape, ground, collide with complex geometry, or resolve multi-body contacts.

### Sensors Are Still Simplified

Current limitations include:

- no stochastic sensor noise model
- no latency queues
- no calibration/error models
- no EXO2 water-chemistry behavior
- no camera semantic labeling
- no radar/sonar
- no real sensor degradation model beyond simple rendering/weather effects
- LiDAR depends on Three.js scene geometry, not a physics-grade world representation

### Scenario Persistence Is Limited

The browser demo can construct scenarios interactively, but there is no robust JSON scenario loader/saver, schema version migration, strict validation, or reusable scenario library beyond the current preset/code paths.

### Training Pipeline Is Not Built

The core is structurally suitable for headless execution, but this repo does not yet provide a full training-data pipeline. Missing pieces include:

- dataset orchestration
- batch rollout runner
- seeded stochastic environment generation
- replay artifact packaging
- large-scale parameter sweeps
- production scenario specs
- controller-in-the-loop training API

### The Demo Is Not A Production Web App

There is no build system, bundler, deploy pipeline, authentication, backend, or persistence service. Browser modules are loaded directly. Some imports use manual cache-busting query strings.

### Visual Fidelity Is Purposeful But Limited

The renderer is good enough for a local demo/debug simulator, but it is not AAA visual realism. The active boat rendering is mostly procedural even though a GLTF boat asset exists under `models/boat/`.

### Gazebo Is Not Fully Automated For Every User

Gazebo capture requires a local Gazebo installation and the right CLI/runtime environment. The normal test suite can run without Gazebo because it tests generators and frozen/local logic, but new captures are environment-dependent.

## What The Repo Appears Intended To Do Eventually

The planning docs point toward a simulator with two long-term purposes.

### 1. Interactive Evaluation And Demonstration

The simulator should let prospective users, presenters, and researchers:

- configure realistic mission scenarios
- define a boat and sensor suite
- place obstacles and sensing zones
- configure waves/current/weather
- run a BCOD-controlled mission
- visualize route progress, active sensors, uncertainty, predicted path, and energy/cost
- compare selective BCOD sensing against an all-sensors-on benchmark
- rerun and adjust scenarios smoothly in a browser

### 2. Training-Data And Evaluation Rollouts

The same simulation core should eventually support headless runs for:

- scenario files as input
- closed-loop controller interaction
- reproducible stochastic rollouts
- generated sensor outputs
- training/evaluation traces
- batch runs outside the browser

The repo direction is to keep the simulation core independent from rendering and UI so this headless path remains possible.

## Current Roadmap Themes

The active roadmap implied by the code and plans is:

- integrate a real BCOD controller interface
- define the controller observation/command contract
- add strict scenario validation and serialization
- improve sensor realism with noise, latency, degradation, and EXO2 behavior
- build an all-sensors-on benchmark into results
- improve replay/log export workflows
- continue Gazebo parity work with frozen golden logs
- expand from current 3-DOF USV behavior toward richer 6-DOF vehicles when needed
- keep browser presentation polished while preserving a grounded simulation core

Some older plan files are stale. For example, they say browser integration and rendered camera/LiDAR were not started, but current code now has `threeSim.html`, `demo.html`, Three.js renderers, `ThreeSensorProvider`, rendered camera observations, LiDAR raycasting, and result graphs. Treat the plan files as historical roadmap notes, and treat the code plus `README.md`/`SIMULATOR_DEEP_DIVE.md` as the more current state.

## Important Files

| Path | Purpose |
| --- | --- |
| `README.md` | Current quick start and repo overview. |
| `SIMULATOR_DEEP_DIVE.md` | Detailed explanation of simulator lifecycle and physics/runtime design. |
| `Gazebo_Parity_Plan.md` | Long-form parity strategy and target methodology. |
| `schema.js` | Main simulator schema, runtime state, model classes, sensor/environment/metric/log logic. |
| `core/` | Pure physics, frames, integrators, forces, vehicle parameters, vehicle presets. |
| `scenarioPresets.js` | Default developer-viewer scenario. |
| `webSimRunner.js` | `threeSim.html` runtime controller. |
| `threeStateRenderer.js` | Developer Three.js renderer. |
| `demoRunner.js` | Guided demo workflow, scenario builder, stats/results UI. |
| `demoRenderer.js` | Guided demo renderer. |
| `threeSensorProvider.js` | Renderer-backed camera and LiDAR observations. |
| `sensorStreamPublisher.js` | Optional WebSocket publishing for camera, LiDAR, telemetry. |
| `validation/` | Node validation and parity tests. |
| `gazebo/` | Gazebo SDF generation, capture, conversion, and manifests. |
| `data/` | Baseline CSVs for comparison. |
| `data recievers/` | Example local WebSocket receivers. |
| `vendor/` | Vendored Three.js and water normal texture. |
| `sensor-icons/` | Demo UI sensor icons. |
| `models/boat/` | Boat GLTF asset files. |

## Known Gotchas

- Run through an HTTP server; opening the HTML files directly can break ES module imports and WebGL asset loading.
- Browser runtime errors show in DevTools, not in the Python HTTP server output.
- `data recievers` is misspelled and has a space.
- There is no bundler; this is plain browser ES modules.
- Several imports include manual cache-busting query strings.
- The current git status may show unrelated changes outside this repo root because the parent workspace has other website folders.
- Gazebo capture scripts need Gazebo locally installed; ordinary browser use and most Node validation do not.

## Short Summary

This repo currently works as a local browser simulator and validation sandbox for a BCOD autonomous surface vessel demo. It can run waypoint missions with simplified but meaningful marine dynamics, waves/current, obstacles, zones, rendered camera/LiDAR feeds, metrics, CSV export, local sensor streaming, and substantial Node/Gazebo-parity tooling.

It cannot yet act as the final BCOD-integrated simulator, a full 6-DOF/high-fidelity hydrodynamics engine, a production training-data generator, or a production web application. The desired direction is clear: keep the browser demo smooth, keep the simulation core physically grounded and headless-capable, add the real BCOD control loop, improve sensor realism, and use Gazebo only as an offline calibration/reference path.
