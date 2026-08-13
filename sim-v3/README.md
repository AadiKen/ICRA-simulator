# BCOD Mission Simulator

This repository contains a browser-based autonomous surface vessel simulator for the BCOD demo. It models a small boat moving through a bounded water field, following waypoints, reacting to waves/current/obstacles, activating sensors, and exposing the same run through two browser UIs.

The dynamics core has been rebuilt around a force-space, Fossen-style 3-DoF marine craft model. The simulator still exposes the legacy browser state shape for rendering/sensors, but surge, sway, yaw, current coupling, hydrodynamic damping, Coriolis terms, and twin-thruster actuation now run through the pure `core/` modules.

- `threeSim.html`: a developer-oriented state viewer for stepping the simulation, inspecting physics/debug state, viewing raw sensor feeds, and exporting CSV logs.
- `demo.html`: a guided mission demo for configuring the boat, sensors, obstacles, zones, weather, water conditions, route, and then running the mission.

The current goal of the codebase is not to be a full naval simulator. It is a practical, visual BCOD demo simulator that connects mission planning, simplified vessel dynamics, water physics, sensor behavior, and export/comparison tooling in one local browser workflow.

The runtime now exposes two physics modes:

- `coupled6` (interactive default): authoritative NED position, quaternion attitude, and six body velocities with 6×6 inertia, Coriolis, damping, still-water hydrostatics, current, wind, and six-axis actuator wrenches.
- `planar3`: the preserved surge/sway/yaw plant used by legacy regression and MSS maneuvering comparisons.

In the first `coupled6` milestone, rendered waves are visual only (`waveCoupling="none"`). They do not apply physical wave excitation to the six-axis plant.

For a detailed end-to-end explanation of the simulator lifecycle, state ownership, control loop, physics core, water model, sensors, renderers, metrics, logging, validation, and current limitations, see [`SIMULATOR_DEEP_DIVE.md`](SIMULATOR_DEEP_DIVE.md).

## Quick Start

### Local weather credentials

ERA5 historical replay requires a local Copernicus CDS personal access token. Configure `environment.data_sources.era5.credentials` with one of these untracked sources:

- `{ "source": "environment", "env_var": "BCOD_CDS_API_KEY" }`, then export `BCOD_CDS_API_KEY='<PERSONAL-ACCESS-TOKEN>'`.
- `{ "source": "file", "path": "~/.cdsapirc" }`, where the file contains:

  ```yaml
  url: https://cds.climate.copernicus.eu/api
  key: <PERSONAL-ACCESS-TOKEN>
  ```

The token is never copied into resolved experiments or provenance. Accept the ERA5 dataset licence in the CDS website before the first request. Missing credentials fail with `CDS API key not configured`. NWS separately requires its non-secret application/contact identity in `environment.data_sources.nws.user_agent`.

Run the simulator from a local HTTP server. Opening the HTML files directly from disk can break ES module imports, WebGL assets, and browser security rules.

```bash
cd /Users/aadikenchammanaold/Desktop/LEADCAT/Website/bcod-demo-no-model/sim-v3
npm run serve:web
```

Port 8765 is used because port 8000 may be reserved by Docker Desktop on macOS.
Open `http://localhost:8765/demo.html` for the main interface or
`http://localhost:8765/threeSim.html` for the engineering viewer.

Then open:

- `http://localhost:8000/threeSim.html` for the state/debug viewer.
- `http://localhost:8000/demo.html` for the guided demo.

Important: the browser renderer uses local vendored assets:

- `vendor/three.module.js`
- `vendor/waternormals.jpg`

If the page is blank or module loading fails, check the browser console first. The Python server terminal only shows HTTP requests; JavaScript module/runtime failures appear in browser DevTools.

## Main Files

| File | Purpose |
| --- | --- |
| `schema.js` | Simulator schema, legacy-compatible state objects, control, sensors, metrics, logging, environment, and glue into the new dynamics core. |
| `core/` | Pure physics modules: NED/body frames, rigid body state, vehicle parameters, mass matrix, integrator, force models, and vehicle presets. |
| `adapters/` | Boundary adapters for legacy guidance, render-frame conversion, and planner/log-friendly rows. |
| `validation/` | Core validation maneuvers and Otter-style physics checks. |
| `scenarioPresets.js` | Default scenario used by `threeSim.html`: boat parameters, sensors, waves, obstacles, waypoints, and controller config. |
| `webSimRunner.js` | Runtime controller for `threeSim.html`: starts/stops/steps the sim, updates panels, exports CSV, streams sensors. |
| `threeStateRenderer.js` | Three.js renderer for the developer state viewer, including water debug modes and hull/sample visualizations. |
| `demo.html`, `demoRunner.js`, `demoRenderer.js` | Guided mission configuration and cinematic/efficient demo renderer. |
| `threeSensorProvider.js` | Legacy browser-only compatibility provider for camera and LiDAR feeds; production experiments use typed Sensor SDK plugins. |
| `sensorStreamPublisher.js` | Optional WebSocket publisher for camera, LiDAR, and telemetry feeds. |
| `simulatorSmokeTest.js` | Node smoke test for core stepping, logs, commands, local environment, and metrics. |
| `physicsBehaviorTest.js` | Node behavior tests for stability, damping, guidance, wave response, sensor pose, and zone behavior. |
| `mssReference.py` | Python reference implementation of a simplified MSS/Otter-style 3-DOF USV model. |
| `csvCompare.py` | Compares JS simulator CSV exports against an MSS baseline and generates report files. |
| `data/` | Baseline CSV files used for comparison. |
| `data recievers/` | Python WebSocket receivers for camera, LiDAR, and telemetry streams. |
| `sensor-icons/` | SVG icons used by the guided demo sensor UI. |
| `models/boat/` | Boat GLTF asset files, currently present but the active renderers mostly use procedural boat geometry. |

## How The Simulator Works

The simulator is configured through a `scenarioConfig`, which bundles:

- `simConfig`: timestep, duration, seed, and whether ground truth is allowed.
- `boatConfig`: speed/acceleration limits, mass, dimensions, damping, inertia, hydrodynamics, buoyancy, drag, and angular stability parameters.
- `sensorConfig`: legacy browser-demo GPS, IMU, camera, and LiDAR objects and related mount settings. Typed production experiments declare Sensor SDK plugin IDs instead.
- `envConfig`: world bounds, obstacles, denied/favored sensor zones, water field, visibility, and time of day.
- `goalConfig`: waypoint list and tolerance.
- `controlConfig`: controller location, frequency, strategy, power budget, and guidance mode.

`schema.js` then builds an initial `simState` with boat state, belief state, goal state, environment state, sensor state, controls, and metrics. The `simulator` object owns the live models and logs.

At each simulation step, the core flow is:

1. Sample the environment around the boat, including water height, normal, current, and hull sample points.
2. Convert the local environment into water-relative velocity/current inputs for the dynamics core.
3. Update the boat belief used by guidance. In `absolute` mode the belief can copy true roll/pitch; in `relative` mode roll/pitch are hidden while heading remains usable.
4. Run the controller/skipper to choose waypoints, active sensors, desired heading, thrust, braking, and rudder/yaw command.
5. Convert guidance into a physical actuator command, apply twin-thruster lag/saturation, assemble force-space dynamics, and integrate the core rigid body state with RK4.
6. Update waypoint progress, mission status, metrics, logs, and sensor outputs.

The JavaScript coordinate convention is right-handed and Y-up:

- `pos.z` is north/forward.
- `pos.x` is east/lateral.
- `pos.y` is up/heave.
- `heading` is yaw around Y in radians.

CSV export remaps this to MSS-style columns as `north=pos.z`, `east=pos.x`, and `up=pos.y`. Internally, the core uses marine NED/body conventions: world `{N,E,D}`, body `{u,v,w,p,q,r}`, with yaw reported as `psi`.

## Dynamics Core

The new core is deliberately separate from rendering and the DOM:

- `core/rigidBodyState.js`: authoritative NED pose, quaternion, body velocity, and diagnostic acceleration state.
- `core/vehicleParameters.js`: physical parameter schema plus `fromGeometry(...)` bootstrap generation.
- `core/vehicles/otter.js`: Otter-style reference preset for validation.
- `core/vehicles/bcod_usv.js`: BCOD surface vessel bootstrap preset.
- `core/forces/`: actuator, Coriolis, hydrodynamic damping, current coupling, restoring placeholder, and wave excitation models.
- `core/dynamicsCore.js`: assembles wrenches and solves `M * nuDot = tau`.
- `core/integrator.js`: fixed-step RK4 and semi-implicit Euler.

The old `boatModel` name remains as a facade so existing renderers, sensors, metrics, and tests continue to call the same API. Under that facade, guidance acceleration is converted to thrust demand and rudder demand is converted to differential thruster force.

## Browser Surfaces

### Developer State Viewer

Open `threeSim.html` to run the default scenario from `scenarioPresets.js`.

Controls include:

- Start/Pause
- Step once
- Reset
- Run Until Done
- Export CSV
- Water visualization mode: `physics`, `presentation`, `height`, `velocity`, `normal`, or `hull`
- Runtime speed multiplier

The side panel shows time, time of day, boat state, belief state, angular state, water sample state, active sensors, energy metrics, and log count. It also renders live sensor feed cards for GPS, IMU, camera, and LiDAR.

### Guided Mission Demo

Open `demo.html` for the product-style flow. It walks through:

1. Vehicle and sensor suite configuration.
2. Obstacle and sensor-zone placement.
3. Weather, current, and wave setup.
4. Route creation.
5. Live mission run.
6. Results summary.

`demoRunner.js` converts the UI config into the same `scenarioConfig` classes used by the core simulator, so the guided demo and developer viewer share the same underlying physics and mission logic.

## Sensors

The legacy browser demo supports state-derived and renderer-derived compatibility feeds. These are distinct from the typed production plugin registry.

State-derived sensors:

- GPS emits position and velocity.
- IMU emits acceleration, angular velocity, and orientation.
- Water quality is implemented by the typed `water-quality` plugin; the stale legacy EXO2 selector is no longer advertised.

Renderer-derived sensors:

- Day/night cameras are rendered from a mounted `THREE.PerspectiveCamera`.
- LiDAR casts rays into the Three.js scene and returns ranges plus hit points.

`sensorWorldPose()` converts each sensor mount from boat-local frame to world frame. The renderer-derived provider uses the synchronized Three.js scene so cameras and LiDAR observe the visible water, obstacles, boat-relative pose, and active scene targets.

Denied and favored zones can influence sensor activation. Denied zones force configured sensors off while inside the polygon; favored zones are represented as sensor-preference regions.

## Optional Sensor Streaming

Sensor streaming is disabled unless requested. Enable it in either of these ways:

```text
http://localhost:8000/threeSim.html?streamSensors=1
http://localhost:8000/demo.html?streamSensors=1
```

or set local storage:

```js
localStorage.setItem("bcodStreamSensors", "1")
```

Streams are sent to local WebSocket receivers:

- Camera: `ws://127.0.0.1:8765`
- LiDAR: `ws://127.0.0.1:8766`
- Telemetry: `ws://127.0.0.1:8767`

Binary camera and LiDAR messages use a 24-byte little-endian header:

```text
uint32 frameId
float64 timestamp
uint32 a
uint32 b
uint32 c
```

For camera frames, `a=width`, `b=height`, `c=channels`, followed by raw RGBA bytes. For LiDAR, `a=pointCount`, `b=3`, `c=4`, followed by `Float32Array` XYZ points. Telemetry is JSON.

Example receivers live in `data recievers/`:

```bash
python3 "data recievers/cameraDataReciever.py"
python3 "data recievers/lidarDataReciever.py"
python3 "data recievers/sensorTelemetryReceiver.py"
```

Those scripts require Python packages such as `websockets`, `numpy`, and for visual receivers `opencv-python`.

## Validation And Comparison

Run the JavaScript tests with Node.js installed:

```bash
npm test
```

Individual checks:

```bash
npm run test:smoke
npm run test:physics
npm run test:validation
npm run test:preset-parity
npm run test:6dof
```

Headless performance reports:

```bash
npm run benchmark:plant -- --steps 5000 --vessels 10
npm run benchmark:physics -- --steps 1000 --vessels 10 --mode both
```

The official MSS source is pinned in `validation/mss-reference.json`. Run
`npm run test:mss-acceptance` only after frozen traces from that exact MSS
revision have been placed in `validation/mss-golden/`; the local Python model is
not treated as an independent oracle.

Regenerate those traces with GNU Octave from a checkout at the pinned commit:

```bash
MSS_ROOT=/absolute/path/to/MSS npm run generate:mss-golden
npm run test:mss-acceptance
npm run plot:mss
```

The generator verifies the checkout commit before running the official
`CRAFT/USV/models/otter.m`. Identical net surge and yaw wrenches are applied on
both sides, isolating planar plant fidelity from motor dynamics.
The plotting command writes track, velocity, heading, and error overlays plus
an acceptance-limit summary to `validation/mss-plots/`.

The validation tests check Otter parameter loading, Coriolis skew symmetry, deterministic straight-line motion, turning-circle response, current coupling, zig-zag boundedness, and preset parity between `scenarioPresets.js` and `mssReference.py`. The frozen Octave traces are the independent MSS oracle; the local Python utility is not.

The tests load `schema.js` directly, strip ESM `export` keywords, and evaluate the core classes in Node. They do not require the browser renderer.

The state viewer can export a CSV from `webSimRunner.js`. That export includes a header comment block plus time, north/east/up position, surge/sway/heave, heading, roll, pitch, yaw/roll/pitch rates, accelerations, speed, energy, and cost.

For MSS comparison:

```bash
python3 csvCompare.py --js path/to/js_export.csv --mss data/mss_baseline.csv
```

This produces:

- `comparison_report.png`
- `comparison_report.txt`

`mssReference.py` contains a simplified 3-DOF Fossen/Otter-style reference model retained for historical comparison. The primary JavaScript dynamics path is now `core/`, and `validation/coreValidationTest.js` is the local non-circular validation entry point.

Python comparison tooling uses packages such as `numpy`, `pandas`, `matplotlib`, and `scipy`.

## Particularities And Gotchas

- The folder name `data recievers` is misspelled and contains a space. Quote it in shell commands.
- `threeStateRenderer.js`, `threeSensorProvider.js`, and `demoRenderer.js` import Three.js from `vendor/three.module.js`.
- The water normal texture is loaded from `vendor/waternormals.jpg`.
- Many browser imports include cache-busting query strings like `?v=25`. These are manually maintained and are not a build system.
- This folder has a small `package.json` only to mark local files as ES modules and expose test scripts. The browser app is still plain ES modules plus assets.
- `schema.js` is intentionally large and central. It currently acts as domain schema, physics engine, controller, sensor model, logger, and water model.
- Time of day is computed from simulation seconds modulo 24 hours. `day`, `dawn`, `dusk`, and `night` are derived labels.
- The active boat visual is mostly procedural geometry; the `models/boat` GLTF asset exists but is not the primary rendered boat in the current code.
- `comparison_report.png` and `comparison_report.txt` appear to be generated artifacts from comparison runs.
- `__pycache__/` is generated Python cache output and is not source code.

## Recommended Development Workflow

1. Change the core scenario or physics in `schema.js` and `scenarioPresets.js`.
2. Run `node simulatorSmokeTest.js`.
3. Run `node physicsBehaviorTest.js`.
4. Start a local HTTP server and verify `threeSim.html`.
5. If the change affects guided configuration or visuals, verify `demo.html`.
6. Export a CSV from the state viewer when comparison against MSS or baseline data is needed.

For physics changes, pay special attention to:

- timestep consistency between different `simHz` values,
- bounded angular velocity in waves,
- roll/pitch restoring behavior,
- heading convention: heading `0` means forward motion along world `+z`,
- sensor mount transforms,
- waypoint completion using the boat footprint plus tolerance.

## Current Architecture In One Sentence

`schema.js` owns the simulation truth, `webSimRunner.js` and `demoRunner.js` turn that truth into browser workflows, the Three.js renderers visualize and generate camera/LiDAR observations from it, and the Python/CSV tools validate the run against external reference data.
