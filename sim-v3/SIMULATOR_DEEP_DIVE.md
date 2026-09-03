# Legacy Browser Simulator Deep Dive

This document explains the root-level legacy `threeSim.html` and `demo.html` compatibility subsystem: how a scenario becomes runtime state, how each fixed timestep advances the mission, how guidance becomes physical thrust, how water and waves feed its planar dynamics, how sensors are produced, and how those browser surfaces consume the result. It does not describe the repository's primary typed research architecture; see [`README.md`](README.md) for the authoritative current architecture, serving instructions, three vehicle configurations, and `planar3`/`coupled6` modes.

This compatibility subsystem is a browser-first autonomous surface-vessel demo. It is intentionally focused on one legacy vessel, waypoint-following mission logic, simplified water/environment behavior, a 3-DOF marine dynamics model, sensor approximations, metrics, logging, and browser visualization.

## Quick Start

Run the simulator behind a local HTTP server. Directly opening the HTML files from disk can break ES module loading, WebGL assets, and browser security rules.

```bash
cd sim-v3
python3 -m http.server 8001
```

Open one of these pages:

- `http://localhost:8001/threeSim.html`: developer-oriented state viewer, physics debugger, sensor feed viewer, CSV exporter.
- `http://localhost:8001/demo.html`: guided mission demo with configuration steps and richer presentation.

Run the local checks with:

```bash
npm test
```

Individual scripts:

```bash
npm run test:smoke
npm run test:physics
npm run test:validation
npm run test:preset-parity
```

## Mental Model

At a high level, the simulator is a fixed-step mission loop:

```text
scenario config
  -> initial runtime state
  -> repeated simulator.step()
      -> sample environment
      -> collect sensor observations
      -> update control command when due
      -> build boat belief
      -> compute guidance
      -> convert guidance to actuator forces
      -> integrate 3-DOF vessel dynamics
      -> update mission/failure state
      -> update metrics and logs
      -> render/browser UI reads the resulting state
```

The important separation is:

- `schema.js` owns most domain objects, state objects, mission logic, control, sensors, environment, metrics, logging, and the bridge into physics.
- `core/` owns the pure physics core: rigid-body state, frames, vehicle parameters, force models, and integrators.
- Browser runners own UI timing, rendering, panels, buttons, sensor visualization, and CSV export.

## Major Files

| Path | Role |
| --- | --- |
| `schema.js` | Central simulator schema and runtime model. Defines configs, runtime state, controller, skipper/guidance, sensors, environment sampling, metrics, logging, and the `simulator` class. |
| `core/dynamicsCore.js` | Sums force model wrenches and solves the 3-DOF acceleration equation. |
| `core/integrator.js` | RK4 and semi-implicit Euler stepping for the dynamics state vector. |
| `core/vehicleParameters.js` | Builds mass, added-mass, damping, restoring, and actuator parameter sets. |
| `core/forces/` | Actuator, damping, Coriolis/added-mass, current coupling, restoring placeholder, and wave excitation force models. |
| `adapters/legacyGuidanceAdapter.js` | Converts legacy guidance commands into actuator force demands. |
| `scenarioPresets.js` | Default scenario for `threeSim.html`. |
| `webSimRunner.js` | Runs `threeSim.html`: reset/start/pause/step/export/debug/sensor panels. |
| `threeStateRenderer.js` | Three.js state viewer renderer and water/hull debug modes. |
| `threeSensorProvider.js` | Renderer-backed camera and LiDAR observation provider. |
| `demoRunner.js` | Guided demo UI controller and scenario builder. |
| `demoRenderer.js` | Guided demo renderer. |
| `sensorStreamPublisher.js` | Optional WebSocket publishing for camera, LiDAR, and telemetry feeds. |
| `validation/` | Physics validation maneuvers and parameter parity checks. |
| `physicsBehaviorTest.js` | Behavior tests for stability, damping, guidance, waves, sensors, and zones. |
| `simulatorSmokeTest.js` | Headless smoke test for stepping, logs, command state, environment, and metrics. |

## Coordinate Systems

The browser/app state uses a Three.js-style right-handed, Y-up frame:

```text
world x = east / lateral
world y = up
world z = north / forward
heading = yaw around world y
```

The physics core uses marine-style 3-DOF NED/body variables:

```text
position: N, E, D
attitude: yaw
velocity: u, v, r
```

Where:

- `N` is north.
- `E` is east.
- `D` is down.
- `u` is surge velocity in the boat body frame.
- `v` is sway velocity in the boat body frame.
- `r` is yaw rate.

The bridge between the two worlds lives in `boatModel.ensureCoreState()` and `boatModel.syncBoatFromCore()` in `schema.js`. In practice:

```text
app pos.z <-> core position.N
app pos.x <-> core position.E
app pos.y <-> -core position.D
app velocity.z/x <-> body u/v after heading transform
app angularVel.y <-> core r
```

Heading convention matters: heading `0` means the boat points and thrusts along app/world `+z`.

## Scenario Configuration

A run starts from a `scenarioConfig` object. It bundles six config groups:

- `simConfig`: fixed step rate, duration, seed, and ground-truth allowance.
- `boatConfig`: speed/acceleration limits, mass, dimensions, power model, damping knobs, hydrodynamic estimates, stability constants, and start pose.
- `sensorConfig`: installed sensor objects.
- `envConfig`: map bounds, obstacles, sensor zones, water field, visibility, and starting time of day.
- `goalConfig`: waypoint list and waypoint tolerance.
- `controlConfig`: controller mode, control rate, strategy, timeout, and guidance mode.

`scenarioPresets.js` creates the default developer-viewer scenario. The guided demo builds a similar `scenarioConfig` dynamically from UI state in `demoRunner.js`.

## Runtime State

`createInitialSimState(scenarioC)` turns static config into mutable runtime state:

- It computes the simulation start time from the configured time of day.
- It creates a runtime `waterFieldModel` so the boat can start at the water surface height.
- It points the boat toward the first waypoint if one exists.
- It creates the true `boatState`.
- It creates the initial `boatBelief`.
- It creates goal, sensor, environment, control, and metric state containers.
- It stores all of those in one `simState`.

The most important runtime objects are:

- `simState`: current truth and mission state.
- `boatState`: position, velocity, acceleration, orientation, heading, angular rates, water diagnostics, rigid-body bridge state, last actuator command, and last dynamics wrench.
- `goalState`: waypoints, active waypoint index, completion/failure flags, waypoint hit history.
- `sensorsState`: installed sensors, active set, last outputs.
- `envState`: bounds, obstacles, zones, water config, visibility, time.
- `metricState`: energy/cost accumulators.

Logs are not stored in `simState`; they are owned by `logger` and exposed as `sim.logs`.

## The Simulator Object

The `simulator` class is the main runtime owner. Its constructor creates:

- `boatModel`
- `goalModel`
- `controlModel`
- `sensorModel`
- `skipperModel`
- `envModel`
- `metricModel`
- `logger`

The public stepping API is:

```js
sim.step();
sim.runSteps(stepCount);
sim.runUntilDone(maxSteps);
sim.simStatus();
sim.getSensorFeeds();
```

Both browser surfaces ultimately drive the same `simulator` object.

## One Timestep In Detail

`simulator.step()` is the central loop.

### 1. Stop Check

The step returns immediately if the run should stop. Stop reasons include:

- goal completed
- goal failed
- duration elapsed

Goal failure currently comes from collision or out-of-bounds checks.

### 2. Environment Sampling

`envModel.getLocalSample(state)` samples the world around the current boat state.

It checks obstacles by comparing boat position against obstacle radius plus boat hitbox. It also builds a grid of hull sample offsets:

```text
3 beam stations x 5 length stations = 15 hull sample points
```

Each local hull offset is rotated by the boat orientation, moved into world space, and passed to `waterFieldModel.sampleAt(...)`.

The resulting `envSample` contains:

- center water velocity
- center water acceleration
- center water height
- visibility
- obstacle hits
- time of day
- hull water samples
- center water sample

This sample is stored on `state.localEnv` and also passed into the boat model.

### 3. Sensor Observation Collection

`sensorModel.getObservations(...)` is called every sim step, but each sensor has its own sampling cadence.

The model:

- starts from the previous observation set
- asks an optional renderer-backed provider to sync the scene
- skips inactive sensors
- skips sensors that are not due based on `hz`
- emits new observations for sensors that are due
- preserves previous outputs for sensors that did not sample this step

State-derived sensors are produced directly:

- GPS returns pose, position, and velocity.
- IMU returns pose, acceleration, angular velocity, angular acceleration, and orientation.

Renderer-derived sensors are delegated when `ThreeSensorProvider` is installed:

- day camera
- night camera
- LiDAR

Without a provider, those return placeholder observations explaining that the Three.js layer is required.

### 4. Control Command Update

The controller does not necessarily run every sim step. It updates when:

```text
state.steps * sim.stepTime >= controlInvocationCount * controlModel.stepTime
```

The built-in strategies are simple:

- `heuristic`: returns the next one or two waypoints and activates all configured sensors.
- `local`: returns only the next waypoint and no active sensors.
- anything else: returns no waypoints and no active sensors.

After a command is generated, denied zones are applied. If the boat is inside a denied zone, sensors listed by that zone are removed from the active sensor set.

### 5. Boat Belief Update

The skipper uses a belief state rather than always using full truth.

In `absolute` guidance mode, `boatBelief` is a copy of the true boat state.

In `relative` guidance mode:

- vertical position is set to water height
- roll and pitch are hidden
- heading remains available
- roll/pitch angular rates and angular accelerations are hidden

This keeps the guidance problem closer to horizontal navigation while preserving a usable heading estimate.

### 6. Skipper Guidance

`skipperModel.getGuidance(...)` converts the current command and boat belief into a low-level guidance object:

```js
new guidanceObj(w, a, target, desiredHeading, desiredSpeed)
```

Where:

- `w` is normalized rudder/yaw command.
- `a` is requested forward acceleration.
- `target` is the current lookahead target.
- `desiredHeading` is the heading toward the target.
- `desiredSpeed` is the planned speed.

The skipper:

- chooses command points from the active command or mission waypoints
- computes distance to the next waypoint
- computes desired heading and heading error
- estimates closing speed and forward speed
- computes stopping distance
- lowers desired speed when close to a waypoint or badly misaligned
- computes acceleration or braking demand
- computes yaw command from heading error and yaw-rate error

This is a guidance layer, not the physics layer. It asks for acceleration and yaw behavior; the boat model turns that request into forces.

### 7. Environment State Applied To Boat Facade

`boatModel.updatePosEnv(...)` stores the latest local environment sample and records water diagnostics on `boatState`.

It does not integrate the boat directly. The current physical integration happens in the next call, `updatePosGuidance(...)`, through the dynamics core.

### 8. Guidance Converted To Actuators

`boatModel.updatePosGuidance(...)` is the bridge from mission guidance to physical dynamics.

It first calls `legacyGuidanceToActuatorCommand(...)`, which converts:

```text
guidance.a -> surgeForce = acceleration * mass
guidance.w -> differentialForce based on yaw authority and beam
```

Then `ActuatorModel.commandWrench(...)` applies twin-thruster behavior:

- clamp total demand
- clamp differential demand
- split into port and starboard thrust targets
- apply first-order motor lag with `motorTimeConstant`
- return `[surgeForce, 0, yawMoment]`

That applied wrench is attached to the command so the dynamics core uses the lagged thrust rather than recomputing instantaneous thrust inside RK4 substeps.

### 9. 3-DOF Dynamics Integration

The physics core state vector is:

```text
[N, E, yaw, u, v, r]
```

The dynamics core computes:

```text
etaDot = body velocity transformed into world N/E velocity
nuDot = inverse(massMatrix) * totalWrench
```

And returns:

```text
[Ndot, Edot, yawDot, udot, vdot, rdot]
```

The default integrator is RK4. Per step, RK4 evaluates the derivative four times and blends the result:

```text
y_next = y0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

The available semi-implicit Euler integrator exists, but the configured path uses RK4.

### 10. Force Stack

The core force stack is additive. Each model returns a body-frame wrench:

```text
[surge force, sway force, yaw moment]
```

The configured stack is:

```js
[
  ActuatorModel,
  AddedMassCoriolis,
  HydrodynamicDamping,
  RestoringForces,
  WaveExcitation
]
```

The summed wrench is then multiplied by the inverse mass matrix.

#### Mass And Added Mass

`VehicleParameters.buildMassMatrix()` creates:

```text
M = rigidBodyMass + addedMass
```

Rigid body mass includes vessel mass, yaw inertia, and optional CG offset coupling. Added mass increases apparent inertia because the hull accelerates surrounding water with it.

#### Current Coupling

The environment current is converted into NED, then into the boat body frame. The core computes relative velocity:

```text
relative u = boat u - current u
relative v = boat v - current v
relative r = boat r
```

Damping and added-mass terms use relative velocity, so current affects drag and track without directly teleporting the boat.

#### Hydrodynamic Damping

Damping is linear plus quadratic:

```text
surge damping = -(Xu*u + Xuu*abs(u)*u)
sway damping  = -(Yv*v + Yvv*abs(v)*v)
yaw damping   = -(Nr*r + Nrr*abs(r)*r)
```

This bounds speed under thrust and damps yaw rate.

#### Coriolis And Added-Mass Coupling

The Coriolis model builds rigid-body and added-mass Coriolis matrices, verifies skew symmetry, multiplies by relative velocity, and negates the result into a wrench.

This is what creates coupling between forward motion, lateral motion, and yaw rate.

#### Restoring Forces

`RestoringForces` currently returns zero in the 3-DOF core. This is reasonable for the current horizontal-only model because heave, roll, and pitch are not core states.

#### Wave Excitation

`WaveExcitation` looks at hull water samples. For submerged samples, it estimates a side force from:

- water density
- gravity
- approximate panel area
- sample depth
- water surface normal

It accumulates sway force and yaw moment. This is a lightweight wave-force approximation, not a full seakeeping or CFD model.

### 11. Presentation Seakeeping

After the 3-DOF core advances, `applyPresentationSeakeeping(...)` updates visual pitch and roll.

It averages water normals across hull samples, creates pitch/roll targets from the wave slopes, then applies spring-damper behavior:

```text
angular acceleration = (target - currentAngle) * stiffness - angularRate * damping
```

This makes the boat visually rock with the water. It is intentionally separate from the core 3-DOF physics. In other words:

- yaw affects navigation physics
- surge/sway affect navigation physics
- roll/pitch mostly affect presentation and sensor pose

### 12. Core State Synced Back To Browser State

`syncBoatFromCore(...)` copies the integrated physics state back into `boatState`:

- core `E` becomes app `pos.x`
- core `N` becomes app `pos.z`
- body velocity becomes world velocity
- core acceleration becomes world acceleration
- core yaw becomes `orientation.y` and `heading`
- core yaw rate becomes `angularVel.y`
- core yaw acceleration becomes `angularAcceleration.y`
- last actuator command and last dynamics wrench are retained for debug/exports

The browser renderers, panels, sensors, metrics, and logs read this app-facing state.

### 13. Mission Progress

`goalModel.updateMissionProgress(...)` checks whether the boat has reached the active waypoint.

It uses two reach tests:

- simple horizontal distance less than tolerance
- an ellipse around the boat footprint expanded by tolerance

When a waypoint is reached:

- a `waypointHitRecord` is appended
- `waypointIdx` increments
- if all waypoints are reached, `completed` becomes true

### 14. Failure State

The simulator marks the goal failed if:

- any sampled obstacle hit has `collision`
- the boat leaves map bounds

Collision is coarse. It is not a physical contact solver; it is mission/failure logic.

### 15. Metrics

`metricModel.captureMetrics(...)` updates energy/cost values.

Sensor cost is:

```text
sum(active sensor cost * dt)
```

Movement cost is:

```text
(basePowerDraw + movementPowerFactor * speed^2) * dt
```

The metric state tracks:

- last sensor cost
- last movement cost
- last total cost
- last speed
- cumulative sensor cost
- cumulative movement cost
- total energy

### 16. Logging

The logger records:

- true boat states
- boat belief states
- metrics
- active sensor sets
- placeholder arrays for uncertainty samples, predicted paths, and mission events

Snapshots copy vector values so later mutation does not rewrite old log entries.

### 17. Time Update

At the end of the step:

- simulation time increments by fixed `stepTime`
- time-of-day label is recomputed
- tick and step counters increment
- final stop state/reason is refreshed

## Water Field

The water model is a height field with superposed waves plus constant current.

`waterFieldModel.sampleAt(pos, time)` returns a `waterSample` containing:

- sample position
- surface height
- velocity
- acceleration
- normal
- depth/submerged status through `waterSample`

The water height is the sum of all wave heights:

```text
height(x, z, t) = sum(wave.heightAt(x, z, t))
```

The water normal comes from summed wave gradients:

```text
normal = normalize([-dh/dx, 1, -dh/dz])
```

Velocity is the sum of wave velocities plus configured current. Acceleration is from waves only.

## Sensors

Sensors are mounted relative to the boat. `sensorWorldPose(...)` converts each mount to world position/orientation using the current boat pose. The returned pose includes:

- position
- orientation
- forward vector
- right vector
- up vector

### GPS

GPS is state-derived. It returns:

- time
- sensor id/name/type
- world pose
- boat position
- boat velocity

### IMU

IMU is state-derived. It returns:

- time
- world pose
- acceleration
- angular velocity
- angular acceleration
- orientation

### Camera

Camera observations require `ThreeSensorProvider`.

The provider:

- syncs the Three.js scene to current simulation state
- creates a mounted `THREE.PerspectiveCamera`
- renders into a WebGL render target
- reads RGBA pixels
- converts pixels to a data URL for UI display
- keeps raw RGBA bytes for publishing/export-like use

### LiDAR

LiDAR observations also require `ThreeSensorProvider`.

The provider:

- syncs the Three.js scene
- computes a ray grid from horizontal FOV, vertical FOV, angular resolution, and max range
- rotates each local ray into world space through the sensor pose
- raycasts against scene objects flagged as LiDAR targets
- records ranges, points, hit count, and min range

Water hits are included only if the sensor has `includeWater`.

### Sensor Feed Formatting

`sim.getSensorFeeds()` returns UI-friendly feed cards. Each feed includes:

- id/name/type
- active status
- timestamp
- display config
- pose
- status: inactive, stale, placeholder, or live
- display type: placeholder, position, motion, image, pointCloud, or raw
- formatted summary

## Optional Sensor Streaming

`sensorStreamPublisher.js` can publish live sensor outputs to local WebSocket receivers.

Enable streaming with:

```text
?streamSensors=1
```

or:

```js
localStorage.setItem("bcodStreamSensors", "1")
```

Default endpoints:

- camera: `ws://127.0.0.1:8765`
- LiDAR: `ws://127.0.0.1:8766`
- telemetry: `ws://127.0.0.1:8767`

Receiver examples are in `data recievers/`. The directory name is misspelled and contains a space, so quote it in shell commands.

## Browser Runtime Surfaces

### `threeSim.html`

`webSimRunner.js` runs the developer state viewer.

It:

- creates the default scenario from `scenarioPresets.js`
- constructs `new simulator(scenario)`
- constructs `ThreeStateRenderer`
- installs `ThreeSensorProvider`
- optionally creates `SensorStreamPublisher`
- starts a `requestAnimationFrame` loop

The render loop uses an accumulator:

```text
real frame delta * speed multiplier -> accumulator
while accumulator >= sim.stepTime:
    sim.step()
    accumulator -= sim.stepTime
```

This keeps simulation truth fixed-step even though rendering is variable-rate.

The state viewer supports:

- start/pause
- step once
- reset
- run until done
- speed multiplier
- water mode switching
- debug state panel
- live sensor feed panel
- CSV export

### `demo.html`

`demoRunner.js` runs the guided demo.

It walks through:

1. Vehicle setup.
2. Obstacle and operating area setup.
3. Weather/water setup.
4. Route setup.
5. Mission run.
6. Results.

It ultimately builds the same `scenarioConfig` classes and runs the same `simulator` engine. The difference is UI workflow and presentation, not the physics core.

## CSV Export

The state viewer can export logs from `webSimRunner.js`.

The export maps app axes into MSS-style columns:

```text
north = pos.z
east  = pos.x
up    = pos.y
```

Rows include:

- time
- north/east/up
- surge/sway/heave
- heading
- roll/pitch
- yaw/roll/pitch rates
- total accelerations
- yaw acceleration
- speed
- total energy
- last total cost

The export begins with comment lines summarizing steps, stop reason, final position, sim rate, and axis convention.

## Validation And Test Coverage

The local tests cover behavior rather than proving full physical fidelity.

`simulatorSmokeTest.js` checks that the headless simulator can step, log, update commands, sample local environment, and produce metrics.

`physicsBehaviorTest.js` checks:

- still water/no input stability
- yaw damping
- roll/pitch presentation recovery
- forward thrust and heading convention
- guidance acceleration direction
- waypoint guidance behavior
- braking behavior
- sensor pose behavior
- relative/absolute guidance belief behavior
- denied-zone sensor behavior

`validation/coreValidationTest.js` checks:

- Coriolis matrix skew symmetry
- deterministic straight-line maneuver
- turning-circle response
- current coupling effect
- zig-zag boundedness

`validation/presetParityTest.js` checks parity between preset values and reference expectations.

## What This Simulator Is Good At

This simulator is strongest for:

- browser-based mission demos
- visualizing waypoint-following behavior
- showing effects of waves, current, obstacles, and sensor activation
- debugging a simplified USV control loop
- producing repeatable local runs
- exporting logs for comparison
- approximating camera/LiDAR feeds from the visible Three.js scene

## Current Limits of the Legacy Subsystem

The legacy subsystem is not a full robotics simulator. These statements do not describe the typed `coupled6` production path.

Important limitations:

- The core dynamics are 3-DOF, not full 6-DOF.
- Roll and pitch are presentation/seakeeping behavior, not fully coupled physical states.
- Collision is a mission failure check, not contact dynamics.
- Obstacles do not exert physical impulses or friction.
- The wave model is a height field plus simple excitation, not CFD.
- The actuator model is twin-thruster differential drive, not detailed propulsor/rudder modeling.
- Sensor models are simplified; camera/LiDAR depend on the renderer scene, not a robotics middleware sensor stack.
- There is no ROS, URDF, SDF, TF tree, plugin system, or multi-robot world model.
- `schema.js` is broad and central; it mixes schema, runtime logic, models, and state helpers.

These are acceptable tradeoffs for a lightweight browser demo, but they matter if comparing this system to Gazebo or to a high-fidelity naval simulator.

## Development Workflow

For changes to mission logic or physics:

1. Edit the relevant source.
2. Run `npm test`.
3. Start `python3 -m http.server 8001`.
4. Verify `threeSim.html`.
5. If UI/presentation is affected, verify `demo.html`.
6. Export CSV if comparison against baselines is needed.

For physics changes, pay close attention to:

- `simHz` sensitivity
- heading `0` along world `+z`
- current-relative damping
- bounded yaw rate
- bounded forward speed under constant thrust
- waypoint arrival behavior
- sensor mount transforms
- roll/pitch presentation not feeding back into the 3-DOF core in unintended ways

## End-To-End Data Ownership Summary

The shortest useful ownership map is:

```text
scenarioPresets.js / demoRunner.js
  create scenarioConfig

schema.js:createInitialSimState()
  creates simState and boatState

schema.js:simulator
  owns all runtime models and logs

schema.js:envModel
  samples obstacles and water

schema.js:sensorModel
  samples state/provider sensors

schema.js:controlModel
  emits waypoint/sensor commands

schema.js:skipperModel
  converts command + belief into guidance

schema.js:boatModel
  converts guidance into actuator wrench
  bridges app state <-> core rigid-body state

core/dynamicsCore.js
  sums forces and integrates 3-DOF dynamics

schema.js:goalModel / metricModel / logger
  update mission, cost, and logs

webSimRunner.js / demoRunner.js
  render and expose the resulting state
```
