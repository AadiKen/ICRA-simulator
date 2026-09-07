# Stonefish Harness — Gate 0 Consolidated Report

Date: 2026-09-07  
Login host: `codenimbus-000-3.csl.illinois.edu`  
Runtime node: `codenimbus-002-1.csl.illinois.edu` (`gpu-l40-csl-cnice`)  
Stonefish source: `$HOME/stonefish-src`

## Outcome

**Gate 0 passes.** The missing dependencies were built in a user-owned
prefix, Stonefish 1.6.0 and all bundled tests built successfully, the library
installed locally, and `ConsoleTest` ran without a display or visible GPU.
No HoloOcean, Gazebo/VRX, bcod-sim, vehicle, Gym-wrapper, or RL code was
modified.

## Part A2 — local dependencies

Everything was installed below `$HOME/stonefish-deps` without `sudo`.

| Dependency | Version | Commit | Result |
|---|---:|---|---|
| GLM | 1.1.0 | `6f14f4792a0cde5d0cf2c910506724d61cb95834` | Installed |
| SDL2 | 2.33.0 | `b90ac95029d801c5abc59472ba8e2200dff31e1e` | Built and installed |
| FreeType | 2.13.2 | system package | Reused |

The earlier `pkg-config` result of `26.1.20` for FreeType is the package's
libtool-style value. CMake and `freetype.h` both identify the actual library as
FreeType 2.13.2.

GLM is header-only for Stonefish, although the current GLM install also emits
its small CMake library target. SDL2 was built shared-only in Release mode.
Tests and install-tests were disabled, as were video, X11, Wayland, OpenGL,
OpenGL ES, Vulkan, rendering, audio, joystick, haptics, sensors, and power
backends. Filesystem, threads, timers, dynamic loading, and CPU information
were retained. Configuration completed without another missing-library
cascade.

No source patch was required. The generated `SDL2Config.cmake` uses imported
targets and contains no malformed `-lSDL2 ` token. `sdl2.pc` does contain
ordinary trailing whitespace after `-lSDL2`, but Stonefish configured and
built successfully through the CMake package, so it was left unchanged.

## Part B — Stonefish build

Stonefish was cloned from `https://github.com/patrykcieslak/stonefish.git`:

- version: 1.6.0 (`git describe`: `v1.5-15-gb21eb8e`)
- commit: `b21eb8e194c570ff2f61e91aeffb38d73dc25f42`
- dependency prefix: `$HOME/stonefish-deps`
- install prefix: `$HOME/stonefish-install`

Two clean build trees were used because upstream makes `BUILD_TESTS=ON` and
installation mutually exclusive:

1. `$HOME/stonefish-build-install`: `BUILD_TESTS=OFF`,
   `EMBED_RESOURCES=ON`. This produced and installed `libStonefish.so`; embedded
   resources make the user-local install self-contained.
2. `$HOME/stonefish-build-tests`: `BUILD_TESTS=ON`,
   `EMBED_RESOURCES=OFF`. This produced `Stonefish_test` and all bundled test
   executables, with resource paths intentionally bound to the cloned source.

Both choices therefore serve their documented purpose rather than being
competing settings in one build.

Stonefish's top-level CMake unconditionally calls `find_package(OpenGL
REQUIRED)` and puts OpenGL in its link-library list, even for a console build.
OpenGL development libraries must consequently exist at configure/link time.
On this host the linker discarded unused GL dependencies: neither
`ConsoleTest` nor `libStonefish_test.so` has a GL/OpenGL/GLX/EGL `DT_NEEDED`
entry.

## Part C — headless bring-up

The bundled `Tests/ConsoleTest` ran on the preferred L40 node with:

- `DISPLAY` unset;
- `WAYLAND_DISPLAY` unset;
- `CUDA_VISIBLE_DEVICES` empty; and
- no graphics or CUDA context creation.

The executable advanced normally until an intentional five-second timeout
(`124`, as expected). It logged 2,035 steps, from simulation time 0.002 s to
4.070 s, in 5.008 s wall time. Thus approximately 2,000 logged physics steps
took five seconds including startup and console-I/O overhead. This is only a
bring-up sanity number; the example is configured for 500 steps/s and runs in
real-time mode.

SDL2 remains a runtime dependency (`libSDL2-2.0.so.0`) even in console mode,
principally for mutex/thread/console infrastructure. FreeType also remains a
runtime dependency because the monolithic Stonefish shared library includes
graphical code. OpenGL is a build-time requirement here but not a retained
runtime dependency for this console binary.

## Part D — source reconnaissance

Findings below are from the exact cloned commit, not paper claims.

### 1. Python/Gym bindings

They are **not present in this release**. The only Python file in the checkout
is `docs/conf.py`; there is no Python package, binding generator, pybind code,
Gym/Gymnasium environment, or Python-facing API. Consequently this release
provides no Python sensor reads, actuator setpoints, reset, or stepping. The
ICRA 2025 paper's direct Gym binding is not in this upstream tree.

The C++ API does expose the underlying operations needed by a future wrapper:
sensor history/last-sample access, actuator setters, scenario restart/reset,
and real-time or explicit fixed-step simulation. That is not a Python binding.

### 2. Existing surface vessel

A usable starting example **is bundled**: `Tests/FloatingTest` constructs a
150 kg floating `Boat` from `boat.obj`/`boat_gra.obj`, enables a flat ocean,
attaches an `Odometry` sensor, and drives one full `Thruster`. Its hull uses
`PhysicsMode::FLOATING`; its propeller geometry uses
`PhysicsMode::SUBMERGED`. This is an actuated surface-vessel example, though it
is a simple single-thruster demonstration rather than the required
differential twin-thrust Vehicle A. No separately named ASV/USV scenario was
found.

### 3. Sensor semantics and oracle classification

All scalar-sensor channel noise is applied centrally in
`ScalarSensor::AddSampleToHistory`: configured per-channel Gaussian noise is
added before range clipping. Source behavior is:

| Sensor | Source signal and modeled error | Classification |
|---|---|---|
| GPS | Sensor-frame world position; independent Gaussian error is added to world/NED X and Y before geodetic conversion. | Sensor-derived/noisy position, not a clean oracle when noise is configured. |
| Compass | Yaw extracted from the sensor-frame world orientation; heading Gaussian noise is a channel setting. | Sensor-derived/noisy heading. |
| Accelerometer | Link linear and angular kinematics at the sensor offset, transformed into the sensor frame, minus gravity so it reads specific force; per-axis Gaussian noise and range limits are supported. | Sensor-derived/noisy specific force. |
| Gyroscope | Link angular velocity transformed into the sensor frame; fixed bias is added explicitly, then per-axis Gaussian channel noise/range clipping applies. | Sensor-derived/noisy angular rate. |
| IMU | Ground-truth orientation, angular velocity, and offset-point acceleration/specific force transformed into the sensor frame; supports angle/rate/acceleration Gaussian noise plus accumulated yaw drift. Reset clears yaw drift. | Composite simulated sensor, not a clean oracle when its error model is configured. |
| Odometry | Reads world position and quaternion directly from the attached body's sensor transform, plus body-frame linear and angular velocities. Optional Gaussian position/velocity/rate noise and a noisy quaternion angle can be enabled. | Navigation ground truth by provenance; optional corruption does not make it an independently derived physical sensor. |
| INS | Integrates internally measured body acceleration and angular rate; can correct velocity/altitude with DVL, horizontal position with valid GPS, and depth with pressure. It outputs N/E/depth, altitude, lat/lon, body velocity, attitude, body rates, and body acceleration. | A navigation solution/fusion product, not a raw sensor and not direct pose ground truth. |

The generic sensor RNG is a process-global `std::mt19937` initialized from
`std::random_device`; no public seed setter was found. Noise-enabled runs are
therefore not reproducible through an exposed seed knob.

### 4. GPS above-water behavior

`GPS::InternalUpdate` tests the **GPS sensor origin**, not the hull, with
`Ocean::IsInsideFluid`. For a flat surface, `Ocean::GetDepth` is the point's
world Z, and `IsInsideFluid` is true at depth `>= 0`; the water surface itself
therefore counts as submerged.

When submerged, GPS still appends a fresh, timestamped sample containing:

- latitude = `BT_LARGE_FLOAT`;
- longitude = `BT_LARGE_FLOAT`;
- North = `0`; and
- East = `0`.

There is no explicit boolean `fix_valid`. INS recognizes failure indirectly by
requiring latitude `<= 90` and longitude `<= 180`, so it ignores this sentinel
sample and continues its inertial prediction. A consumer reading GPS directly
must reproduce that validity test; treating the zero N/E values as a fix would
create a false jump to the origin.

### 5. Thrusters on a surface vessel

The full `Thruster` has the same water-presence constraint as
`SimpleThruster`. Each tests the **actuator/thruster-frame origin** using
`Ocean::IsInsideFluid`; if it is not submerged (or no ocean exists), reported
thrust and torque are set to zero and no force is applied. The full model's
rotor state continues updating before that water test.

The hull's state does not gate thrust. A floating hull with propeller origins
below the instantaneous water surface receives thrust normally. This is
exactly how `FloatingTest` is assembled: floating hull, submerged propeller
geometry, full thruster attached at local Z `+0.3`. With NED positive-down Z,
that actuator origin is below the nominal Z=0 water surface.

### 6. Frame convention

The simulation world is right-handed **NED**:

- world `+X` = north;
- world `+Y` = east;
- world `+Z` = down/depth; and
- positive gravity is installed as `(0, 0, g)`.

Vehicle/body frames use the aligned marine convention **FRD**:

- body `+X` = forward/surge (thrusters apply local `(thrust, 0, 0)`);
- body `+Y` = starboard/sway (the test vehicle places its named starboard
  components at positive Y and port components at negative Y); and
- body `+Z` = down/heave (also required by right-handed X-forward,
  Y-starboard, and demonstrated by the surface boat's submerged propeller at
  positive Z).

Sensor-frame vectors are produced by inverse-transforming world kinematics
into the sensor/body frame. GPS reports world N/E. INS integrates body-frame
velocity into NED, emits position/geodetic fields in NED/global coordinates,
and velocity/rates/acceleration in its configured output/body frame.

### 7. Determinism mechanisms (not tested)

Available controls and relevant defaults are:

- `SimulationManager(stepsPerSecond, Solver, CollisionFilter)` sets the base
  physics quantum to `1 / stepsPerSecond`; `StepSimulation` passes this as
  Bullet's fixed substep. `SimulationApp::Run(..., timeStep > 0)` selects an
  explicit fixed-step caller path, whereas `timeStep == 0` follows wall time.
- Solver choices are sequential impulse (`SI`, default), Dantzig, PGS, Lemke,
  and the declared NNCG value. In the implementation's MLCP switch, NNCG is
  not handled separately and falls through to the default Dantzig solver.
- Bullet uses 100 constraint iterations, warmstarting factor 1, SIMD, two
  friction directions, split impulse, no randomized constraint order, and
  continuous collision detection disabled.
- Scenario-configurable solver values include ERP, stop ERP, contact ERP,
  global damping/friction, linear/angular sleeping thresholds, and the fluid
  dynamics prescaler. Initial-condition solving separately exposes timestep,
  iteration/time caps, gravity use, and linear/angular tolerances through C++.
- Physics-fluid work can run through a thread pool. The default maximum is the
  detected physical-core count; C++ `setMaxPhysicsThreads(n)` and scenario XML
  `<multithreading max_physics_threads="...">` can force it to one or another
  fixed count. Hydrodynamic/aerodynamic tasks are enqueued per object then
  joined, so single-thread operation is the conservative reproducibility knob.
- Sensor noise is nondeterministically seeded from `std::random_device`, with
  no exposed reseeding API. Disable noise or add an explicit seed mechanism in
  a later gate before claiming deterministic noisy observations.

No determinism comparison was run, as required.

## Gate 0 disposition

Stonefish console mode is feasible on CodeNimbus. Gate A can proceed after
this report is reviewed, using `FloatingTest` as the closest bundled starting
point and preserving the key constraints identified here: the GPS sentinel,
per-actuator submergence, fixed-step execution, single-thread option, and lack
of an upstream Python/Gym binding.
