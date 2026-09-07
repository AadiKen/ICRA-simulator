# Stonefish Harness — Gate A Report

> Gate D follow-up: the propeller center was subsequently moved from NED
> `z=0.30 m` to `z=0.40 m` after native fluid-boundary gating was observed to
> interrupt startup thrust. See `GATED_VALIDATION_REPORT.md`; Gate A/B
> regressions pass with the retained placement.

Date: 2026-09-07  
Stonefish: 1.6.0, commit `b21eb8e194c570ff2f61e91aeffb38d73dc25f42`  
Verification node: `codenimbus-002-1.csl.illinois.edu` (`gpu-l40-csl-cnice`)

## Outcome

**Gate A passes.** A headless C++ Vehicle A application, process-isolated
Python bridge, and explicit sensor RNG seed hook were implemented. Reset,
fixed stepping, normalized twin-thruster commands, sensor reads, differential
response, and submerged-GPS rejection passed end to end.

No 15-field contract, reward, controller, training code, `Odometry`, or `INS`
was added. Existing HoloOcean, Gazebo/VRX, and bcod-sim paths were not changed.

## Part 1 — C++ application and Vehicle A

The application is in `stonefish/src` and links against the locally installed
Stonefish library. It subclasses `ConsoleSimulationApp`, reads commands only
while autostepping is disabled, and invokes Stonefish's explicit fixed-step
path.

| Parameter | Implementation |
|---|---|
| Physics rate | 500 steps/s |
| Fixed timestep | 0.002 s |
| Physics threads | 1 |
| Mass | 55 kg |
| Inertia | `(15, 20, 30)` kg·m²; yaw is the specified 30 kg·m² |
| Hull | `FloatingTest` boat geometry, `PhysicsMode::FLOATING` |
| Actuators | Port and starboard full `Thruster` instances |
| Lateral offsets | port Y = −0.395 m; starboard Y = +0.395 m |
| Propeller depth | local Z = +0.30 m in NED/FRD |
| Rotor dynamics | `FirstOrder(0.25)` |
| Commands | normalized and clamped to `[−1, 1]` |
| Thrust conversion | clamped interpolation `−95 → −95 N`, `0 → 0 N`, `95 → 95 N` |

Only yaw inertia was specified. Roll and pitch inertias were set to positive,
triangle-valid 15 and 20 kg·m² so Stonefish could receive a complete inertia
vector; they are not claimed as bcod-sim calibration values.

Stonefish's `Thruster::maxSetpoint` limits rotor speed, not Newtons. To express
the required force limit exactly, the normalized command maps to a ±95 rotor
setpoint and a saturated linear interpolation model maps that state one-to-one
to ±95 N. First-order filtering occurs before conversion. The starboard
left-handed propeller uses Stonefish's `invertedSetpoint` flag so equal user
commands have equal surge-force sign despite Stonefish's handedness flip.

The analytic bcod-sim surge damping law was not copied. Stonefish retains its
geometry-derived hydrodynamics as required.

## Part 2 — sensor and oracle isolation

The vehicle attaches exactly three sensors:

- `GPS`, at local Z = −0.50 m by default, clearly above the flat surface;
- `IMU`, at the hull origin; and
- `Compass`, at the hull origin.

`Odometry`, `INS`, standalone `Accelerometer`, and standalone `Gyroscope` are
not constructed. The IMU supplies attitude, body angular rate, and body
specific force, so separate inertial sensors were unnecessary.

The bridge observation has exactly four keys: `gps`, `fix_valid`, `imu`, and
`compass`. Actuator thrust is available only in a separate `diagnostics`
object for bring-up checks. Neither oracle sensor has a bridge lookup or
protocol operation.

GPS validity follows upstream INS behavior plus a sample-existence check:
latitude must be at most 90 degrees and longitude at most 180 degrees. A
submerged sensor's large-float sentinel therefore maps to `fix_valid=false`;
the accompanying zero North/East values remain visible but are never declared
a valid fix.

## Part 3 — Python interop

A line-oriented subprocess protocol was chosen instead of pybind11. Each
Python `StonefishBridge` owns one simulator process, which aligns naturally
with Stonefish's multi-instance training model, isolates native crashes and
global RNG state, avoids a new pybind11 dependency, and keeps stdout framing
simple. Stonefish informational lines are ignored until a JSON response line
is received.

Protocol operations are:

- `RESET <uint32-seed> [gps-z-ned]`: stop if needed, seed sensor RNG, rebuild
  the scenario, and start in manual-step mode;
- `STEP <port> <starboard> <positive-count>`: clamp commands to `[−1, 1]`
  and advance exactly `count` fixed 0.002 s steps; and
- `QUIT`: stop and exit cleanly.

Responses contain operation status, executed step count, simulation time, the
sensor-only observation, and actuator diagnostics. The optional GPS Z input is
an intentionally narrow verification hook for the documented mounting trap,
not a general scenario-authoring interface.

## Part 4 — sensor RNG seed patch

The only upstream Stonefish source change is stored verbatim in
`patches/0001-sensor-rng-seed-hook.patch`. It adds one public static method:

```diff
diff --git a/Library/include/sensors/Sensor.h b/Library/include/sensors/Sensor.h
@@
 class Sensor
 {
 public:
+    //! Set the process-global sensor noise generator seed.
+    static void SetRandomSeed(uint32_t seed);
diff --git a/Library/src/sensors/Sensor.cpp b/Library/src/sensors/Sensor.cpp
@@
 std::random_device Sensor::randomDevice;
 std::mt19937 Sensor::randomGenerator(randomDevice());
+
+void Sensor::SetRandomSeed(uint32_t seed)
+{
+    randomGenerator.seed(seed);
+}
```

`reset(seed=N)` invokes this hook before rebuilding and resetting sensors. The
patch does not change distributions, sampling order, default construction, or
physics. The build script applies it idempotently and rebuilds/reinstalls the
local Stonefish library. Per the specification, determinism was not compared.

## Part 5 — verification

`python/test_bridge.py` passed on the L40 node with `DISPLAY` and
`WAYLAND_DISPLAY` removed and `CUDA_VISIBLE_DEVICES` empty.

Verified behavior:

- Reset returned simulation time 0 and valid GPS, 9-channel IMU, and
  1-channel Compass readings.
- A 250-step equal-thrust command advanced simulation time to
  0.5000000000000003 s and moved GPS North to 0.02805 m.
- A subsequent 500-step differential command (`port=−1`, `starboard=+1`)
  changed compass yaw from approximately 0 to −0.55231 rad. Final reported
  thrusts were −92.55 N and +94.03 N, demonstrating differential actuation
  without attempting Gate B sign certification.
- Resetting with GPS local Z = +0.50 m produced
  `gps=[1e30, 1e30, 0, 0]` and `fix_valid=false` both immediately and after a
  physics step. The sentinel was detected rather than accepted as position
  `(0, 0)`.
- The observation key-set assertion excludes `Odometry` and `INS`.

Build and test instructions are recorded in `stonefish/README.md`.
