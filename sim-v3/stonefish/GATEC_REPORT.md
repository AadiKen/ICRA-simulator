# Stonefish Gate C — dynamics, determinism, and terminations

Date: 2026-09-07  
Host: `codenimbus` (32 physical cores visible)  
Stonefish integration step: 0.002 s; contract control interval: 0.1 s

## Part 1 — empirical Vehicle A dynamics

Each straight-line trial started from an independent seed-9001 reset. Speed is
calculated from consecutive one-second local-NED GPS displacements. Cruise
speed is the final-five-second mean for 25-second trials and the final-ten-
second mean for the extended 60-second full-thrust trial. Settling requires
entry into and continued residence within ±2% of the reported cruise value,
with at least five seconds of observed dwell.

| Command per thruster | Force per thruster (N) | Reported cruise speed (m/s) | 2% settling time (s) |
|---:|---:|---:|---:|
| 0.25 | 23.750000000000 | 0.658788817725 | 6 |
| 0.50 | 47.500000000000 | 1.396290212287 | 7 |
| 0.75 | 71.250000000000 | 2.046507203815 | 7 |
| 1.00 | 95.000000000000 | 2.304599249559 | not settled within 60 s |

The full-thrust response initially peaks near 2.494 m/s and then has a
2.053–2.415 m/s range over the final ten seconds. It is therefore reported as
a tail mean, not falsely described as a clean steady state. Stonefish's hull is
well below bcod-sim's theoretical approximately 5.45 m/s top speed.

The task-performance criterion is nevertheless met without modifying the
hull. At the relevant half command Stonefish settles at 1.396 m/s, comfortably
above the approximately 1 m/s LOS working speed and the 0.669892 m/s minimum
average needed to cover the frozen 80.387 m centerline in 120 s. At 1 m/s the
straight-line time is 80.387 s, leaving 39.613 s of timeout margin; the measured
half-command speed corresponds to 57.57 s before turn/path overhead.

### Turning response

Each case first cruised for 10 s at `(0.6, 0.6)`, then applied the listed
differential command for 10 s. Speed is the final-five-second mean and radius
is speed divided by the final IMU yaw-rate magnitude.

| Yaw demand | `(port, starboard)` | Speed (m/s) | Yaw rate (rad/s) | Radius (m) |
|---:|---:|---:|---:|---:|
| 0.1 | (0.7, 0.5) | 1.572037401478 | +0.098296213817 | 15.992858121667 |
| 0.2 | (0.8, 0.4) | 1.481659738728 | +0.169819640403 | 8.724902109145 |
| 0.3 | (0.9, 0.3) | 1.411627186661 | +0.233017502883 | 6.058030702400 |
| 0.4 | (1.0, 0.2) | 1.346648321980 | +0.291613632438 | 4.617919644989 |

### Auditable tuning decision

No hull geometry, hydrodynamic drag parameter, mass, inertia, thruster offset,
force ceiling, or actuator lag was changed. The before- and after-response
numbers are consequently identical. The existing mesh-derived Stonefish
hydrodynamics already satisfy the explicitly stated frozen-route envelope, so
forcing its unrelated 5.45 m/s theoretical maximum would be unjustified.

The only C++ scenario additions in this gate are a seabed and a distant test
object for native contact classification. Repeating the response measurements
after those additions reproduced the baseline values exactly.

## Part 2 — determinism characterization

The prediction was written to `gate_c_determinism_preregistration.json` before
the repeat protocol ran: both thread settings were expected to be bit-identical;
default-only divergence collapsing at one thread would support scheduling as
the cause, while persistence at one thread would rule scheduling out.

Protocol for each thread setting:

- 8 independent process repeats
- seed 24680 and identical 100-step action sequence
- 101 samples per repeat including reset
- all 15 frozen float64 observation fields compared exactly
- GPS, IMU, and Compass noise explicitly enabled

| Thread setting | Repeats | Samples/repeat | Maximum divergence over every field and step |
|---|---:|---:|---:|
| Stonefish default (32 physical-core pool) | 8 | 101 | 0.0 |
| `max_physics_threads=1` | 8 | 101 | 0.0 |

The preregistered prediction was confirmed. Both settings are bit-identical;
there is no measurable scheduling-induced divergence in this scenario. Because
every field at every step ties at zero, there is no unique worst field or step.
The JSON artifact records field-level maxima and uses step 0 as the mechanical
tie location; that is not evidence that step 0 is problematic.

The seed hook was tested with nonzero sensor noise. Two seed-111 resets were
exactly equal across the raw GPS, nine IMU values, and Compass value. Seed 112
changed those draws. Thus the hook controls actual noise sampling rather than
merely storing a seed.

## Part 3 — termination taxonomy

| Frozen category | Stonefish mapping | Verification |
|---|---|---|
| Grounding | Native Bullet contact between `VehicleAHull` and named `Seabed` entity | Grounding scenario: `grounding=true`, object contact false, reason `grounding` |
| Object collision | Native Bullet contact between `VehicleAHull` and named `TestObject` entity | Object scenario: object contact true, grounding false, reason `object_collision` |
| Instability | Non-finite IMU state immediately, or absolute IMU roll/pitch above 60° continuously for 1.0 s | 61° roll for ten 0.1 s updates produced `instability` |
| Allocation failure | No clean physical allocator equivalent exists for fixed-thruster Vehicle A. Only non-finite direct normalized command or reported thrust is classified as mapping failure | Injected non-finite command produced `allocation_failure`; ordinary first-order rotor lag is explicitly excluded |

Termination precedence among these categories is instability, grounding,
object collision, then allocation failure. The normal-contact scenario produced
no termination. Grounding and object collision are cleanly distinguishable in
Stonefish because contacts retain the counterpart entity; they are not reduced
to a generic collision boolean.

The allocation limitation is intentional and explicit: this Vehicle A port has
no allocator, infeasibility result, or wrench-residual signal. Fabricating one
from rotor lag would silently change the meaning of the frozen category.

## Stop condition

Gate C stops here. Reward, LOS-PID-v2, throughput benchmarking, and training
were not implemented or started.
