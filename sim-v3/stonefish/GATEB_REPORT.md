# Stonefish Gate B — frame signs and 15-field mapping

Date: 2026-09-07  
Host: `codenimbus` (L40 node)  
Frozen contract content SHA-256: `2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36`

## Part 1 — directional sign checks

Each check began from an independent reset with seed 4242 and applied the
listed constant command for 500 Stonefish steps (1.0 s). Positive NED yaw is a
clockwise turn viewed from above: starboard/right.

| Physical input `(port, starboard)` | Compass yaw (rad) | IMU yaw rate (rad/s) | North displacement (m) | East displacement (m) |
|---|---:|---:|---:|---:|
| Port forward, starboard reverse `(1, -1)` | +0.275391251720 | +0.920104240499 | -0.009773614214 | -0.001268088541 |
| Port reverse, starboard forward `(-1, 1)` | -0.275412628171 | -0.920126196089 | -0.009610614040 | +0.002056221960 |
| Equal forward `(1, 1)` | +0.000083098456 | +0.000306965828 | +0.350699200225 | +0.000442214989 |

The first command therefore produces the expected positive/right NED turn.
Reversing the command reverses the sign. The absolute yaw-rate magnitudes
differ by 0.002386% (relative), so they are near-identical. Equal forward
thrust leaves only 0.000307 rad/s yaw rate and 0.442 mm east displacement after
one second; those residuals establish the numerical/asymmetry noise floor for
this check. No sign correction was required.

Gate A's reported **-0.55231 rad** was produced by `port=-1` (port reverse) and
`starboard=+1` (starboard forward) for 500 internal steps, after a preceding
250-step `(0.5, 0.5)` forward segment. It was a final Compass heading, not a
yaw-rate measurement and not a from-rest maneuver.

## Part 2 — frozen 15-field mapping

The mapping is based on the actual frozen bcod-sim contract file. Its canonical
content hash and 15-field revision were independently checked by
`surveyor-15-field-contract.test.ts` before the Stonefish diagnostic ran.

| # | Frozen contract field | Stonefish source and conversion |
|---:|---|---|
| 1 | `imu.linear_accel_x` | `IMU[6]`, body forward acceleration, identity |
| 2 | `imu.linear_accel_y` | `IMU[7]`, body right acceleration, identity |
| 3 | `imu.linear_accel_z` | `IMU[8]`, body down acceleration, identity |
| 4 | `imu.angular_rate_x` | `IMU[3]`, roll rate about forward, identity |
| 5 | `imu.angular_rate_y` | `IMU[4]`, pitch rate about right, identity |
| 6 | `imu.angular_rate_z` | `IMU[5]`, yaw rate about down, identity |
| 7 | `imu.orientation_yaw_rad` | `Compass[0]`, unchanged NED heading; no gyro integration or IMU orientation substitution |
| 8 | `gps.relative_goal_north_m` | configured goal north minus `GPS[2]` local north |
| 9 | `gps.relative_goal_east_m` | configured goal east minus `GPS[3]` local east |
| 10 | `gps.fix_valid` | Gate A GPS sentinel detection, encoded as `1.0`/`0.0`; relative goal is zeroed when invalid |
| 11 | `previous_action.effector_0` | last applied normalized port command |
| 12 | `previous_action.effector_1` | last applied normalized starboard command |
| 13 | `previous_action.steer_0` | `0.0`; Vehicle A has no steering effector and the contract requires unused fields to be zero |
| 14 | `previous_action.steer_1` | `0.0`; Vehicle A has no steering effector and the contract requires unused fields to be zero |
| 15 | `normalized_time_remaining` | `(2400 - elapsed contract physics steps) / 2400` |

### Actual frame transform

- World position: Stonefish NED to contract NED is
  `diag(+1, +1, +1)` (identity).
- Body acceleration and angular rate: Stonefish FRD to contract FRD is
  `diag(+1, +1, +1)` (identity).
- Heading: Stonefish Compass NED yaw is passed unchanged. Positive yaw is
  clockwise about Down.
- Stonefish GPS latitude/longitude fields are not used; its local north/east
  fields supply the contract's local tangent coordinates.

There are no silently approximated sensor fields. Fields 11–14 are action-stream
metadata rather than sensor readings, and field 15 is task-clock metadata.
Steering fields 13–14 have no physical Vehicle A effector and are explicitly
zero. Ground velocity is not acquired or mapped because it is excluded by the
frozen contract.

Stonefish continues to integrate internally at 0.002 s. The adapter advances 25
internal steps per frozen 0.05 s physics step and two frozen physics steps per
0.1 s control interval.

## Part 3 — end-to-end diagnostic trace

The fixed open-loop sequence used target `(north=20 m, east=5 m)`, seed 731,
and the following 10 Hz commands:

1. reset;
2. 10 steps `(0.55, 0.55, 0, 0)`;
3. 10 steps `(0.70, -0.35, 0, 0)`;
4. 10 steps `(-0.35, 0.70, 0, 0)`;
5. 5 steps `(0, 0, 0, 0)`.

All 15 fields at every reset/control sample are stored in
`gate_b_results.json`. Summary:

- 36 samples, each exactly 15 float values (540 values total)
- all values finite; 0 NaNs
- GPS valid in 36/36 samples
- Compass yaw range: -0.000000338251 to +0.523090813537 rad
- yaw at phase endpoints: +0.000043896640 rad after forward,
  +0.291439663831 rad after the commanded right turn,
  +0.454797370295 rad with yaw rate already reversed to
  -0.382086364065 rad/s after the left-turn segment, and
  +0.245041408196 rad after coast
- relative goal changed from `(20.0, 5.0)` m at reset to
  `(18.667573002562, 4.842768540212)` m at the final sample
- normalized time remaining changed from 1.0 to 0.970833333333, exactly 70
  elapsed frozen physics steps out of 2400

This is only the requested mapping diagnostic. Reward, LOS-PID-v2,
terminations, determinism comparisons, and training were not implemented or
started.
