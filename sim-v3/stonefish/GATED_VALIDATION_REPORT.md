# Stonefish Gate D and pre-training validation

## Completion logging

Stonefish now uses the simulator-neutral completion-fraction definition in
`packages/python-client/bcod_sim/common_task.py`. The numerator comes directly
from the shared progress reward component, while the denominator is the full
seeded route length including the start-to-first-waypoint leg. Per-step reward
CSV rows include completion fraction, success, waypoints reached, and
termination reason. This logging change does not alter Gate D physics results
and did not launch training.

Reward alone is not the primary comparison metric. Completion fraction reduces
the known scale and geometry confounds, but simulator-specific hull response
(including HoloOcean's stiffer damping) can still change completion without a
policy-quality change; reports must show reward, completion, and success side
by side.

Date: 2026-09-07  
Remote host: `codenimbus-000-3.csl.illinois.edu` (`x86_64`, 32 visible CPUs)  
Training/PPO started: no

## Part 0 — Gate C full-command loose end

The full-command behavior is **not** a continuing monotone approach to a
terminal speed. A 120 s diagnostic remained finite and bounded, with a final-
60-second mean speed of 2.052175481057 m/s, range 1.868751060475–
2.215908701807 m/s, and linear tail slope of only
0.000039644722 m/s per second. Maximum absolute roll was 0.720130364187 rad
(41.26°) and maximum absolute pitch was 0.097030812730 rad (5.56°).

The non-settling observed in Gate C is bounded surface-coupled hull
attitude/yaw oscillation around a drag-limited regime. There is no unbounded
state or controller/integrator term in this open-loop case. It is benign at the
approximately 1 m/s task operating point, where the controller remains well
below this regime.

During action-parity verification, the original propeller center at NED
`z=0.30 m` was found to cross Stonefish's native fluid boundary briefly during
startup, causing physical thrust to drop to zero even though the first-order
state continued rising. Moving it directly downward to `z=0.40 m` is the only
plant change in Gate D. A rejected `z=0.60 m` trial kept it submerged but added
enough pitch moment to capsize the full-command vessel; that value was not
retained. At `z=0.40 m`, thrust remains continuous and the 120 s state remains
bounded.

## Part 1 — throughput

Each configuration executed 4,096 aggregate contract control steps, with 50
Stonefish 0.002 s integration steps per 0.1 s control step. Every environment
was an isolated console process using one physics thread. Engine startup is
reported separately from synchronized steady stepping.

| Parallel instances | Control steps/s | Internal physics steps/s | End-to-end wall time incl. startup (s) |
|---:|---:|---:|---:|
| 1 | 490.735 | 24,536.7 | 12.285 |
| 2 | 984.311 | 49,215.5 | 8.106 |
| 4 | 1,964.235 | 98,211.7 | 6.041 |
| 8 | 3,902.134 | 195,106.7 | 5.021 |
| 16 | 7,704.407 | 385,220.4 | 4.556 |
| 32 | **14,361.552** | **718,077.6** | 4.420 |
| 64 | 12,293.559 | 614,678.0 | 8.989 |

Throughput peaks at 32 instances and regresses by 14.4% at 64 instances. For
context, this measured peak is about 10.2 times the supplied HoloOcean peak of
approximately 1,409 control steps/s at four instances.

The benchmark ran through the requested `ssh codenimbus` connection on the
host named above. `nvidia-smi` could not communicate with an NVIDIA driver, so
the process could not independently verify that an L40 was exposed. This is a
CPU-only console benchmark, but the device-visibility caveat must remain on any
strict hardware-matched comparison.

Equal timesteps remain the comparison rule. These results affect wall-clock
cost only and do not change a training budget.

## Part 2 — shared reward port

Reward evaluation imports
`packages/python-client/bcod_sim/common_task.py`; no reward equation was copied
into the Stonefish adapter.

- shaping `gamma` is asserted to be exactly 1.0 at task construction;
- shaping `k` is read from the frozen contract;
- timeout is treated as non-absorbing and retains the observed next-state
  potential;
- a direct timeout probe moving from final distance 10 m to 12 m produced
  shaping `-0.351441772244089`, exactly the expected
  `0.1757208861220446 × (10 − 12)`, rather than a positive terminal bonus.

## Part 3 — action parity

The retained interface is normalized `[-1,1]`, ordered port then starboard,
with positive values producing forward thrust. Each thruster has an exact
±95 N ceiling and a configured 0.25 s first-order time constant. The signed
probe `(0.5,-0.5)` reached `(+47.499999909662,
−47.499999909662) N` after 5 s.

Stonefish integrates its `FirstOrder` state by an explicit 0.002 s update.
Measured forces match that discrete law to a maximum absolute error of
`5.68e-13 N`. Relative to an analytic continuous exponential with the same
0.25 s time constant, the largest sampled transient difference is
0.140262040609 N. The steady force ceiling is unaffected: measured force at
5 s is 94.999999819323 N. Thus force-level, sign, ordering, and parameter
parity hold; the small disclosed mismatch is time discretization of the
nominally identical lag.

## Part 4 — pre-training validation

The adapter applies the frozen calm-water route randomization (route rotation,
start offset, and heading), derives controller velocity from successive GPS
positions, uses Compass heading and IMU yaw rate, and drives the bridge at
10 Hz. LOS-PID-v2 includes the requested conditional-integrator anti-windup and
cosine turn-speed scheduling from 1.0 down to 0.5 m/s.

| Seed | Success | Control steps | Shaped return | Base return | Mean surge speed (m/s) |
|---:|---:|---:|---:|---:|---:|
| 7319 | yes | 755 | 244.444431 | 232.462706 | 0.986652 |
| 7320 | yes | 756 | 243.903533 | 231.921809 | 0.986139 |
| 7321 | yes | 756 | 243.947554 | 231.965829 | 0.986182 |
| 7322 | yes | 755 | 243.943523 | 231.961799 | 0.986373 |
| 7323 | yes | 755 | 243.922181 | 231.940457 | 0.986390 |

Result: **5/5 success, 100% success rate, median shaped return
243.943522981189, and every shaped return clearly positive.**

### Correction findings

Both five-seed ablations also succeeded:

- Without the PI integral, median transit increased to 1,017 control steps and
  median mean surge speed fell to 0.729378 m/s. Integral action was not required
  for eventual calm-water success, but it was required for accurate 1 m/s
  tracking and restored substantial timeout margin.
- Without turn-speed scheduling, all five succeeded with a 755-step median and
  0.990709 m/s median mean surge speed. Scheduling was not necessary for this
  Stonefish calm route; unlike HoloOcean, the plant retained adequate turn
  authority at cruise speed. The fixed corrected controller still includes it
  as required.

### Deliberate exploit check

On paired seed 7319, the corrected approach made +66.211973 m final-goal
progress and scored shaped/base returns `+244.444431/+232.462706`. Constant
moderate reverse thrust retreated −77.588843 m, reached the full 120 s timeout,
and scored `−1102.065892/−1088.431912`. Retreating and timing out therefore
scores decisively worse than making progress.

## Decision and stop condition

The pre-training validation gate **passes**. Stonefish is ready for training
review as the second arm. PPO training was not started and requires confirmation
and review first.
