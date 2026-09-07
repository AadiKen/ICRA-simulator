# HoloOcean fixed-action scenario and actuator audit

## Completion-fraction logging addendum

HoloOcean now imports the same completion tracker as bcod-sim, Gazebo, VRX,
and Stonefish. The tracker consumes HoloOcean's already-computed shared
progress reward rather than recomputing displacement from HoloOcean telemetry,
and normalizes it by the full seeded route length. Its reward-component CSV
rows now include completion fraction, success, waypoints reached, and
termination reason. This logging change does not change the documented hull
response limitation and did not launch training.

Completion is a better cross-simulator proxy than raw reward, but it is not
physics-invariant. In particular, HoloOcean's stiffer damping may inflate
completion fraction independently of policy quality, so reward, completion,
and success must remain side-by-side measurements.

Date: 2026-09-07  
Scope: reset-parity correction, diagnosis, and audit instrumentation. No reward,
actuator, damping, or training parameters were changed.

## Finding

HoloOcean had a reset-parity failure, but seed 10000 was not made substantially
easier by route length or initial heading before correction:

| Seed-10000 quantity | bcod-sim | HoloOcean | Interpretation |
|---|---:|---:|---|
| Requested first-waypoint distance | 27.1391 m | 27.1391 m | Identical by construction |
| Native observed first-waypoint distance | 27.1391 m | 26.5925 m | HoloOcean is 0.5466 m (2.01%) shorter after its 0.55 m spawn/GPS offset |
| First-leg bearing | -0.1802° | +0.6676° | Different reset draw |
| Requested initial heading | +3.0567° | -5.6816° | Different reset draw |
| Absolute heading error | 3.2369° | 6.3492° | HoloOcean starts less favorably aligned |

The HoloOcean adapter now uses the literal bcod-sim `Mulberry32` generator and
frozen draw order. Stonefish was found to retain the same NumPy mismatch and was
aligned as well so the requested three-way rerun would use the same route,
start, and heading.

## Reset parity

The five-seed HoloOcean parity gate now passes. Maximum residuals are numerical
roundoff from translating bcod-sim's 10,000 m world origin: 1.18e-12 m for
route coordinates and 1.42e-12 degrees for headings, below the 1e-9 tolerance.
Current and wind draw errors are exactly zero.

Result: **PASS — route, start, heading, current draw, and wind draw match.**

## Actuator path

The exact seed-10000 fixed-action path was rerun in live HoloOcean on the GPU
cluster. For 25 control steps, the audit independently calculated:

1. normalized action multiplied by the configured 501.1328125 N ceiling;
2. two 0.05 s first-order updates per control step with time constant 0.25 s;
3. the force actually submitted to HoloOcean with wind mode off.

Reported target force, wrapper lag state, and native applied force matched the
independent calculation exactly for every sampled step:

- maximum target-force error: 0 N
- maximum lag-state error: 0 N
- maximum applied-versus-lagged error: 0 N

Result: **PASS — the documented thrust rescaling and actuator lag are active,
not merely configured.**

## Damping and drag capability

HoloOcean's built-in SurfaceVessel exposes no scenario or Python API parameter
for its native damping or drag. The engine source hardcodes:

- mass: 200 kg
- drag coefficient: 0.8
- drag area: 1.0 m²
- Unreal linear damping: 3.0
- Unreal angular damping: 0.75

The Python SurfaceVessel documentation explicitly states that editing mirrored
constants does not change C++ thruster/PD behavior. HoloOcean offers a custom
dynamics control scheme, but using it would replace the built-in plant rather
than tune a physical parameter exposed by the native thruster scheme. No
scale-fitting or engine rebuild was attempted.

Result: **DOCUMENTED STRUCTURAL LIMITATION — native damping is not tunable.**

## Post-fix matched-action rerun

The final seed-10000 rerun used the same route, start, heading, 1,200 normalized
actions, reward, and zero wind/current in all three simulators:

| Simulator | Return | Mean cross-track | Cross-track reward |
|---|---:|---:|---:|
| bcod-sim | -97.891 | 6.488 m | -155.714 |
| Stonefish | -414.496 | 16.111 m | -386.668 |
| HoloOcean | -27.288 | 0.847 m | -20.334 |

The values do not converge. HoloOcean's cross-track deviation is 7.66 times
smaller than bcod-sim's and 19.02 times smaller than Stonefish's. Its residual
return advantage is therefore documented as a native hull/damping response
difference. The comparison uses identical normalized actions and each backend's
declared topology-native actuation; HoloOcean retains its previously approved
performance-calibrated 501.133 N ceiling, not bcod-sim's literal 95 N ceiling.

## Added instrumentation

The portable PPO trainer now wraps every training environment—bcod-sim,
Stonefish, HoloOcean, Gazebo, and VRX—with a backend-neutral streaming component
trace. Each environment writes:

`metrics/reward-components-env-<rank>.csv`

Every control step records episode index and seed, control and physics step,
progress, cross-track, action-delta, terminal, base reward, potential shaping,
shaped reward, termination reason, and termination/truncation flags. Files are
flushed after every row so a stopped job retains its evidence. Completed run
manifests declare the trace glob and column schema.

This instrumentation does not alter observations, actions, rewards, episode
state, or PPO configuration.

## Artifacts

- `artifacts/rl-campaign/holoocean-reset-scenario-parity.json`
- `artifacts/rl-campaign/holoocean-fixed-action-path-audit.json`
- `artifacts/rl-campaign/cross-simulator-reward-audit.json`
- `artifacts/rl-campaign/cross-simulator-fixed-action-calm-after-rng-fix.json`
- `artifacts/rl-campaign/holoocean-surface-vessel-damping-limitation.json`
