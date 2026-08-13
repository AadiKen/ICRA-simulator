# Simulator Readiness

## Current defensible description

This repository now provides a preserved `planar3` plant and an interactive-default
`coupled6` plant with authoritative NED position, quaternion attitude, six body
velocities, 6×6 mass and Coriolis mechanics, damping, current, wind, actuators,
and linear still-water hydrostatic restoring.

## Evidence currently available

- Automated stability, damping, guidance, actuator, hydrostatic, wave, motion,
  timestep-convergence, Gazebo-generation, and preset-parity checks.
- Deterministic Node execution without a browser or WebGL context.
- A repeatable headless performance benchmark:
  `npm run benchmark:physics -- --steps 10000`.
- Open-loop comparison tooling for generated Gazebo and local reference traces.

## Claims not yet supported

- Quantitative agreement with physical sea-trial data.
- General hydrodynamic fidelity across different hull forms.
- Physical six-axis wave excitation; rendered waves are currently visual-only
  in `coupled6`.
- Lower total computational cost than Gazebo or another robotics simulator
  under equivalent physics and sensor workloads.
- Stable browser resource behavior across repeated reset/restart cycles; cleanup
  APIs exist, but still require browser/WebGL smoke testing.

## Remaining path to externally validated 6-DoF

1. Generate and freeze external Otter maneuver traces from the pinned MSS commit.
2. Replace derived six-axis Otter coefficients with identified/measured values.
3. Add deterministic regular-wave excitation and response-amplitude validation.
4. Calibrate and validate against independent free-decay or sea-trial traces.

## Benchmark interpretation

The headless benchmark includes the simulator control loop, environment
sampling, physics, logging, metrics, and state-derived sensors. It excludes
Three.js rendering, GPU camera readback, PNG conversion, LiDAR raycasting, DOM
updates, and WebSocket receiver overhead. Browser sensor performance must be
reported separately.
