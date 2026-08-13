# Pre-migration golden gate

Captured from source commit `1724962` before production physics migration.

The gate covers seven independent surfaces:

1. Pinned MSS Vehicle A trajectories.
2. Six-DoF matrices, invariants, equilibrium, free decay, integration, and timestep response.
3. Deterministic wave dispersion, height, normal, and velocity response.
4. Maneuver convergence sweeps.
5. Gazebo model/world/manifest generation checksums.
6. Large-angle hydrostatic restoring characterization.
7. Actuator command, state, thrust, and wrench traces, including boundaries and transients.

Run `npm run test:migration-gate` after every migration step. The comparison emits a metric-delta table and fails on numerical changes outside the committed absolute/relative tolerances or any Gazebo checksum change. Do not recapture goldens to make a migration pass; an intentional baseline change requires a separately reviewed physics justification.

Supplemental pre-component goldens are append-only and independently checksummed.
The actuator surface freezes command clamping, forward/reverse asymmetry, lag and
rate limiting, saturation entry/hold/exit, unequal port/starboard commands, and
exact dead-zone/saturation thresholds through actuator state, thrust, and wrench.
The legacy path currently ignores configured dead-zone and failure-mode fields;
that limitation is frozen for migration parity and must be changed separately
after the parity migration lands.
