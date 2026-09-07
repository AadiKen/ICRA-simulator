# Cross-simulator completion fraction

`completion_fraction` is the clamped ratio of signed cumulative route progress
to the full seeded route length:

```text
completion_fraction = clamp(cumulative_route_progress_m / full_route_length_m, 0, 1)
```

The numerator is not independently recomputed from simulator telemetry. Each
control step contributes the exact distance delta already used by the shared
progress reward (`reward_components.progress / 2.0`). Reverse motion therefore
reduces cumulative progress, and clamping prevents values outside `[0, 1]`.

The denominator is computed once at reset from the seeded geometry: the
start-to-waypoint-0 leg plus every consecutive waypoint leg. The frozen common
task has three route targets, so its full-episode denominator contains the
three traveled segments `start -> waypoint 0 -> waypoint 1 -> waypoint 2`.
Final-leg curriculum episodes use only the isolated final leg.

Every row of `metrics/reward-components-env-<rank>.csv` now co-locates
`completion_fraction`, `success`, `waypoints_reached`, and
`termination_reason` with the reward components. The terminal row is the
per-episode result; no separate distance-derived completion metric is allowed.

Reward alone must not be treated as the primary cross-simulator comparison
metric because reward scale is affected by route geometry, cross-track error,
and simulator-specific hull dynamics. Completion fraction reduces those
confounds but does not eliminate them: for example, HoloOcean's stiffer damping
can improve route completion independently of policy quality. Report reward,
completion fraction, and success side by side.

This definition is implemented once in
`packages/python-client/bcod_sim/common_task.py` and is imported by bcod-sim,
Gazebo Harmonic, VRX, HoloOcean, and Stonefish. The scripted cross-simulator
check is `validation/rl-campaign/verify_completion_fraction_consistency.py`.
