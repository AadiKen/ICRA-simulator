# Shared LOS guidance and Vehicle C allocation

Vehicle A and Vehicle C use the same simulator-neutral `LOS-PID-v2` guidance law in `validation/rl-campaign/ports/portable-controllers.ts`. For a leg with signed cross-track error `e`, its desired heading is `leg_heading - atan2(e, lookahead)`. The surge loop regulates body surge speed and the yaw PD loop regulates heading; their output contract is a desired planar wrench `[surge force, yaw moment]`. Vehicle-specific code may map that wrench to actuators, but must not duplicate or modify the guidance law.

## Vehicle C design decisions

**Allocation strategy: minimum thrust norm.** Vehicle C minimizes the sum of squared port and starboard thrust magnitudes while satisfying requested surge, sway, and yaw. This is the Moore-Penrose solution over each pod's body-frame force components. It is the standard deterministic choice for an underdetermined allocator, minimizes the quadratic thrust surrogate, and avoids adding an unvalidated visual-symmetry preference. If a pod limit is reached, all force components are scaled together so the requested wrench direction is retained.

**Pure-sway moment cancellation: included as part of the general allocator.** It is not a bonus special case. For pod position `(x,y)`, yaw is `x Fy - y Fx`; therefore the 1.5 m aft placement couples lateral force into yaw. The general surge/sway/yaw constraint automatically adds equal-and-opposite surge components to cancel that moment. Consequently, a zero-yaw pure-sway allocation is not exactly “both pods at ±90°”; that naive command produces an uncompensated yaw moment at the aft offset.

The same allocator is the production bcod-sim path and is intended to be reused unchanged by the Gazebo adapter after Gate D. Vehicle C’s hydrodynamic coefficients remain design-derived and behaviorally unvalidated; the bcod-sim rate is a simulator reference, not physical validation.

## Reference-rate provenance

The frozen contract's `0.66` classical reference is the 200-episode, full-route Surveyor LOS-PID-v2 result in `terminal-radius-sweep-rescaled.json`, at the selected 2 m pass-through radius and 120 s timeout. It is distinct from the numerically identical `0.66` produced by the 1.5M-step RecurrentPPO checkpoint on 50 final-leg-isolation episodes.

The `0.70` LOS-PID-v2 sanity result is also a 50-episode final-leg-isolation diagnostic. Despite its label, it embeds gains of lookahead 4 m, yaw `kp=70`, `kd=35`, and target speed 1.5 m/s; the frozen portable LOS-PID-v2 implementation uses 8 m, `kp=100`, `kd=35`, and 1.0 m/s. The `0.70` is therefore neither a reproduction nor an update of the frozen `0.66`. Vehicle C's full-route result may be compared only with the full-route frozen reference, with the differing plant and actuator topology stated.

## Task 2 angle-continuity experiment

The first Task 2 candidate retained the minimum-norm body-force solution and added an opt-in angle-continuity cost when choosing between each pod force's equivalent forward-thrust and reverse-thrust representations. Production keeps the weight at zero unless explicitly selected for an experiment.

On checkpoint seeds 30000, 30003, 30007, 30013, and 30048, a positive continuity weight removed every sampled azimuth target jump greater than 90 degrees and reduced sampled angle-tracking errors greater than 45 degrees to zero. It did not resolve any trajectory: seed 30003 still made no approach, while the other four still approached and then departed. The candidate is therefore mechanically effective but rejected as the dominant trajectory fix. Its result is recorded in `artifacts/rl-campaign/vehicle-c-angle-continuity-checkpoint.json`; no full 50-seed reference is warranted for this candidate.
