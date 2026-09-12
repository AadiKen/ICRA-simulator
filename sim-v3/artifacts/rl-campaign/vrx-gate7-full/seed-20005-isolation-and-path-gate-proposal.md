# Seed 20005 isolation and draft path-based Gate 7 proposal

Status: diagnostic and post-hoc proposal only. This document does not change Gate 7 authorization or reclassify the recorded batch.

## Seed 20005 isolation

Seed 20005 uses a 1.165926813 m/s current toward 112.917927 degrees and the frozen `common-waypoint-transit-v1` mission. Corrected TraceV2 telemetry was used: quaternion-derived yaw and pose-derived planar velocity with wrapped yaw differencing.

The vessel-current relative velocity vector never reaches zero. Its minimum norm is 0.125907 m/s at 80.70 s. Relative-speed intervals below 0.20 m/s occurs in only two short intervals: 4.35-4.45 s (0.15 s) and 80.60-80.75 s (0.20 s). Below 0.30 m/s occurs in four intervals, none longer than 0.35 s. The along-current component crosses zero frequently, but this is not a singular vector state; the only continuous interval longer than 0.5 s with absolute along-current speed below 0.05 m/s is 54.75-55.25 s, while the full relative-speed norm remains at least 0.491526 m/s.

The path error does not originate at those low-relative-speed intervals. It first exceeds 1 m at 6.00 s, 10 m at 27.95 s, 40 m at 49.20 s, 60 m at 60.80 s, and 80 m at 73.05 s. At the minimum relative-speed norm (80.70 s), nearest-path error is already 91.084 m. Maximum error is 93.661 m at 82.45 s, when relative speed has recovered to 1.309350 m/s. Low relative speed therefore overlaps the late high-error state but does not explain the onset or sustained growth.

The legacy plausibility envelope flags 1,276 of 2,400 corrected samples. Maximum horizontal speed is 2.661102 m/s versus the actuator-only 2.627176 m/s bound; maximum yaw rate is 2.902393 rad/s versus the actuator-only 1.656921 rad/s bound. The flagged samples occur in 24 ordinary multi-second motion intervals rather than ten-sample yaw-wrap blocks or reset boundaries. Because this is the disturbed run, current and wind add forces and moments that the actuator-only bound omits. These flags are not evidence of an odometry publication artifact, and the corrected trace contains no reset/yaw-wrap spike signature.

Conclusion: the proposed low-relative-speed singularity is not supported. Seed 20005 remains a legitimate unresolved outlier, but this isolation does not identify its physical cause.

## Draft path-based Gate 7 protocol proposal

This proposal was developed after examining the 20 calm and 20 disturbed results. That is a material limitation: its thresholds are post-hoc and must not be represented as preregistered. The original checkpoint tolerance remains the historical, pre-result definition. Adoption requires review and validation on new held-out seeds.

For each seed and condition, construct the Node and VRX planar paths from all 2,400 corrected samples. For calm kinematic comparison, use each absolute trajectory. For environmental response, use `(environment-on - calm)` within each simulator before comparing paths. Compute nearest distance from every VRX point to the full Node polyline, synchronous along-track and cross-track RMS, and normalize each distance by that seed's Node path length.

The measured non-outlier envelope comprises all 20 calm seeds and 19 disturbed seeds, excluding unresolved seed 20005. Across these 39 observations, normalized mean nearest-path error averages 4.015650%; its observed maximum is 8.871097%. Normalized nearest-path RMS averages 5.037511%; its maximum is 9.852032%. Normalized along-track RMS has a maximum of 13.551046%, and normalized cross-track RMS has a maximum of 13.588129%.

The draft per-seed bounds are therefore the exact observed non-outlier envelope, not rounded values:

- normalized mean nearest-path error <= 8.871097%;
- normalized RMS nearest-path error <= 9.852032%;
- normalized synchronous along-track RMS <= 13.551046%;
- normalized synchronous cross-track RMS <= 13.588129%.

A path-based kinematic pass would mean that VRX reproduces the geometry of Node's calm planar trajectory within those four bounds; it would not assert sample-synchronous phase agreement around a loop or deterministic per-state equality. A path-based environmental-response pass would mean that VRX reproduces the geometry and along/cross-track scale of Node's disturbance-induced path after subtracting each simulator's calm response; it would not validate instantaneous wrench equality, identical loop phase, or transient equality at a particular checkpoint.

Seed 20005 exceeds the proposed normalized mean bound at 19.334193%, with normalized RMS 23.367288%, along-track RMS 18.688792%, and cross-track RMS 21.904733%. The low-relative-speed test did not explain it, so the protocol should not include an allowance for occasional low-relative-velocity cases. It should remain a failure requiring independent diagnosis. No outlier budget is proposed.

Before adoption, these post-hoc bounds should be frozen and evaluated on a new held-out seed set. Failure on that independent set would require revising or rejecting the proposal, not widening the thresholds again.
