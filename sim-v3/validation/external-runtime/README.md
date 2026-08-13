# External runtime cross-checks

The 2026-08-02 campaign is recorded in
`artifacts/external-runtime/2026-08-02-cross-checks.json`.

## Result

Capytaine executed its actual BEM solve and reproduced the committed Vehicle B
bootstrap artifact byte-for-byte. Gazebo Harmonic launched, loaded the generated
world, and advertised every required topic, but its live constant-thrust capture
failed because positive BCOD surge produced 7.0397 m of negative-north motion.
Consequently Gazebo is **not** successful validation evidence yet.

The ordinary `test:gazebo` and `test:gazebo-capture` scripts remain generator and
dry-run contract checks. They must never be reported as proof that Gazebo ran.

ROS 2, VRX, CUDA, and MPS did not run. Their infrastructure gates are explicit in
the artifact. The MSS acceptance test passed against pinned traces, but the pinned
MSS source checkout was unavailable, so Octave regeneration was not performed.

## Gazebo supersession

The original failed Gazebo result is retained unchanged. It is superseded by
`artifacts/gazebo/live-cross-check/report.json`, which verifies the original
artifact checksum before naming the exact failed check it replaces. The repaired
campaign passes all six planar maneuvers. Its scope remains an implementation
cross-check with a stabilized primitive-buoyancy fixture; it does not validate
coupled6 hydrostatics.

## Decision

The Gazebo planar maneuver cross-check is now **passed and safe to move on from**.
The overall external-runtime phase still **needs more work** because ROS 2, VRX,
CUDA, MPS, and fresh MSS regeneration remain blocked, and Gazebo coupled6
hydrostatics are outside the repaired fixture's evidence scope.
