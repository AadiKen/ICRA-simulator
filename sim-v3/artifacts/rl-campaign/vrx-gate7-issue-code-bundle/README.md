# VRX Gate 7 physics-discrepancy bundle

This bundle contains the source code and compact evidence directly related to the
Surveyor VRX/Node Gate 7 investigation. It intentionally excludes raw telemetry,
runtime logs, compiled shared libraries, Docker images, and unrelated training
artifacts.

## Contents

- `validation/rl-campaign/ports/`: VRX model generation, episode preparation,
  command replay, telemetry export, scoring, plugins, and diagnostic transforms.
- `validation/rl-campaign/diagnostics/`: focused added-mass, Coriolis, damping,
  integration-order, force, thrust, surface-restoring, open-loop, and planar-lock
  analysis scripts.
- `core/` and `packages/core/`: Node mass, Coriolis, damping, force assembly, and
  coupled-six reference implementation used by the comparisons.
- `model-inputs/`: Surveyor plant and task-contract inputs.
- `evidence/`: compact JSON/Markdown results needed to understand the findings,
  including the latest planar-relock measurement-path invalidation.
- `review-notes/`: supplied investigation proposals retained as context, when
  present on the source machine.

## Latest status

The open-loop command replay is exact to floating-point roundoff, but the
unconstrained calm trajectory diverges strongly. The attempted planar-relock run
cannot yet separate out-of-plane coupling from the unresolved planar residual:
the artificial joint chain makes the existing odometry topic report the
model/carriage frame rather than base-link motion. Production Gate 7, geometry,
added mass, and damping were not changed by that diagnostic.
