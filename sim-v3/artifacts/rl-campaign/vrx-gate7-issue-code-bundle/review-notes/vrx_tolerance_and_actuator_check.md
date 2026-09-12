# DO THIS NOW

Two things. Do them in order.

## Part 1 -- Confirm the 1e-4 tolerance is actually the right metric for this comparison

Before treating calm-parity as a decisive fail, confirm where `1e-4` comes
from. This project has at least two very different tolerance types:
determinism checks (bit-identical reproducibility of the SAME simulator
across runs -- e.g. the earlier 55-cell determinism sweep used
machine-epsilon-scale tolerances like 1.11e-16) versus cross-simulator
conformance (Gate 7's own defined tolerance is 30% / 15 degrees).

- Find the exact source of the `1e-4` value used in the calm-parity scorer.
- State explicitly: is this a determinism-style tolerance being applied to a
  cross-simulator comparison where it doesn't belong, or is it a genuinely
  intended conformance bound for this specific check?
- If it's a misapplied determinism tolerance: recompute the calm-parity
  "fail" characterization using the correct conformance-scale comparison
  (Gate 7's own 30%/15 degree metric applied to the calm run, same as it's
  applied to the disturbed run) and report that instead.
- If `1e-4` is genuinely correct for this specific check: say so and explain
  why this check is held to a tighter bound than Gate 7's own tolerance.

Report this before Part 2's results are treated as final -- it changes how
"decisively fails" should be read.

## Part 2 -- Isolate the actuator/thrust response path (never tested until now)

Every diagnostic in this entire investigation, from the original force-
vector test through the damping fix, has been a passive coast: zero thrust.
This calm-parity run is the FIRST test with active propulsion. The actuator
time constant (0.35s) is documented in the model json as "borrowed generic
Vehicle A fallback; not Surveyor-specific" -- flagged as weak provenance
from the start, never isolated.

Build a static, zero-velocity, thrust-only force/response check, same
pattern as the original current-only force-vector diagnostic:

1. Zero current, zero wind, zero initial velocity.
2. Command a known, fixed thrust value to both port and starboard thrusters
   (pick something representative of what the seed-20000 mission actually
   commands early in the run).
3. Record force output over time from BOTH simulators through the actuator
   lag -- not just steady-state magnitude, the full step response through
   the 0.35s time constant. Sample at a fine enough rate to see the
   transient shape, not just start/end.
4. Compare Node's actuator model output against VRX's thruster plugin
   output for the identical commanded step, same force/time curve.

## What this tells you

- If force-vs-time matches closely: actuation itself isn't the problem,
  divergence is in vehicle dynamics under load once thrust is applied
  (different question, would need its own isolation).
- If it doesn't match -- different time constant, different steady-state
  magnitude, different curve shape: that's very likely a real, well-targeted
  fix, especially given the known-weak provenance of the 0.35s value. Report
  exactly what differs (magnitude vs. timing vs. shape) before proposing any
  fix.

## Still untouched: Gate 7, geometry, added-mass configuration, the just-
   applied damping values (zW/kP/mQ stay as reviewed and applied). Report
   back on both parts before further action.
