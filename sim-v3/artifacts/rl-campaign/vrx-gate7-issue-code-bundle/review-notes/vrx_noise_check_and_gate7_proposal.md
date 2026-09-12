# DO THIS NOW

Strict start-state single-step comparison reduced the residual from 0.105
m/s2 to 0.002-0.006 m/s2 (17-53x smaller), independently reproduced. This
reframes the whole investigation: the force model (mass matrix, damping,
bias/Coriolis) looks essentially correct per-step. The 0.105 m/s2 seen in
every prior full-trajectory test is likely accumulated integration
divergence between two independently-evolving nonlinear simulators, not a
missing force term. Two checks before that conclusion is accepted.

## Check 1 -- Is the residual noise-like or systematic?

- Run 5-10 more single-step samples across the existing coast telemetry (same
  method as the last 3: matched nu_0, no averaging), spread across the run.
- For each, record the residual vector (not just its magnitude).
- Check whether the residual vector's direction correlates with velocity
  direction or yaw-rate direction (would indicate a small remaining
  systematic term) or appears uncorrelated / sign-flipping (consistent with
  integration-scheme noise, not a missing force).
- Report the full table plus this characterization. Don't just report "small
  enough" -- show the pattern.

## Check 2 -- Integrator scheme match

- Identify Node's actual time-stepping scheme (check the stepper
  implementation -- RK4, semi-implicit Euler, etc.) and its order.
- Identify DART/Gazebo Physics 7.6.0's default integration scheme for this
  world (check the `dart` physics engine config in the world SDF, or
  gz-physics/DART documentation/defaults if not explicitly set).
- Report both explicitly. If they differ (e.g. Node uses a higher-order
  method, DART uses semi-implicit Euler), that alone is expected to produce
  small per-step disagreement at a 0.05s step size between two otherwise-
  correct implementations of the same continuous-time model -- report this
  as the likely source of Check 1's residual, don't treat it as still
  mysterious if it's a known, expected effect of differing integrator order.

## If both checks come back clean (residual noise-like, integrator mismatch
   confirmed as plausible explanation):

This is no longer a bug hunt -- it's evidence for redefining Gate 7 with a
quantitative basis, not by loosening a failing test post-hoc (the concern
flagged much earlier in this investigation). Draft, but do NOT apply yet:

- A proposed Gate 7 tolerance derived from the actual per-step force-model
  agreement characterized in Check 1 (e.g. residual as a percentage of
  typical acceleration magnitude at these test velocities), not a
  round-number guess.
- Explicit acknowledgment that this tolerance is justified by the underlying
  physics-model agreement now demonstrated, and is being set BEFORE any new
  Gate 7 run under the new definition -- not fitted to make a specific
  failing run pass.
- What Gate 7 would test under this definition (bounded behavioral
  equivalence) versus what it tested before (near-exact trajectory
  matching), stated plainly so the change in what's being claimed is
  explicit, not buried.

Present this proposal for review. Do not apply it, do not rerun Gate 7 under
it, until reviewed.

## If either check does NOT come back clean:

Report plainly what's inconsistent (e.g. residual correlates with a specific
direction, or integrators already match and can't explain the gap). Do not
proceed to the Gate 7 redefinition proposal -- go back to treating the
single-step residual as still needing explanation.

## Still untouched: Gate 7 itself, geometry, SimpleHydrodynamics, added-mass
   configuration.
