# DO THIS NOW

Root cause understood: real convex nonlinear buoyancy restoring force
(confirmed correct, not a bug) combined with zero heave/roll/pitch damping
(zW/kP/mQ all zero in SimpleHydrodynamics) and a 0.05s explicit timestep.
This DOF was never in Node's planar3 scope, so there's no source-of-truth
value to restore -- zero was never validated, just never populated. Two
steps: confirm the mechanism cheaply, then propose (don't apply) a real fix.

## Step 1 -- Timestep sanity check (confirms mechanism, not a fix to ship)

Rerun the zero-thrust coast test (same one that blew up at 35.6s
articulated / 49.0s merged) at a much finer timestep -- try 0.005s and
0.001s -- everything else unchanged, still zero damping.

- If blow-up disappears or is drastically delayed at finer steps: confirms
  this is numerical stiffness from zero damping interacting with the
  explicit integration step, not a deeper physics error. Report this as
  confirmation only -- changing the project-wide timestep is NOT the fix to
  pursue, Gate 7 and the whole campaign are calibrated to 0.05s and changing
  that is a much bigger decision than this task scope.
- If blow-up persists even at very fine timesteps: that would mean something
  else is still wrong beyond damping/stiffness interaction -- report this
  immediately, don't proceed to Step 2 as scoped, flag for review instead.

## Step 2 -- Propose conservative damping estimates (proposal only, do not apply)

Since there is no Node reference value for heave/roll/pitch damping (out of
scope for planar3), any values here are new engineering estimates, not a
parity restoration. Do not pick numbers to make instability disappear --
derive them from first principles, same documentation discipline already
used for the rotor thruster estimates elsewhere in this project (labeled
engineering estimates, with stated reasoning and an explicit "not
manufacturer/measured data" caveat).

- Estimate reasonable order-of-magnitude linear damping for zW, kP, mQ for a
  hull this size (52.3 kg, 1.83m length, twin 0.17m-radius pontoons) --
  typical small-craft viscous/wave-damping coefficients, not tuned to a
  target outcome.
- State the derivation/reasoning for each value explicitly, same as the
  rotor estimate documentation pattern (classification, reasoning, any
  reference used for the power/size class comparison).
- Test these estimates against the REAL 0.05s timestep (not the fine
  timestep from Step 1) in the same zero-thrust coast test: does the vessel
  stabilize (settle toward equilibrium) rather than blow up?
- Report the proposed values, reasoning, and test result. DO NOT apply them
  to the production model.sdf, DO NOT touch Gate 7, DO NOT run Gate 7 or the
  batch with these values. This is a proposal for review, not a fix
  deployment -- these are new physical assumptions being introduced, and
  that decision point needs sign-off before it goes anywhere near a scored
  run.

## Still untouched: Gate 7, geometry, added-mass configuration,
   SimpleHydrodynamics' existing (non-damping) values.
