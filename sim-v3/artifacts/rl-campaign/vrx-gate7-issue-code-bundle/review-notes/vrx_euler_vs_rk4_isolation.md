# DO THIS NOW

Check 1 confirmed systematic (not noise): sway residual negative in all 11
samples, Pearson correlation with yaw rate -0.878, smoothly rotating
direction. Check 2 confirmed Node uses RK4, DART uses first-order
semi-implicit Euler at 0.05s. But comparing two different codebases (Node
vs. DART) conflates integrator order with everything else that differs
between them (library internals, float handling, constraint solve details).
This step isolates integrator order as the single variable.

## Step 1 -- Confirm DART's exact scheme before implementing anything

Before writing a Node-side Euler stepper, pin down precisely what
"semi-implicit Euler" means in DART's implementation: typically velocity
updated first using force evaluated at the start-of-step state, then
position updated explicitly using the NEW velocity (semi-implicit / symplectic
Euler), as opposed to plain explicit Euler (position updated using the OLD
velocity). Getting this distinction right matters -- if the Node-side stepper
implements the wrong variant, the comparison won't mean anything. Check the
DART numerical-methods doc already referenced, and if it's ambiguous, check
gz-physics's DART integration call site directly.

## Step 2 -- Add a temporary semi-implicit Euler stepper to Node

- New stepper, alongside the existing RK4 one in `integrator.js` --
  additive, don't modify or replace the RK4 path. This is diagnostic-only,
  same discipline as every other diagnostic script in this investigation:
  isolated, not wired into production training.
- Same force model (mass matrix, damping, bias/Coriolis) as RK4-Node uses --
  only the time-stepping scheme changes.
- Same 0.05s step size as the VRX world.

## Step 3 -- Run the same coast trajectory three ways

- RK4-Node (existing, the "ground truth" already used throughout this
  investigation)
- Euler-Node (new, from Step 2)
- VRX (existing telemetry, already have it)

Diff Euler-Node against RK4-Node using the exact same method as the
single-step bias check (matched start-of-step nu_0, no averaging), at the
same sample times used in the 11-sample table (1.90, 2.15, 2.45, ... 5.15s).

## Step 4 -- Compare the two residual patterns

Report side by side:
- Euler-Node vs. RK4-Node residual (sign, magnitude, correlation with yaw
  rate) at each sample time
- VRX vs. RK4-Node residual (already have this, the 11-sample table)

If they match in sign, rough magnitude, and yaw-rate correlation: integrator
order alone explains the systematic residual, cleanly and with a mechanism,
not just a correlation. This becomes the quantitative basis for the Gate 7
tolerance discussion.

If Euler-Node's residual is smaller than, differs in sign from, or doesn't
correlate the same way as VRX's residual: integrator order is a partial
explanation at most. Report the gap between the two residual patterns
explicitly (e.g. as a remaining-residual table: VRX-vs-RK4 minus
Euler-vs-RK4) -- that gap is the real remaining unknown, and its size is the
new open question.

## Still do not draft or apply any Gate 7 tolerance change, and do not touch
   Gate 7, geometry, SimpleHydrodynamics, or added-mass configuration.
   Report back with the comparison before proposing anything.
