# DO THIS NOW

Hand calculation confirms strong analytical stability (roll GM ~+2.17m,
pitch GM ~+5.43m) -- the geometry itself is fine. But the vessel crossed 60
degrees in 6.6s, which is inconsistent with a linear small-angle problem: a
sign error in the small-angle regime would cause smooth immediate
divergence, not a hold-together-then-tumble pattern. That pattern looks like
a large-angle nonlinear failure the GM check can't see -- most likely in how
the two-point-per-pontoon buoyancy discretization behaves as one of those
points approaches full emersion/submersion. Test across a range of angles,
not at one point.

## Build a static force/moment probe for vrx::Surface

Hold the vessel at a sequence of FIXED angles, zero velocity, zero thrust,
and read the plugin's computed restoring force/moment directly (not through
a dynamics run -- a static probe, same spirit as the earlier
WorldFluidAddedMassMatrix / mass-matrix readback checks).

## Roll sweep

Fixed roll angles: 0, 5, 10, 20, 30, 45, 60 degrees (matches the actual
observed failure trajectory -- 60 is where it tumbled). Zero pitch, zero
heave offset from equilibrium draft.

For each angle, record the restoring moment. Report as a table, not just
pass/fail at each point.

## Pitch sweep

Same set of angles (0, 5, 10, 20, 30, 45, 60 degrees), zero roll, zero heave
offset.

## Heave sweep (secondary, same pass)

A couple of fixed heave offsets from equilibrium draft (e.g. +/- 0.02m, +/-
0.05m -- small compared to the 0.0717m equilibrium draft, since larger
offsets risk fully surfacing or fully submerging a pontoon on their own,
which is a separate thing worth knowing if it happens but note it
explicitly if so). Zero roll, zero pitch. This checks whether a heave-side
bug could be coupling into what looks like a roll problem.

## What to check in the results

1. Sign at small angles should match the analytical GM prediction (positive
   restoring) -- this confirms the probe itself is wired correctly. If it
   doesn't match even here, the probe has a bug, fix that first.
2. Does the restoring moment keep growing roughly as expected as angle
   increases (consistent with the two discrete buoyancy points staying
   partially submerged and producing more differential force), or does it
   saturate, flatten out, or reverse sign somewhere before 60 degrees? That
   reversal/flattening point, if it exists, is very likely the actual bug --
   report the specific angle where behavior changes.
3. Is there any evidence one of the two per-pontoon force points fully
   leaves the water (or fully submerges) within this angle range? That would
   explain a step-change or discontinuity in the moment curve.

## Report back

Full table for all three sweeps, plus a plain statement of where (if
anywhere) the curve stops behaving like a restoring force. Do not modify
vrx::Surface, geometry, or damping yet -- this is measurement only. The fix
gets scoped once we know whether this is a saturation/discontinuity problem
(discretization) versus something else the sweep reveals.

## Still untouched: Gate 7, SimpleHydrodynamics, added-mass configuration.
