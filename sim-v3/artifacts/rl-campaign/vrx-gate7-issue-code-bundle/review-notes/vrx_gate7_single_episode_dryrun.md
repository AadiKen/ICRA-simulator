# DO THIS NOW

Confirmed: every diagnostic since the start of this investigation ran on the
planar-locked harness, which Gate 7 itself never uses. The added-mass and
Coriolis fixes are model properties and carry over to the real plant. The
small systematic residual investigation (integrator isolation, sway/yaw-rate
correlation, etc.) does NOT carry over -- its relevance to Gate 7 is unknown
until tested on the real unconstrained system.

## Do this: one single-episode unconstrained dry run

Not a full Gate 7 batch (20+20 episodes) -- one episode, to check direction
and rough magnitude before committing more effort.

1. Use the exact same generation path Gate 7 batch uses: `prepare-vrx-episode.ts`
   -> `renderSurveyorVrxModel()`, no planar-constraint transform
   (`apply-vrx-planar-constraint.ts` NOT invoked). Confirm this explicitly in
   your report -- state which script/function produced the model used.
2. Model must include the current fixes: native `fluid_added_mass`
   (`xx=2.615, yy=39.225, rr=1.4312975867`), `SimpleHydrodynamics` explicit
   added-mass terms at zero (unchanged from current state).
3. Run ONE episode -- pick one seed from the existing Gate 7 seed set (e.g.
   seed 20000, already referenced) so it's comparable to prior Gate 7
   attempts. Same disturbance/current conditions as a real Gate 7 episode,
   not a bespoke coast test.
4. Compare against Node's reference trajectory for that same seed/episode,
   using whatever metric Gate 7 itself uses (kinematic parity / environmental
   vector divergence / direction difference -- the same metrics from the
   original 574.7% failure report), not the planar diagnostic's m/s2
   residual metric. This needs to be apples-to-apples with the original Gate
   7 failure numbers.

## Report back

- The resulting divergence numbers in Gate 7's own metrics.
- How they compare to: (a) the original 574.7%/77.7 degree failure, and (b)
  Gate 7's 40-70% acceptance band.
- Do NOT run the full 20+20 batch yet. Do NOT flip `execution_authorized` or
  touch Gate 7's gating. This is a single dry-run data point to decide
  whether the current fixes are pointing in the right direction at real
  6-DOF scale, before investing further effort in the planar-harness
  residual investigation.

## If the single episode lands near or inside the acceptance band:
Report this clearly -- it means the planar-residual investigation was a
useful confidence-building exercise but is not blocking, and the real batch
run is likely the right next step (separate authorization).

## If the single episode is still far off:
Report the actual numbers and which failure mode dominates (kinematic parity
vs. environmental response, same breakdown as the original failure report).
That tells us whether to look at roll/pitch dynamics or `vrx::Surface`'s
wave interaction next -- neither has been touched by any fix or diagnostic
so far.

## Still untouched: Gate 7's execution_authorized gate, geometry beyond the
   already-applied added-mass fix, SimpleHydrodynamics beyond the already-
   confirmed zeroed explicit terms.
