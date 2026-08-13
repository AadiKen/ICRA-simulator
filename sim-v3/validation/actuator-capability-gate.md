# Integrated actuator capability gate

This gate runs only after the behavior-preserving actuator migration passes the
original immutable golden. It is a deliberate behavior change and must not alter
the pre-migration reference retroactively.

## Required integrated behavior

- Commands strictly inside the configured dead zone produce zero thrust while
  commands at each documented threshold follow an explicit, tested boundary rule.
- Sub-dead-zone commands draw configured idle/control power despite producing no
  thrust; propulsion energy and actuator energy remain separately attributable.
- `healthy`, `failed-off`, and `stuck` transitions are deterministic, checkpointed,
  replayable, and visible in actuator state artifacts.
- Entering `failed-off` during a run removes that actuator's wrench contribution
  and emits exactly one `ACTUATOR_FAILURE` event with actuator id, step, mode, and
  triggering source. Recovery, if permitted by configuration, emits a separate
  state-transition event.
- `stuck` holds the documented command/state variable and continues to respect
  physical thrust, azimuth, rate, and power limits.
- Vehicle C `failed-off` allocation removes the actuator from the controllable set,
  recomputes the reduced allocation matrix and reachable wrench set, and exposes
  rank, singular values, condition number, achieved wrench, and residual.
- Vehicle C `stuck` allocation retains the actuator's measured constant thrust and
  azimuth as an exogenous bias wrench. The remaining controllable actuators solve
  for `commanded wrench - stuck bias`, subject to their actual bounds and rates.
  Reachability is evaluated against this biased constrained set, never the nominal
  allocation matrix. An unreachable command produces an explicit infeasibility
  result and residual; it must not be presented as successful least-squares
  delivery.
- Both failure modes keep commands finite, respect limits, and degrade according
  to configured wrench priorities, but their diagnostics and acceptance fixtures
  remain separate because rank reduction and bias rejection are different cases.
- USV-Bench-36 actuator-failure scenarios execute through the complete command →
  actuator state → thrust → wrench → event → metric path; an isolated allocator
  unit test is insufficient.

## Acceptance artifacts

- Boundary table for both dead-zone signs and exact/adjacent thresholds.
- Mid-run failure and stuck traces with checkpoint/restore replay identity.
- Energy table covering zero command, sub-dead-zone command, productive thrust,
  saturation, failed-off, and stuck states.
- Separate Vehicle C `failed-off` rank-reduction and nonzero-angle/nonzero-thrust
  `stuck` bias-rejection reports, including reachability and infeasibility cases.
- Benchmark smoke artifact containing an `ACTUATOR_FAILURE` producer event and
  the corresponding failure-category summary row.

Until these artifacts pass, actuator-failure benchmark configurations and the
integrated Vehicle C failure claim are marked unsupported, and MMG work does not
start.

## Intentional golden supersession

The legacy actuator golden is retained permanently. After capability code is
committed, `validation/behavior-supersession/create.mjs` creates a second golden
tagged with that implementation commit plus a path-by-path, justified delta
artifact. It refuses capture unless the original legacy checksum still matches.
The capability golden and delta land in a separate commit; default rebaselining
remains forbidden.
