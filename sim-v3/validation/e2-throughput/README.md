# E2 throughput heuristic pilot

This is a short, non-publication audit. It times a small number of steady-state
Vehicle A `planar3` steps and linearly estimates the wall clock for a 300-step
rollout and the eventual 3-warmup/10-measured-rollout protocol. Every estimate
is labeled `heuristic-pilot-extrapolation` in the artifact.

Run `npm run pilot:e2-throughput`. The full sweep, bridge isolation, memory
measurement, and E4/E5 training matrices remain explicitly deferred. The pilot
must not be cited as the completed E2 result.
