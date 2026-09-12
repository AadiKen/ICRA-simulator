# Sensor-driven demo policy

This package implements the demo-scoped v2 sensor policy contract without modifying the portable PPO trainers. See `P0_FINDINGS.md` for repository verification results.

The policy receives four stacked 80-value frames. Radar is noisy, range limited, dropout-prone, and line-of-sight limited; AIS is long range, cooperative-only, irregular, and stale. A light alpha-beta tracker fuses both and coasts contacts for a bounded time. Reward and termination are computed from backend ground truth, never from sensor beliefs.

The included local adapter connects the production Node plant to the repository's frozen `LOS-PID-v2` Vehicle A controller. Its validation scope is the bcod-sim reference campaign; it is not represented as field-trial evidence. Start the main training run with:

```sh
npm run train:sensor-demo
```

The quick figure configuration trains for 50,000 steps, checkpoints and
evaluates every 10,000 steps, and uses five held-out episodes per evaluation.
This is one tenth of the original training budget and is intended for rapid
curve generation rather than final policy performance.

Training displays a live bar with completion percentage, completed steps,
elapsed time, estimated time remaining, and steps per second. Pass
`--no-progress` when redirecting output to a log that should not contain live
updates.

Run the environment-and-track-zeroed ablation with `npm run train:sensor-demo-blind`. To use the field-trial autopilot later, pass `--backend-factory module:function`; the factory must return a configured `SensorDemoWaypointEnv`.

Each run writes figure-ready data beside its model:

- `progress.csv` and `progress.json`: PPO loss, policy/value loss, entropy,
  explained variance, KL divergence, clip fraction, learning rate, update count,
  steps, timing, and rolling episode performance.
- `episodes.csv`: every episode's seed, return, success, collision, termination,
  final goal distance, closest approach, track count, duration, and decomposed
  reward totals.
- `evaluations.csv` plus per-evaluation JSON: deterministic held-out success,
  collision, return, distance, clearance, and episode-length results.
- `checkpoints/`, `best-model.zip`, `status.json`, and `run-metadata.json`:
  recovery, model selection, live status, configuration, and provenance.

By default, checkpoints and ten-episode held-out evaluations run every 25,000
steps. Override this with `--checkpoint-every`, `--evaluate-every`, and
`--evaluation-episodes`. Generate the loss and success figure after both runs:

```sh
npm run plot:sensor-demo-training
```
