# Cross-simulator common evaluation suite

This package evaluates all four portable PPO arms on one standalone planar Fossen plant. The
judge imports no bcod-sim, Gazebo, HoloOcean, or Stonefish engine. Its claim is deliberately
narrow: transfer performance to a validated-but-imperfect analytic plant under matched task
conditions—not dynamics neutrality or ground truth.

The checked-in version 1.1 candidates use a 1.0 m terminal radius, selected by the
content-hashed task-hardening sweep in `artifacts/common-suite/task-hardening/`.
The contracts remain `calibration-required`. The bcod-sim verification is off-ceiling;
provide quick native nominal success rates for the other three arms next. Freezing is
refused if any arm is at or near the floor or ceiling. Every frozen condition uses the same
200 seeds.

Run order:

1. `python -m eval.common_suite.judge.validate_against_simulators --traces TRACES.json --output judge-validation.json`
2. `python -m eval.common_suite.calibrate_ceiling --calibration-result calibration.json`
3. `python -m eval.common_suite.runner --config run-config.json --output result.json`
4. `python -m eval.common_suite.report result.json tables/`

Before step 3, generate the remaining preflight evidence with
`python -m eval.common_suite.adapters.run_replay_gates`,
`python -m eval.common_suite.checkpoint_parity`, and finally
`python -m eval.common_suite.preflight`. The last command evaluates gates in order, stops at
the first failed gate, and emits a content-hashed authorization artifact. `runner.py` refuses
to load real policies unless that artifact is present, untampered, and records all four gates
as passed.

`TRACES.json` maps all four arm names to same-plant, added-mass-consistent native open-loop
trace files. Validation reports position RMSE, heading RMSE, and a normalized domain-gap value
per arm. A failed judge-validation cell blocks the suite.

The run config supplies adapter factories, checkpoints, native replay logs, training steps,
training wall times, and the judge-validation artifact. All four adapter replay gates must
pass. Those gates establish repeatable translation only, not physical fidelity. Training
steps must match exactly before evaluation begins.

The judge uses RK4 integration, applies fixed-thruster lag once per physics step, builds the
actual 15-field policy observation, accepts two normalized thruster actions, and logs success,
mean cross-track error, completion time, integrated absolute thrust, and safety violations.
Reporting retains every condition separately, places domain gap in each result row, separates
wall-clock from sample efficiency, and applies Holm correction to the five primary
winner-versus-runner-up cross-track comparisons.

The analytic judge is not asserted ground truth. Vehicle C retains an unresolved
symmetric-thrust lateral/heading residual. Adapter effort was not blind, and a small domain gap
for one training arm remains a visible modeling confound rather than proof of superior transfer.
