# Cross-simulator reward and harness divergence audit

Date: 2026-09-07  
Scope: diagnosis only; no simulator, reward, task, or training code was changed.  
Audited revision: `1f1afad79a6a09491f67abd0788271a0f55a6f74`

## Bottom line

The five requested checks do **not** find a reward reimplementation, stale
contract, shaping-gamma mismatch, action-delta mismatch, terminal-reward
mismatch, or PPO-configuration mismatch that explains the observed initial
training returns (`bcod-sim` roughly -1100, Stonefish roughly -200, HoloOcean
roughly -100 to -200).

On seed 10000 with one identical 1,200-control-step action trace, bcod-sim and
Stonefish produced close component totals and close aggregate returns:

| Simulator | Progress | Cross-track | Action delta | Terminal | Shaping | Total |
|---|---:|---:|---:|---:|---:|---:|
| bcod-sim | -25.285 | -401.840 | -0.100 | -10.000 | -0.395 | -437.619 |
| Stonefish | -25.748 | -406.453 | -0.100 | -10.000 | -0.984 | -443.284 |
| HoloOcean | +2.252 | -25.331 | -0.100 | -10.000 | +0.170 | -33.008 |

This rules out a multiplicative or gamma-dependent reward-port error between
bcod-sim and Stonefish. Their matched-action totals differ by only 5.664 return
points (1.3% of the bcod-sim magnitude), not the approximately 900-point gap
seen during training.

The training-return divergence therefore remains **unexplained by Tasks 1-5**.
The evidence localizes the remaining question to simulator-dependent policy
trajectories, especially accumulated cross-track error, rather than the reward
formula itself. The existing PPO monitor files contain only aggregate episode
returns, so the component responsible for the original training episodes
cannot be proven retrospectively.

## Task 1 — reward implementation sharing

All three harnesses execute `packages/python-client/bcod_sim/common_task.py`'s
literal `compute_reward` source. The runtime SHA-256 of the function source was
identical for all three:

`6faa7ab684ef1750808bc6e3e94fd9aafd8182053f0e16e12cecb98573003fe9`

- bcod-sim imports `bcod_sim.common_task.compute_reward`.
- HoloOcean imports `bcod_sim.common_task.compute_reward` and resolves to the
  same Python function object as bcod-sim.
- Stonefish inserts the directory containing the shared module and imports it
  as top-level `common_task`. This creates a distinct Python module identity,
  but its resolved source file and function-source hash are exactly the shared
  file. There is no local or partial reward reimplementation.

Result: **PASS — no local reward reimplementation.**

## Task 2 — contract hash and observation size

Every runtime harness asserted:

- contract SHA-256:
  `2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36`
- observation size: 15 fields

The instantiated environments also returned 15-field observations during the
matched trace.

Result: **PASS — no stale 17-field or older contract.**

## Task 3 — shaping gamma in actual use

The frozen contract supplied `gamma_shaping = 1.0`. Each instantiated runtime
read that value and passed it into the shared `compute_reward` call. HoloOcean
and Stonefish additionally refuse construction if the value is not exactly
1.0. The per-step trace confirms pure potential-difference behavior rather than
the former PPO-discount-coupled formula.

Result: **PASS — `gamma_shaping = 1.0` uniformly.**

## Task 4 — matched-seed component trace

Protocol:

- environment seed: 10000
- maximum length: 1,200 control steps
- identical action at every step in every simulator:
  `a0 = 0.55 sin(0.071t)`, `a1 = 0.55 sin(0.071t + 0.83)`
- full reward components recorded at every step

All three timed out at step 1,200. Terminal reward was exactly zero before the
last step and exactly -10 at timeout. Action-delta totals were bit-for-bit equal
at `-0.09970771811904329`, confirming that action-delta reward uses normalized
policy actions rather than backend-specific thrust units.

The initial distance to the first waypoint was 27.1391 m for bcod-sim and
Stonefish. HoloOcean reported 26.5925 m because its initial native GPS position
did not exactly equal the requested spawn point. Step-one total rewards were
small and close: -0.00829, -0.00982, and -0.00799 respectively. There was no
step-zero/step-one reward-scale discontinuity.

The large full-episode HoloOcean difference is almost entirely cross-track:
mean cross-track was 1.055 m, versus 16.743 m in bcod-sim and 16.936 m in
Stonefish under the matched action sequence. That is a plant/trajectory
difference, not a reward computation difference.

Result: **PASS for reward-component wiring; simulator trajectories differ.**

## Task 5 — shared PPO configuration

The three backends call the same `algorithm_config()` function. Canonical
configuration SHA-256 was identical:

`a088eed28d5e8c1d8e8c32bbe393b5846c9bffc23f3c0e8b9367ffb362e5ef8c`

The shared configuration is RecurrentPPO/MlpLstmPolicy, seed 7319, `n_steps`
512, batch size 512, 10 epochs, learning rate 0.0003, PPO gamma 0.99,
GAE lambda 0.95, clip range 0.2, `[128,128]` network, LSTM size 128 with critic
LSTM enabled, and entropy coefficient 0.005 from the preregistered protocol.

Result: **PASS — no backend-specific config copy or drift.**

## Additional harness differences exposed

These differences are real but did not produce a large bcod-sim/Stonefish gap
under the matched action trace:

1. bcod-sim uses `Mulberry32` for reset sampling; HoloOcean and Stonefish use
   NumPy's generator. Seed 10000 therefore does not identify the same route,
   heading, or disturbance realization across all backends.
2. bcod-sim applied seeded current and wind. Stonefish reported both as zero.
   HoloOcean applied sampled current but ran with wind mode off.
3. HoloOcean uses a performance-calibrated 501.133 N thrust ceiling and a
   materially different native plant. Reward action-delta nevertheless remains
   in normalized policy-action space and matched exactly.

These prevent the numeric seed from being a literal common physical scenario.
They are relevant comparison-protocol differences, but this audit does not
attribute the original roughly -1100 versus -200 training gap to them because
bcod-sim and Stonefish remained close when driven by identical actions.

## Evidence

The machine-readable artifact contains the complete 3,600-step trace, runtime
source routing, scenario data, component totals, harness assertions, and PPO
configuration hashes:

`artifacts/rl-campaign/cross-simulator-reward-audit.json`
