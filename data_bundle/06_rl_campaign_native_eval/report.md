# Four-policy native evaluation — BCOD, Gazebo, Stonefish, HoloOcean

Status: **completed native-simulator evaluation; not an analytic common-judge result**

All four deterministic feed-forward PPO policies used training seed 7319 and the same 50
held-out reset seeds, 10000–10049, under the calm `common-waypoint-transit-v1` portable
protocol. No optimizer update was performed during evaluation. Success intervals are 95%
Wilson intervals; continuous intervals use 20,000 deterministic paired/bootstrap resamples
with analysis seed 7319.

| Native training/evaluation simulator | PPO steps | Success | Success 95% CI | Median return | Bootstrap 95% CI | Mean episode steps | Failure mode |
|---|---:|---:|---:|---:|---:|---:|---|
| bcod-sim | 1,015,808 | 50/50 (100%) | 92.9–100% | 240.99 | 239.38–242.29 | 868.96 | none |
| Gazebo Harmonic | 999,424 | 50/50 (100%) | 92.9–100% | 229.00 | 222.94–230.49 | 934.24 | none |
| Stonefish | 1,015,808 | 41/50 (82%) | 69.2–90.2% | 216.21 | 211.03–223.83 | 784.24 | 9 instability terminations |
| HoloOcean | 1,007,616 | 15/50 (30%) | 19.1–43.8% | 15.37 | 7.97–34.78 | 2,368.40 | 35 timeouts |

No episode reported a collision. BCOD and Gazebo tie on success. BCOD has the higher return:
the paired mean BCOD-minus-Gazebo difference is 16.55, with bootstrap 95% CI 13.15–20.17.

## Seed-paired return comparisons

| Difference | Paired mean | Bootstrap 95% CI | Success-rate difference |
|---|---:|---:|---:|
| BCOD − Gazebo | 16.55 | 13.15–20.17 | 0 points |
| BCOD − Stonefish | 66.67 | 40.07–96.66 | 18 points |
| BCOD − HoloOcean | 258.08 | 196.66–329.92 | 70 points |
| Gazebo − Stonefish | 50.12 | 22.84–81.55 | 18 points |
| Gazebo − HoloOcean | 241.54 | 181.66–316.21 | 70 points |
| Stonefish − HoloOcean | 191.42 | 118.67–272.71 | 52 points |

All paired mean-return intervals exclude zero. For success, BCOD and Gazebo are indistinguishable
in this sample. Stonefish beats HoloOcean on 30 discordant seeds, while HoloOcean beats
Stonefish on 4; the remaining seeds have matching binary outcomes.

## Runtime observations

| Simulator | Training-chain wall time | Mean evaluation wall time per episode |
|---|---:|---:|
| bcod-sim | 0.42 h | 1.30 s |
| Stonefish | 2.07 h | 5.34 s |
| HoloOcean | 1.06 h | 13.77 s |
| Gazebo Harmonic | 17.12 h | 98.01 s |

Gazebo's training total sums 61 completed 16,384-step segments and therefore includes repeated
container/setup overhead. Treat it as observed campaign wall time, not a clean steady-state
throughput benchmark.

## Interpretation

On binary task completion, BCOD and Gazebo jointly rank first, followed by Stonefish and then
HoloOcean. Using return as the secondary ranking gives BCOD, Gazebo, Stonefish, HoloOcean.
The especially clear failure signatures are Stonefish instability and HoloOcean timeout.

This comparison does not isolate learned policy quality from simulator dynamics: every policy
was evaluated in its own training simulator. It also has approximate—not exact—matched training
budgets because PPO rollout batches ended at slightly different step counts. The stronger paper
claim must come from the new standalone analytic judge after judge-domain validation, all-arm
ceiling calibration, frozen condition contracts, and all four adapter replay gates pass.

## Checkpoint identity

- bcod-sim: `fd5d24ee9db33b67f1e2fd99f5b4c4e084a7c47dd18af6779b7fafc4ba7a4335`
- Gazebo: `4f1d1e372cc949e98c685fe1864ce07eef4162369e536c44593331448440d7e2`
- Stonefish: `93d32030c705e3034f805955ec44b4289fca49f0e0163bbaf982911862bdf12a`
- HoloOcean: `417431f744200519cd84ec25ff48085eefd69cf18f65561da8c36fef37dd8d91`
