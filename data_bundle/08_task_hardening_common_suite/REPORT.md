# Common-suite task hardening

The shared task uses a 1.0 m terminal success radius in version `1.1.0-candidate`.
No route, time/energy budget, or environmental-disturbance field was changed.

The current bcod-sim checkpoint was evaluated on seeds 10000–10009 without training:

| Radius | Successes | Rate |
|---:|---:|---:|
| 2.0 m (previous 50-episode native evaluation) | 50/50 | 100% |
| 1.5 m | 8/10 | 80% |
| **1.0 m (selected)** | **7/10** | **70%** |
| 0.75 m | 6/10 | 60% |

The selected checked-in nominal contract has content hash
`61f9b70d8d63b8f150e3c1a2889d0ef988ac7b46126370719a3ffb6f64a8a927`.
Its exact rerun produced 7/10 success, confirming that bcod-sim left the ceiling.

The matching native Gazebo checkpoint was then evaluated on the same seeds and
exact nominal contract without training. It produced 9/10 success (90%): nine
success terminations and one timeout. Under the preregistered exclusive
saturation bounds `(0.05, 0.95)`, Gazebo is non-saturated. The checkpoint hash is
`4f1d1e372cc949e98c685fe1864ce07eef4162369e536c44593331448440d7e2`.

The remaining native checks ran on the cluster with the same contract, seeds,
and no training:

| Arm | Successes | Rate | Saturation result | Failure modes |
|---|---:|---:|---|---|
| bcod-sim | 7/10 | 70% | non-saturated | 3 non-successes |
| Gazebo | 9/10 | 90% | non-saturated | 1 timeout |
| HoloOcean | **0/10** | **0%** | **floor-saturated** | 10 timeouts |
| Stonefish | 4/10 | 40% | non-saturated | 6 instabilities |

HoloOcean's 0% result triggers the required stop-and-report rule. The contracts
therefore remain `calibration-required`, not frozen, and the full multi-seed
campaign remains unauthorized. No difficulty knob was auto-adjusted. Selecting a
different portable difficulty is now a human judgment call.
