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

The contracts intentionally remain `calibration-required`, not frozen. Gazebo,
HoloOcean, and Stonefish still require native nominal saturation checks before
contract freeze or the full multi-seed campaign. HoloOcean is not installed on
this host, and no local Stonefish runtime was found; those facts are not converted
into inferred results.
