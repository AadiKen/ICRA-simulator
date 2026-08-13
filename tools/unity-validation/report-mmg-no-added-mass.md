# Vehicle B Unity comparison: MMG hull resistance, added mass excluded

- Unity editor used for this rerun: 6000.5.7f1. The pinned 2023.2.20f1
  installation could not acquire its local editor entitlement; this version
  deviation is recorded rather than silently mixed into the result.
- Added mass: disabled, unchanged from the original stable baseline.
- Planar hull resistance: production Vehicle B MMG XH/YH/NH enabled.
- Geometry: documented WAM-V substitution with propulsion applied at the CG.

- Compared points: 1000
- Duration: 19.98 s

## Per-channel RMSE

| Channel | Original | MMG corrected | Change |
|---|---:|---:|---:|
| N (m) | 9.901611 | 9.779660 | -0.121951 |
| E (m) | 5.533126 | 5.585891 | +0.052765 |
| yaw (rad) | 0.697730 | 0.787346 | +0.089616 |
| u (m/s) | 1.227111 | 1.220263 | -0.006848 |
| v (m/s) | 0.388805 | 0.384991 | -0.003814 |
| r (rad/s) | 0.085979 | 0.096501 | +0.010522 |
| Horizontal position (m) | 11.342724 | 11.262501 | -0.080223 |
| Horizontal velocity (m/s) | 1.287234 | 1.279554 | -0.007680 |

## Straight segment (0.02–4.98 s, rudder zero)

| Metric | Original | MMG corrected | Change |
|---|---:|---:|---:|
| Surge u RMSE (m/s) | 1.009 | 1.000534 | -0.008466 (-0.84%) |
| N-position RMSE (m) | 1.932 | 1.902037 | -0.029963 (-1.55%) |

The straight-line prediction is directionally confirmed, but the improvement
is too small to call meaningful. The remaining WAM-V buoyancy/drag stack and
the turning-segment rudder/geometry substitutions dominate the discrepancy.
The full-run heading and yaw-rate metrics worsen despite small improvements in
N, u, v, and aggregate position/velocity.

Yaw and r are reported in radians and radians per second, respectively.
This result is MMG-hull-resistance-corrected but added-mass-excluded; those are
independent limitations as documented in `VALIDATION_HARNESS_SPEC.md`.
