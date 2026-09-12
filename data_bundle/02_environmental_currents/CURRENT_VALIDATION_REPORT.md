# San Francisco current validation — July 13–15, 2026 UTC

## Method and scope

The comparison uses the shallowest (`Depth=0.0 m`) field from the archived regional RTOFS 3-D `US_west` nowcasts at 00, 06, 12, and 18 UTC. HF-radar observations from `ucsdHfrW6` are selected at those 12 embedded RTOFS valid times; neither source is temporally interpolated. Each QC-passing radar cell is matched to the nearest native RTOFS cell within 12 km. Multiple radar cells may map to one RTOFS cell, so cell-hours are spatially correlated.

The 12 model times are one sixth of the 72 times in the original hourly design. The large cell-hour count does not restore temporal degrees of freedom, and this three-day result does not establish temporal generality.

The window was selected as the maximum-tidal-range three-day window in the recent archive history. The pinned CO-OPS station 9414290 query contains 72 hourly predictions spanning -0.496 to 2.194 m, a 2.690 m range.

HF radar represents approximately the upper 2.4 m; the RTOFS proxy is the 0.0 m z level. Barotropic velocity is excluded from every accuracy statistic and was used only to derive the shared RTOFS wet/land mask.

## Results

There are 2,595 matched cell-hours at 12 model times. Another 217 QC-eligible radar cell-hours fall inside the RTOFS land mask and are reported only as coverage loss (7.72% of 2,812 eligible cell-hours). Of the 6,804 radar grid cell-hours sampled at the model timestamps, 3,992 have no valid radar vector. No remaining observation was rejected by the configured DOP, contributing-site, contributing-radial, spatial, or temporal thresholds.

| Metric | Bias | MAE | RMSE |
|---|---:|---:|---:|
| Eastward u (m/s) | 0.0604 | 0.1057 | 0.1349 |
| Northward v (m/s) | 0.0043 | 0.1070 | 0.1359 |
| Speed (m/s) | -0.0534 | 0.0941 | 0.1289 |

Circular direction mean error is 19.42 degrees and direction MAE is 67.15 degrees (`N=2,592`; three calm-vector pairs are undefined for direction). Mean-centered complex correlation magnitude is 0.3379 with phase 25.41 degrees.

### Zone strata

| Zone | N | Speed bias (m/s) | Speed RMSE (m/s) | Direction MAE (deg) | Complex magnitude |
|---|---:|---:|---:|---:|---:|
| Shelf | 2,072 | -0.0402 | 0.1046 | 66.65 | 0.3971 |
| Nearshore | 523 | -0.1057 | 0.1978 | 69.13 | 0.2242 |
| Inside mask | 217 eligible cell-hours | — | — | — | — |

### Baseline comparison

| Zone | RTOFS speed RMSE | Zero-current RMSE | 6-hour persistence RMSE |
|---|---:|---:|---:|
| Overall | 0.1289 m/s | 0.1909 m/s (`N=2,595`) | 0.1002 m/s (`N=2,370`) |
| Shelf | 0.1046 m/s | 0.1760 m/s (`N=2,072`) | 0.0878 m/s (`N=1,895`) |
| Nearshore | 0.1978 m/s | 0.2409 m/s (`N=523`) | 0.1390 m/s (`N=475`) |

Persistence uses the observed speed at the same radar cell one native RTOFS interval earlier.
The first model time at each cell therefore has no persistence predecessor.

The Bay-specific 500 m radar product has observations at 392 spatial cells during the window; all 392 map to land-masked RTOFS cells. These observations characterize coverage absence and are not included in agreement statistics.

### Reference-speed strata

| HF-radar speed | N | Speed bias (m/s) | Speed RMSE (m/s) | Direction MAE (deg) | Complex magnitude |
|---|---:|---:|---:|---:|---:|
| <0.25 m/s | 2,224 | -0.0243 | 0.0878 | 69.14 | 0.3477 |
| 0.25–0.75 m/s | 371 | -0.2282 | 0.2647 | 55.23 | 0.4253 |
| >=0.75 m/s | 0 | — | — | — | — |

## Interpretation boundary

These values characterize one field, one site, one tidal window, and 12 model times. They do not validate bathymetry, wind, waves, water level, or temperature. They also must not be described as an hourly RTOFS validation: the archived hourly 2-D diagnostic files contain barotropic rather than surface-layer velocity, and the required hourly surface-current objects were absent from the archive checked for this window.
