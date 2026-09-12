# Tier 0 + Tier 1 environmental cleanup

## Baselines

All baseline values are speed RMSE. Persistence uses the observation at the same
location one native model interval earlier; the first time at each location is excluded.

| Quantity | Model RMSE | Zero baseline | Persistence baseline |
|---|---:|---:|---:|
| SF current, overall | 0.1289 m/s | 0.1909 m/s (`N=2,595`) | 0.1002 m/s (`N=2,370`, 6 h) |
| SF current, shelf | 0.1046 m/s | 0.1760 m/s (`N=2,072`) | 0.0878 m/s (`N=1,895`, 6 h) |
| SF current, nearshore | 0.1978 m/s | 0.2409 m/s (`N=523`) | 0.1390 m/s (`N=475`, 6 h) |
| SF wind | 1.4921 m/s | 4.8298 m/s (`N=72`) | 1.3381 m/s (`N=71`, 1 h) |
| Boston wind | 1.3762 m/s | 5.6116 m/s (`N=72`) | 1.0779 m/s (`N=71`, 1 h) |

The exact machine-readable values are in
`artifacts/environmental-validation/environmental-baselines-20260713-15.json`.

## RTOFS wet masks

| Site | Native cells | Wet | Land |
|---|---:|---:|---:|
| Honolulu | 168 | 142 | 26 |
| Miami | 169 | 89 | 80 |
| Boston | 192 | 42 | 150 |

Each render-ready artifact retains native RTOFS indices, coordinates, wet/land state,
the source checksum, and extraction method.

## ENC obstacles and CATZOC coverage

The remaining-site pass used the same five NOAA ENC Direct harbour-band point classes
and obstacle-weighted quality-polygon assignment as the retained San Francisco pass.

| Site | Obstacles | CATZOC-covered | Coverage |
|---|---:|---:|---:|
| San Francisco | 497 | 497 | 100% |
| Honolulu | 348 | 348 | 100% |
| Miami | 739 | 739 | 100% |
| Boston | 1,694 | 1,694 | 100% |

## Figures

- `figures/out/publication/fig1_vessels.png` plots all six Vehicle A `planar3` maneuvers
  and all four Vehicle C `coupled6` maneuvers from full trajectory samples.
- `figures/out/publication/fig2_geography.png` combines four-site bathymetry, NDBC and
  CO-OPS markers, RTOFS wet masks, ENC/CATZOC badges, San Francisco current
  accuracy, and separate San Francisco/Boston wind accuracy panels.
- Honolulu and Miami remain coverage-only in the wind panels because their designated
  NDBC archives contain no usable wind vectors for the selected window.
