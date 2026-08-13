# BCOD-Sim pre-ICRA 2027 baseline

Generated on 2026-07-31 before implementation changes.

## Source and tools

- BCOD repository commit before this report: `8c2421cd4d4f438bdba89f66893c2b217c2fc174`
- Branch at capture: `new-demo-site`
- Node.js: `v24.13.0`
- npm: `11.6.2`
- Python: `3.14.6`
- MSS repository: `https://github.com/cybergalactic/MSS`
- MSS commit: `c660120aa7ea16d0022064bd759d12a934ec4f76`
- MSS model: Otter USV

## Commands and status

| Command | Status |
| --- | --- |
| `npm test` | Passed |
| `npm run test:mss-acceptance` | Passed |

## MSS acceptance tolerances

All maneuvers run for 60 seconds. Acceptance requires:

- position RMSE <= 0.25 m
- heading RMSE <= 2 degrees
- body-speed RMSE <= 0.1 m/s

## MSS comparison metrics

| Maneuver | Samples | Position RMSE (m) | Heading RMSE (deg) | Body-speed RMSE (m/s) | Max position error (m) | Max heading error (deg) | Max body-speed error (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| constant-thrust | 1200 | 0.006964850956 | 0 | 0.000281931869 | 0.012224360151 | 0 | 0.003535859020 |
| coast-down | 1200 | 0.000376111923 | 0 | 0.000275293057 | 0.000832861087 | 0 | 0.003631357376 |
| turning-circle | 1200 | 0.001937112766 | 0.014616624668 | 0.000301591132 | 0.003004314537 | 0.026417815236 | 0.003837099759 |
| zig-zag | 1200 | 0.007063424048 | 0.001510705878 | 0.000331751512 | 0.012400381451 | 0.003407482875 | 0.003540702921 |
| current-drift | 1200 | 0.000065544174 | 0 | 0.000074098202 | 0.000223765897 | 0 | 0.001381426720 |

## Golden-trace SHA-256 checksums

| Trace | SHA-256 |
| --- | --- |
| `coast-down.csv` | `8267776265a7a7a3372ea6b90a85b515d727fe78f3bae9269b95ae13eecf0914` |
| `constant-thrust.csv` | `8ebb1c187ac7fb671ea14f4824c1606abe479999b403c1ac92d0834dc70ecef9` |
| `current-drift.csv` | `e869f4222f740f622119568e545aaac8a79113e62fe0cea2d537789bbaf37fe3` |
| `turning-circle.csv` | `536b175acede88d02a8f3e0265a8acf3d418c8b06ef5c94a6e7b785661c48c3e` |
| `zig-zag.csv` | `2d03a607593415e067132c0750ff3aef49bcb4e3c956794cd11890848bd6c3b8` |

The local `mssReference.py` implementation is convenience tooling and is not treated as an independent validation oracle.
