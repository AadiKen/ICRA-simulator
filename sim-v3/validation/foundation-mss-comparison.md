# MSS comparison after foundation extraction

This checkpoint was captured after adding the typed monorepo contracts and before removing or redirecting any legacy simulator code. The legacy implementation remains authoritative until the full migration gate passes.

Both the baseline and foundation checkpoint use MSS commit `c660120aa7ea16d0022064bd759d12a934ec4f76` and the tolerances recorded in `baseline-pre-icra2027.md`.

| Maneuver | Baseline position RMSE (m) | Foundation position RMSE (m) | Delta | Baseline heading RMSE (deg) | Foundation heading RMSE (deg) | Delta | Baseline speed RMSE (m/s) | Foundation speed RMSE (m/s) | Delta |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| constant-thrust | 0.006964850956 | 0.006964850956 | 0 | 0 | 0 | 0 | 0.000281931869 | 0.000281931869 | 0 |
| coast-down | 0.000376111923 | 0.000376111923 | 0 | 0 | 0 | 0 | 0.000275293057 | 0.000275293057 | 0 |
| turning-circle | 0.001937112766 | 0.001937112766 | 0 | 0.014616624668 | 0.014616624668 | 0 | 0.000301591132 | 0.000301591132 | 0 |
| zig-zag | 0.007063424048 | 0.007063424048 | 0 | 0.001510705878 | 0.001510705878 | 0 | 0.000331751512 | 0.000331751512 | 0 |
| current-drift | 0.000065544174 | 0.000065544174 | 0 | 0 | 0 | 0 | 0.000074098202 | 0.000074098202 | 0 |

This is not the final post-migration report. Legacy removal is forbidden until a later report proves the migrated production path retains these tolerances.

