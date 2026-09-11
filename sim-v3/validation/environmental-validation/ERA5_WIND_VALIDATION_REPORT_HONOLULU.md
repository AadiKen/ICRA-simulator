# ERA5 versus NDBC wind validation — Honolulu

## Scope

ERA5 hourly 10 m wind vectors were compared with NDBC station OOUH1 observations from
2026-07-13 00:00 UTC through 2026-07-15 23:00 UTC. Each QC-passing NDBC observation exactly
on the hour was paired with the nearest ERA5 grid cell at the identical timestamp. The 639
sub-hourly station rows were excluded explicitly so that an ERA5 hour was not reused.

Hourly samples are serially correlated, so the report does not claim that the matched hours are
independent observations or provide an independent-sample confidence interval. GDOP is not
applicable to a single-station wind instrument.

## Results

| Metric | Value |
|---|---:|
| Matched hours | 72 |
| QC/temporal failures | 0 |
| Wind-speed bias | +4.637 m/s |
| Wind-speed RMSE | 5.037 m/s |
| Eastward-component bias | -4.671 m/s |
| Eastward-component RMSE | 4.911 m/s |
| Northward-component bias | -2.043 m/s |
| Northward-component RMSE | 2.490 m/s |
| Complex-correlation magnitude | 0.492 |
| Complex-correlation phase | 5.296 deg |

Match quality was 72/72, with 0 QC or temporal-match failures.
The 639 sub-hourly records are exclusions, not failed hourly matches.


The machine-readable report, including all matched vectors, rejection counts, source URLs,
and source checksums, is `artifacts/environmental-validation/era5-ndbc-wind-honolulu-20260713-15.json`.
