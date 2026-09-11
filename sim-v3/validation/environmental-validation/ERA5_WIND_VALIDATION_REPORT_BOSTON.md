# ERA5 versus NDBC wind validation — Boston

## Scope

ERA5 hourly 10 m wind vectors were compared with NDBC station 44013 observations from
2026-07-13 00:00 UTC through 2026-07-15 23:00 UTC. Each QC-passing NDBC observation exactly
on the hour was paired with the nearest ERA5 grid cell at the identical timestamp. The 355
sub-hourly station rows were excluded explicitly so that an ERA5 hour was not reused.

Hourly samples are serially correlated, so the report does not claim that the matched hours are
independent observations or provide an independent-sample confidence interval. GDOP is not
applicable to a single-station wind instrument.

## Results

| Metric | Value |
|---|---:|
| Matched hours | 72 |
| QC/temporal failures | 0 |
| Wind-speed bias | +0.864 m/s |
| Wind-speed RMSE | 1.376 m/s |
| Eastward-component bias | +0.663 m/s |
| Eastward-component RMSE | 1.376 m/s |
| Northward-component bias | +0.020 m/s |
| Northward-component RMSE | 1.697 m/s |
| Complex-correlation magnitude | 0.842 |
| Complex-correlation phase | -7.645 deg |

Match quality was 72/72, with 0 QC or temporal-match failures.
The 355 sub-hourly records are exclusions, not failed hourly matches.


The machine-readable report, including all matched vectors, rejection counts, source URLs,
and source checksums, is `artifacts/environmental-validation/era5-ndbc-wind-boston-20260713-15.json`.
