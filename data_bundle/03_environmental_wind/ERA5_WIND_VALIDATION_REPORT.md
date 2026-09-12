# ERA5 versus NDBC wind validation

## Scope

ERA5 hourly 10 m wind vectors were compared with NDBC station 46026 observations from
2026-07-13 00:00 UTC through 2026-07-15 23:00 UTC. Each QC-passing NDBC observation exactly
on the hour was paired with the nearest ERA5 grid cell at the identical timestamp. The 355
sub-hourly station rows were excluded explicitly so that an ERA5 hour was not reused.

This is a short-window, single-site validation result. Hourly samples are serially correlated,
so the report does not claim 72 independent observations or provide an independent-sample
confidence interval. GDOP is not applicable to a single-station wind instrument.

## Results

| Metric | Value |
|---|---:|
| Matched hours | 72 |
| Wind-speed bias | +0.209 m/s |
| Wind-speed RMSE | 1.492 m/s |
| Eastward-component bias | +0.890 m/s |
| Eastward-component RMSE | 1.620 m/s |
| Northward-component bias | +0.703 m/s |
| Northward-component RMSE | 1.649 m/s |
| Complex-correlation magnitude | 0.774 |
| Complex-correlation phase | 0.237 deg |

### Baseline comparison

ERA5's 1.492 m/s speed RMSE compares with 4.830 m/s for a zero-wind baseline
(`N=72`) and 1.338 m/s for one-hour observed persistence (`N=71`). Persistence
uses the preceding hourly NDBC speed; the first hour has no predecessor.

The machine-readable report, including all matched vectors, rejection counts, source URLs,
and source checksums, is `artifacts/environmental-validation/era5-ndbc-wind-20260713-15.json`.

## Retrieval-fidelity result

The refreshed four-site archive audit contains one live CDS ERA5 cell for each of San
Francisco, Honolulu, Miami, and Boston. All four decoded adapter values equal the values read
from their downloaded NetCDF source cells. ERA5 therefore passes the project-specific
archive-to-adapter fidelity check at all four sites for the selected 2026-07-15 12:00 UTC
timestamp inside the 2026-07-13 through 2026-07-15 window.

The same audit still contains zero historical NWS cells because the NWS forecast-grid adapter
has no corresponding archived forecast payload. Accordingly, the audit supports the ERA5
fidelity claim, but by itself does not support an “all six sources have historical cells” claim.
