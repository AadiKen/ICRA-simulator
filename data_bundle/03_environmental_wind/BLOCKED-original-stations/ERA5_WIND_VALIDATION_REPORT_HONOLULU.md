# ERA5 versus NDBC wind validation — Honolulu

## Scope

ERA5 hourly 10 m wind vectors were compared with NDBC station 51202 observations from
2026-07-13 00:00 UTC through 2026-07-15 23:00 UTC. Each QC-passing NDBC observation exactly
on the hour was paired with the nearest ERA5 grid cell at the identical timestamp. The 0
sub-hourly station rows were excluded explicitly so that an ERA5 hour was not reused.

Hourly samples are serially correlated, so the report does not claim that the matched hours are
independent observations or provide an independent-sample confidence interval. GDOP is not
applicable to a single-station wind instrument.

## Results

No accuracy statistics can be reported because NDBC station 51202 supplied zero
usable wind vectors in the requested window. Its 142 candidate records all
failed wind QC (the archive uses missing-value sentinels for wind direction and speed). This is a
source-data limitation, not an ERA5 accuracy result; a wind-reporting reference station must be
selected before this site can support the requested comparison.


The machine-readable report, including all matched vectors, rejection counts, source URLs,
and source checksums, is `artifacts/environmental-validation/era5-ndbc-wind-honolulu-20260713-15.json`.
