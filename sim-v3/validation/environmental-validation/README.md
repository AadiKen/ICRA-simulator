# Environmental product-vs-reference validation

This directory implements a configuration-driven RTOFS surface-current comparison against independent IOOS HF-radar observations. It deliberately does not turn adapter fixtures into physical-accuracy evidence. The existing `environment-coverage` campaign remains a Type 1 pass-through/integration result.

## Scope gates

- Water level is excluded: `CoopsSource` requests the CO-OPS `water_level` product directly; bcod-sim does not compute tides from harmonic constituents.
- Waves and weather are Type 1 only. Wind and ocean temperature are omitted because their proposed references are assimilated into the products.
- A current report labeled `evidence` is rejected unless the input records a URL-backed, RTOFS-version-pinned finding that HF-radar currents are not assimilated.
- A generated fixture cannot be labeled `evidence`.

The operational RTOFS v2.3 observation inventory published by NOAA lists SST, SSS, temperature/salinity profiles, absolute dynamic topography, and sea-ice concentration; it does not list surface-current or HF-radar assimilation. Record that NOAA page, the checked date, and the exact operational version in `currents.independence`. This gate must be revisited when the operational version changes.

## Normalized input and run

Prepare one JSON object matching the exported `Input` shape in `run.ts`. Every source requires a version, retrieval URL, and SHA-256 checksum. Set `currents.speed_bins_mps` to change magnitude strata without changing code. Assign `zone` (for example `offshore` or `nearshore`) to each HF-radar row; missing assignments are retained in an explicit `unclassified` stratum. Set matching distance/time tolerances and the maximum accepted GDOP in the input.

HF-radar rows failing QC, exceeding GDOP, containing invalid vectors/times, or lacking an in-tolerance RTOFS neighbor are counted by reason. Each retained radar cell-hour maps to its nearest RTOFS cell/time. RTOFS cells are intentionally reusable because its roughly 1/12-degree grid is coarser than the radar grid. The report therefore warns that matched cell-hours are spatially correlated and does not manufacture independent-sample confidence intervals.

The current statistics include component and speed bias/RMSE/MAE, circular direction mean error and MAE, and complex-correlation magnitude and phase. Zero-length vectors are excluded only from direction statistics and the direction-specific N is reported.

```sh
npm run preflight:environment-currents
npm run collect:rtofs-window -- 20260908 2026-09-09T01:00:00Z 2026-09-12T00:00:00Z
npm run acquire:environment-currents -- validation/environmental-validation/config.example.json
npm run validate:environment-products -- validation/environmental-validation/input.json artifacts/environmental-validation/report.json
npm run render:environment-currents -- artifacts/environmental-validation/report.json
npm run test:environment-products
```

The pre-flight report is written to `artifacts/environmental-validation/preflight.json`. It checks the live ERDDAP metadata/variables, observes the current NOMADS retention range, queries CO-OPS predictions, ranks feasible windows, and recommends a forward-collection window when the retained historical overlap is too short.

`collect:rtofs-window` caches expiring global RTOFS files and writes a checksum manifest. Its default three-hour interval requires about 24 files for a three-day window. At the currently observed file size this is roughly 3.7 GB; hourly collection would be roughly 11 GB. It is intentionally never started by a test or pre-flight command. After collection, copy the manifest's file paths into a private copy of `config.example.json`; do not commit the large NetCDF files.

`acquire:environment-currents` downloads the full HF-radar window in one ERDDAP subset request, verifies and records checksums, reads the local RTOFS files, derives the operational mask at each radar centroid, declares `inside_mask` before scoring, and emits normalized input. Water cells adjacent to the observed mask boundary are classified as `nearshore` using the configured distance; remaining water cells are `shelf`.

Current results are stratified by site, zone, and configurable reference-speed bins, with the overall matched count and all rejection counts. The report explicitly discloses the HF-radar/RTOFS effective-depth mismatch.

Matching is deterministic and many-radar-to-one-model. Distance and time tolerances are mandatory. The input is checksummed into the output; raw dataset checksums and retrieval URLs are retained verbatim.

Only set `evidence: true` for real downloaded data. A short-window claim must name the site and dates and explicitly state that it establishes neither temporal generality nor accuracy for bathymetry, wind, waves, water level, or temperature.

## Access and prerequisites

All three upstream services used here—NOAA NOMADS RTOFS, NOAA CoastWatch ERDDAP/IOOS HF radar, and NOAA CO-OPS predictions—are open. No account or API key is required. The repository's existing `.venv` already contains xarray, netCDF4, NumPy, and Matplotlib. The only material local prerequisite is disk space for the RTOFS cache and enough uninterrupted download time before the rolling files expire.

The live pre-flight on 2026-09-09 observed only two NOMADS run directories and therefore found no complete historical three-day overlap. It recommended forward collection for 2026-09-09T01:00Z through 2026-09-12T00:00Z based on the largest predicted tidal range inside the available forecast horizon. Re-run pre-flight immediately before collection because both endpoints are rolling.
