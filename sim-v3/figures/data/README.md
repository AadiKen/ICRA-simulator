# Figure geography data

These regenerable inputs are intentionally ignored by Git. Retrieval date: 2026-09-07 UTC.

## GEBCO 2026

Source: CEDA OPeNDAP, GEBCO 2026 ice-surface elevation, 15 arc-second grid. Each request is a 241 × 241 (approximately 1° × 1°) constraint. Run `curl -g -L --fail -o <site>.txt '<URL>'`.

| Site | Index ranges (lat, lon) | SHA-256 |
|---|---|---|
| san-francisco | `30553:30793`, `13688:13928` | `53eabb270509b49ecea80227db2f05175f9d9331a027db6743e0714013111c99` |
| honolulu | `26589:26829`, `5192:5432` | `bcd3a426660731db6e3866d78f525ff371f78652e23f2c3db6cbb08594bb7e7c` |
| miami | `27655:27895`, `23841:24081` | `b87ad5994f69ad23454a7c8f8d5653e797f7dffda254d86937e39fd706b1e973` |
| boston | `31644:31884`, `26042:26282` | `ea64f2ac008a60d68da1b01acbade725c570fd7fd0ecad2105c9477df2fa2ed2` |

URL template (substitute the two ranges from the table):

```text
https://dap.ceda.ac.uk/thredds/dodsC/bodc/gebco/global/gebco_2026/ice_surface_elevation/netcdf/GEBCO_2026.nc.ascii?lat[LAT_START:1:LAT_END],lon[LON_START:1:LON_END],elevation[LAT_START:1:LAT_END][LON_START:1:LON_END]
```

The index conversion is `round((latitude + 90) × 240)` and `round((longitude + 180) × 240)`, calibrated against the committed cell-exact confirmatory artifact. Each range extends 120 cells from its site center.

## Natural Earth 10m

```sh
curl -L --fail -o ne_10m_coastline.zip https://naturalearth.s3.amazonaws.com/10m_physical/ne_10m_coastline.zip
curl -L --fail -o ne_10m_land.zip https://naturalearth.s3.amazonaws.com/10m_physical/ne_10m_land.zip
unzip ne_10m_coastline.zip -d figures/data/naturalearth/coastline
unzip ne_10m_land.zip -d figures/data/naturalearth/land
```

Shapefile checksums: coastline `.shp` `459a4a97c09db19aadf5244026612de9d43748be27f83a360242b99f7fabb3c1`; land `.shp` `4cad3a49bc75c1a4c2f3d7efae04f2f8e63151c96764b2658effabf524331fa6`.
