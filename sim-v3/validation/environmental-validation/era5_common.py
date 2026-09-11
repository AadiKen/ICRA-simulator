from __future__ import annotations

import hashlib
from pathlib import Path

import cdsapi
import numpy as np
import xarray as xr


DATASET = "reanalysis-era5-single-levels"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def retrieve_wind(
    output: Path,
    *,
    year: int,
    month: int,
    days: list[int],
    times: list[str],
    area: list[float],
) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)
    request = {
        "product_type": ["reanalysis"],
        "variable": ["10m_u_component_of_wind", "10m_v_component_of_wind"],
        "year": [f"{year:04d}"],
        "month": [f"{month:02d}"],
        "day": [f"{day:02d}" for day in days],
        "time": times,
        "data_format": "netcdf",
        "download_format": "unarchived",
        "area": area,
    }
    cdsapi.Client(quiet=True).retrieve(DATASET, request, str(output))
    return output


def _coordinate_name(dataset: xr.Dataset, *candidates: str) -> str:
    for name in candidates:
        if name in dataset.coords or name in dataset.dims:
            return name
    raise KeyError(f"Missing coordinate; expected one of {candidates}")


def wind_rows(path: Path, latitude: float, longitude: float) -> list[dict]:
    dataset = xr.open_dataset(path)
    try:
        lat_name = _coordinate_name(dataset, "latitude", "lat")
        lon_name = _coordinate_name(dataset, "longitude", "lon")
        time_name = _coordinate_name(dataset, "valid_time", "time")
        query_lon = longitude % 360 if float(dataset[lon_name].max()) > 180 else longitude
        cell = dataset.sel({lat_name: latitude, lon_name: query_lon}, method="nearest")
        lat = float(cell[lat_name])
        lon = float(cell[lon_name])
        if lon > 180:
            lon -= 360
        rows = []
        for index, instant in enumerate(np.asarray(cell[time_name].values).reshape(-1)):
            rows.append(
                {
                    "time": np.datetime_as_string(instant, unit="s") + "Z",
                    "latitude_deg": lat,
                    "longitude_deg": lon,
                    "u_east_mps": float(np.asarray(cell["u10"].values).reshape(-1)[index]),
                    "v_north_mps": float(np.asarray(cell["v10"].values).reshape(-1)[index]),
                }
            )
        return rows
    finally:
        dataset.close()
