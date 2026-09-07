"""Load locally retained GEBCO grids and Natural Earth coastline data."""
from pathlib import Path
import numpy as np

def load_gebco(path):
    path=Path(path)
    if path.suffix==".txt":
        lines=path.read_text().splitlines()
        def values_after(label):
            i=next(i for i,line in enumerate(lines) if line.startswith(label));return np.fromstring(lines[i+1],sep=",")
        lon=values_after("lon[");lat=values_after("lat[")
        start=next(i for i,line in enumerate(lines) if line.startswith("elevation.elevation["))+1
        rows=[]
        for line in lines[start:start+len(lat)]:
            rows.append(np.fromstring(line.split("]",1)[1].lstrip(", "),sep=","))
        elevation=np.vstack(rows)
        if elevation.shape!=(len(lat),len(lon)): raise ValueError(f"Malformed GEBCO ASCII grid {path}: {elevation.shape}")
        return lon,lat,elevation
    if path.suffix==".npz":
        with np.load(path) as data: return data["lon"],data["lat"],data["elevation"]
    from netCDF4 import Dataset
    with Dataset(path) as ds:
        return np.asarray(ds["lon"][:]),np.asarray(ds["lat"][:]),np.asarray(ds["elevation"][:])

def natural_earth_shapefile(root, layer):
    matches=list(Path(root).rglob(f"ne_10m_{layer}.shp"))
    if not matches: raise FileNotFoundError(f"Natural Earth {layer} data missing under {root}")
    return matches[0]

def add_coastline(ax, root, ccrs, shpreader, colors):
    land=shpreader.Reader(natural_earth_shapefile(root,"land"))
    ax.add_geometries(land.geometries(),ccrs.PlateCarree(),facecolor=colors["land"],
                      edgecolor=colors["ink"],linewidth=.35,zorder=3)
