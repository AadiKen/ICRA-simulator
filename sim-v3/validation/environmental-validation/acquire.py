#!/usr/bin/env python3
"""Acquire/normalize HFR data and local RTOFS files into the validation input."""
from __future__ import annotations
import argparse,hashlib,json,math,urllib.parse,urllib.request
from datetime import datetime,timedelta,timezone
from pathlib import Path
import numpy as np
import xarray as xr

VARS=("water_u","water_v","DOPx","DOPy","hdop","number_of_sites","number_of_radials")
def iso(value): return value.astimezone(timezone.utc).isoformat().replace("+00:00","Z")
def sha(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda:f.read(1024*1024),b""):h.update(chunk)
    return h.hexdigest()
def download(url,path):
    path.parent.mkdir(parents=True,exist_ok=True)
    request=urllib.request.Request(url,headers={"User-Agent":"bcod-sim environmental validation"})
    with urllib.request.urlopen(request,timeout=180) as response,path.open("wb") as out:
        while chunk:=response.read(1024*1024):out.write(chunk)
    return {"id":path.stem,"version":"server response pinned by checksum","url":url,"checksum_sha256":sha(path),"retrieved_at":iso(datetime.now(timezone.utc))}
def radar_url(cfg,start,stop):
    b=cfg["bounds"]
    d=f"[({iso(start)}):1:({iso(stop)})][({b['lat_min']}):1:({b['lat_max']})][({b['lon_min']}):1:({b['lon_max']})]"
    query=",".join(v+urllib.parse.quote(d,safe="():.-") for v in VARS)
    return f"{cfg['radar']['endpoint']}/{cfg['radar']['dataset_id']}.nc?{query}"
def prepare_model(ds,bounds):
    la=np.asarray(ds["Latitude"].values);lo=np.asarray(ds["Longitude"].values);lo=np.where(lo>180,lo-360,lo)
    pad=1.;mask=(la>=bounds["lat_min"]-pad)&(la<=bounds["lat_max"]+pad)&(lo>=bounds["lon_min"]-pad)&(lo<=bounds["lon_max"]+pad)
    indices=np.argwhere(mask)
    if not len(indices):raise RuntimeError("RTOFS file has no coordinate cells near configured bounds")
    return la,lo,indices,la[mask],lo[mask]
def nearest_model(ds,prepared,lat,lon):
    la,lo,indices,local_lat,local_lon=prepared;vals=(local_lat-lat)**2+((local_lon-lon)*math.cos(math.radians(lat)))**2;y,x=indices[int(np.nanargmin(vals))]
    u=np.asarray(ds["u_velocity"].squeeze().values)[y,x];v=np.asarray(ds["v_velocity"].squeeze().values)[y,x]
    return int(y),int(x),float(la[y,x]),float(lo[y,x]),float(u),float(v)
def load_radar(path,site):
    ds=xr.open_dataset(path);rows=[]
    for ti,t in enumerate(ds.time.values):
      for yi,lat in enumerate(ds.latitude.values):
       for xi,lon in enumerate(ds.longitude.values):
        u=float(ds.water_u.values[ti,yi,xi]);v=float(ds.water_v.values[ti,yi,xi])
        if not(math.isfinite(u)and math.isfinite(v)):continue
        get=lambda name:float(ds[name].values[ti,yi,xi]) if name in ds and np.isfinite(ds[name].values[ti,yi,xi]) else None
        rows.append({"site":site,"latitude_deg":float(lat),"longitude_deg":float(lon),"time":np.datetime_as_string(t,unit="s")+"Z","u_east_mps":u,"v_north_mps":v,"qc_pass":True,"dop_x":get("DOPx"),"dop_y":get("DOPy"),"gdop":get("hdop"),"number_of_sites":get("number_of_sites"),"number_of_radials":get("number_of_radials")})
    ds.close();return rows
def main():
    p=argparse.ArgumentParser();p.add_argument("config",type=Path);p.add_argument("--out",type=Path,default=Path("artifacts/environmental-validation/input.json"));p.add_argument("--cache",type=Path,default=Path(".cache/environmental-validation"));p.add_argument("--use-cached-radar",action="store_true");a=p.parse_args();cfg=json.loads(a.config.read_text());start=datetime.fromisoformat(cfg["window_start"].replace("Z","+00:00"));stop=start+timedelta(days=cfg["window_days"])-timedelta(hours=1);url=radar_url(cfg,start,stop);radar_path=a.cache/f"{cfg['radar']['dataset_id']}-{start:%Y%m%d%H}-{stop:%Y%m%d%H}.nc"
    radar_source={"id":cfg["radar"]["dataset_id"],"version":cfg["radar"]["version"],"url":url,"checksum_sha256":sha(radar_path),"retrieved_at":iso(datetime.now(timezone.utc))} if a.use_cached_radar else download(url,radar_path)
    radar=load_radar(radar_path,cfg["site"]);models=[];model_sources=[]
    datasets=[]
    for item in cfg["rtofs"]["files"]:
      if not isinstance(item,dict) or not item.get("path") or not str(item.get("url","")).startswith(("http://","https://")):raise ValueError("Each RTOFS file requires local path and original HTTP retrieval URL")
      path=Path(item["path"]);ds=xr.open_dataset(path);model_time=np.asarray(ds["MT"].values).reshape(-1)[0];datasets.append((path,ds,prepare_model(ds,cfg["bounds"]),model_time));model_sources.append({"id":path.stem,"version":cfg["rtofs"]["version"],"url":item["url"],"checksum_sha256":sha(path),"retrieved_at":iso(datetime.now(timezone.utc))})
    for row in radar:
      target=np.datetime64(row["time"]);path,ds,prepared,model_time=min(datasets,key=lambda item:abs((item[3]-target)/np.timedelta64(1,"s")))
      y,x,lat,lon,u,v=nearest_model(ds,prepared,row["latitude_deg"],row["longitude_deg"]);water=math.isfinite(u)and math.isfinite(v);row["rtofs_mask"]="water" if water else "land";row["zone"]="shelf" if water else "inside_mask"
      if water:models.append({"site":cfg["site"],"latitude_deg":lat,"longitude_deg":lon,"time":np.datetime_as_string(model_time,unit="s")+"Z","u_east_mps":u,"v_north_mps":v,"qc_pass":True,"zone":row["zone"],"grid_id":f"rtofs:Y{y},X{x}"})
    for _,ds,_,_ in datasets:ds.close()
    # A water cell adjacent to any masked radar cell is the predeclared nearshore regime.
    masked=[r for r in radar if r["rtofs_mask"]=="land"]
    limit=cfg["rtofs"].get("nearshore_mask_distance_m",15000)
    for row in radar:
      if row["rtofs_mask"]=="water" and masked:
       d=min(111120*math.hypot(row["latitude_deg"]-m["latitude_deg"],math.cos(math.radians(row["latitude_deg"]))*(row["longitude_deg"]-m["longitude_deg"])) for m in masked)
       if d<=limit:row["zone"]="nearshore"
    bykey={(m["time"],m["grid_id"]):m for m in models};models=list(bykey.values())
    rcfg=cfg["radar"];out={"schema_version":1,"evidence":True,"retrieved_at":iso(datetime.now(timezone.utc)),"sources":[radar_source,*model_sources],"currents":{"rtofs":models,"hf_radar":radar,**cfg["matching"],"max_dop_x":rcfg.get("max_dop_x"),"max_dop_y":rcfg.get("max_dop_y"),"min_sites":rcfg.get("min_sites"),"min_radials":rcfg.get("min_radials"),"speed_bins_mps":cfg["speed_bins_mps"],"zones":["shelf","nearshore","inside_mask"],"independence":{"hf_radar_assimilated":False,"source":cfg["independence_source"],"checked_at":iso(datetime.now(timezone.utc)),"rtofs_version":cfg["rtofs"]["version"]}}}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(out,indent=2)+"\n");print(json.dumps({"output":str(a.out),"radar_rows":len(radar),"rtofs_rows":len(models),"inside_mask":len(masked)},indent=2))
if __name__=="__main__":main()
