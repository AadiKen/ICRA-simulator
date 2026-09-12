#!/usr/bin/env python3
"""Build evidence input from 6-hourly US_west 3-D nowcasts and matching HFR times."""
import argparse,glob,hashlib,json,math
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
import xarray as xr
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def stamp(value):return np.datetime_as_string(value,unit='s')+'Z'
def clean(value):
 value=float(value);return value if math.isfinite(value) else None
def main():
 p=argparse.ArgumentParser();p.add_argument('--rtofs-glob',default='.cache/environmental-validation/rtofs-3dz-us-west-20260713-15/rtofs.*_n*.nc');p.add_argument('--radar',type=Path,default=Path('.cache/environmental-validation/ucsdHfrW6-2026071300-2026071523.nc'));p.add_argument('--mask',type=Path,default=Path('artifacts/environmental-validation/rtofs-mask-sf.json'));p.add_argument('--out',type=Path,default=Path('artifacts/environmental-validation/input-3dz-20260713-15.json'));a=p.parse_args();files=[]
 for name in sorted(glob.glob(a.rtofs_glob)):
  path=Path(name);d=xr.open_dataset(path);t=np.asarray(d.MT.values).reshape(-1)[0]
  if np.datetime64('2026-07-13T00')<=t<=np.datetime64('2026-07-15T23'):files.append((path,d,t))
  else:d.close()
 if len(files)!=12:raise RuntimeError(f'Expected 12 in-window 6-hourly fields, found {len(files)}')
 depths={float(d.Depth.values[0]) for _,d,_ in files}
 if len(depths)!=1:raise RuntimeError(f'Inconsistent shallowest depths: {depths}')
 radar=xr.open_dataset(a.radar);mask=json.loads(a.mask.read_text());mask_cells=next(x['cells'] for x in mask['products'] if x['dataset_id']=='ucsdHfrW6');zone={(round(x['latitude_deg'],5),round(x['longitude_deg'],5)):(x['rtofs_mask'],x['zone']) for x in mask_cells};radar_times={str(np.datetime64(t,'s')):i for i,t in enumerate(radar.time.values)};radar_rows=[];model_rows=[]
 for path,d,t in files:
  key=str(np.datetime64(t,'s'));ti=radar_times[key];lat=np.asarray(d.Latitude.values);lon=np.asarray(d.Longitude.values);lon=np.where(lon>180,lon-360,lon);region=(lat>=36.4)&(lat<=39.3)&(lon>=-124.3)&(lon<=-120.7);idx=np.argwhere(region);rlat=lat[region];rlon=lon[region];u=np.asarray(d.u.isel(MT=0,Depth=0).values);v=np.asarray(d.v.isel(MT=0,Depth=0).values);seen=set()
  for yi,la in enumerate(radar.latitude.values):
   for xi,lo in enumerate(radar.longitude.values):
    uv=(clean(radar.water_u.values[ti,yi,xi]),clean(radar.water_v.values[ti,yi,xi]));mask_zone=zone.get((round(float(la),5),round(float(lo),5)),('land','inside_mask'));row={'site':'san-francisco','latitude_deg':float(la),'longitude_deg':float(lo),'time':stamp(t),'u_east_mps':uv[0],'v_north_mps':uv[1],'qc_pass':uv[0] is not None and uv[1] is not None,'dop_x':clean(radar.DOPx.values[ti,yi,xi]),'dop_y':clean(radar.DOPy.values[ti,yi,xi]),'gdop':clean(radar.hdop.values[ti,yi,xi]),'number_of_sites':clean(radar.number_of_sites.values[ti,yi,xi]),'number_of_radials':clean(radar.number_of_radials.values[ti,yi,xi]),'rtofs_mask':mask_zone[0],'zone':mask_zone[1]};radar_rows.append(row)
    if mask_zone[0]=='land':continue
    dist=(rlat-la)**2+((rlon-lo)*math.cos(math.radians(float(la))))**2;k=int(np.argmin(dist));y,x=idx[k];grid=f'rtofs:Y{y},X{x}'
    if grid in seen:continue
    seen.add(grid);mu,mv=float(u[y,x]),float(v[y,x])
    if math.isfinite(mu) and math.isfinite(mv):model_rows.append({'site':'san-francisco','latitude_deg':float(lat[y,x]),'longitude_deg':float(lon[y,x]),'time':stamp(t),'u_east_mps':mu,'v_north_mps':mv,'qc_pass':True,'grid_id':grid})
 radar.close();sources=[]
 for path,d,t in files:
  name=path.name;day=name.split('.')[1].split('_')[0];lead=name.split('_n')[1].split('.')[0];object_key=f'rtofs.{day}/rtofs_glo_3dz_n{lead}_6hrly_hvr_US_west.nc';sources.append({'id':object_key,'version':'RTOFS v2.3 archived 3-D nowcast','url':'https://noaa-nws-rtofs-pds.s3.amazonaws.com/'+object_key,'checksum_sha256':sha(path),'retrieved_at':datetime.fromtimestamp(path.stat().st_mtime,timezone.utc).isoformat().replace('+00:00','Z'),'native_grid':'US_west; Depth index 0'});d.close()
 query_url='https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?product=predictions&application=bcod-sim&begin_date=20260713&end_date=20260715&datum=MLLW&station=9414290&time_zone=gmt&units=metric&interval=h&format=json';tide=Path('artifacts/environmental-validation/coops-9414290-20260713-20260715.json');sources.extend([{'id':'ucsdHfrW6','version':'v1r0','url':'https://coastwatch.pfeg.noaa.gov/erddap/griddap/ucsdHfrW6.nc','checksum_sha256':sha(a.radar),'retrieved_at':datetime.fromtimestamp(a.radar.stat().st_mtime,timezone.utc).isoformat().replace('+00:00','Z')},{'id':'CO-OPS-9414290-hourly-predictions','version':'query parameters pinned','url':query_url,'checksum_sha256':sha(tide),'retrieved_at':datetime.fromtimestamp(tide.stat().st_mtime,timezone.utc).isoformat().replace('+00:00','Z')}]);out={'schema_version':1,'evidence':True,'retrieved_at':datetime.now(timezone.utc).isoformat().replace('+00:00','Z'),'sources':sources,'window_selection':{'site':'San Francisco','station':'9414290','start':'2026-07-13T00:00:00Z','stop':'2026-07-15T23:00:00Z','rationale':'Maximum predicted tidal range among three-day windows available in the recent RTOFS archive history. CO-OPS hourly predictions give 2.690 m range for this window.','coops_query_url':query_url,'model_cadence_hours':6,'model_times':12,'original_hourly_plan_times':72,'statistical_power_limitation':'Only 12 RTOFS timestamps are available, one sixth of the 72 timestamps in the original hourly plan.'},'currents':{'rtofs':model_rows,'hf_radar':radar_rows,'distance_tolerance_m':12000,'time_tolerance_s':0,'max_dop_x':1.5,'max_dop_y':1.5,'min_sites':2,'min_radials':3,'speed_bins_mps':[.25,.75],'zones':['shelf','nearshore','inside_mask'],'depths':{'hf_radar_effective_m':2.4,'rtofs_shallowest_z_m':next(iter(depths)),'disclosure':'HF radar effective depth is approximately 2.4 m; the RTOFS proxy is the 0.0 m z level.'},'independence':{'hf_radar_assimilated':False,'source':'https://www.emc.ncep.noaa.gov/users/verification/ocean_lake/rtofs/prod/main.php','checked_at':'2026-09-09T00:00:00Z','rtofs_version':'2.3'}}};a.out.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'output':str(a.out),'model_times':len(files),'model_rows':len(model_rows),'radar_rows':len(radar_rows),'shallowest_z_m':next(iter(depths))},indent=2))
if __name__=='__main__':main()
