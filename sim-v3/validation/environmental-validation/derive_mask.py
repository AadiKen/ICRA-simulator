#!/usr/bin/env python3
"""Derive SF regimes from an RTOFS diagnostic grid before scoring currents."""
import argparse,hashlib,json,math
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
import xarray as xr
def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('rtofs',type=Path);p.add_argument('radar',nargs='+',type=Path);p.add_argument('--object-key',required=True);p.add_argument('--source-url',required=True);p.add_argument('--nearshore-m',type=float,default=15000);p.add_argument('--out',type=Path,default=Path('artifacts/environmental-validation/rtofs-mask-sf.json'));a=p.parse_args();r=xr.open_dataset(a.rtofs);lat=np.asarray(r.Latitude.values);lon=np.asarray(r.Longitude.values);lon=np.where(lon>180,lon-360,lon);field=np.asarray(r.u_barotropic_velocity.squeeze().values);region=(lat>=36.4)&(lat<=39.3)&(lon>=-124.3)&(lon<=-120.7);idx=np.argwhere(region);rlat=lat[region];rlon=lon[region];rvalid=np.isfinite(field[region]);products=[];all_cells=[]
 for path in a.radar:
  d=xr.open_dataset(path);coords=[]
  observed=np.any(np.isfinite(d.water_u.values)&np.isfinite(d.water_v.values),axis=0)
  for yi,la in enumerate(d.latitude.values):
   for xi,lo in enumerate(d.longitude.values):
    if not observed[yi,xi]:continue
    dist=(rlat-la)**2+((rlon-lo)*math.cos(math.radians(float(la))))**2;k=int(np.argmin(dist));y,x=idx[k];coords.append({'latitude_deg':float(la),'longitude_deg':float(lo),'rtofs_y':int(y),'rtofs_x':int(x),'rtofs_latitude_deg':float(lat[y,x]),'rtofs_longitude_deg':float(lon[y,x]),'rtofs_mask':'water' if rvalid[k] else 'land'})
  d.close();all_cells.extend(coords);products.append({'dataset_id':path.stem.split('-2026')[0],'source_file':str(path),'checksum_sha256':sha(path),'observed_grid_cells':len(coords),'rtofs_absent_cells':sum(x['rtofs_mask']=='land' for x in coords),'rtofs_absent_fraction':sum(x['rtofs_mask']=='land' for x in coords)/len(coords) if coords else None,'cells':coords})
 land=[x for x in all_cells if x['rtofs_mask']=='land']
 for product in products:
  for cell in product['cells']:
   if cell['rtofs_mask']=='land':cell['zone']='inside_mask';continue
   distance=min((111120*math.hypot(cell['latitude_deg']-m['latitude_deg'],math.cos(math.radians(cell['latitude_deg']))*(cell['longitude_deg']-m['longitude_deg'])) for m in land),default=math.inf);cell['distance_to_observed_mask_m']=distance;cell['zone']='nearshore' if distance<=a.nearshore_m else 'shelf'
 out={'schema_version':1,'artifact_kind':'rtofs-sf-operational-mask-and-preregistered-zones','generated_at':datetime.now(timezone.utc).isoformat().replace('+00:00','Z'),'method':'Nearest native RTOFS grid cell to every HF-radar cell observed at least once in the window. Diagnostic barotropic velocity is used only as the shared HYCOM wet/land mask, never as surface-current validation data. Water cells within the configured distance of an observed land-mask cell are nearshore; other water cells are shelf.','nearshore_mask_distance_m':a.nearshore_m,'rtofs_source':{'object_key':a.object_key,'url':a.source_url,'retrieved_at':datetime.fromtimestamp(a.rtofs.stat().st_mtime,timezone.utc).isoformat().replace('+00:00','Z'),'checksum_sha256':sha(a.rtofs)},'products':products}
 a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'output':str(a.out),'products':[{k:x[k] for k in ('dataset_id','observed_grid_cells','rtofs_absent_cells','rtofs_absent_fraction')} for x in products]},indent=2));r.close()
if __name__=='__main__':main()
