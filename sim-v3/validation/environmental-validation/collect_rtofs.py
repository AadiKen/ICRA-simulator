#!/usr/bin/env python3
"""Cache the expiring RTOFS forecast files for a selected forward window."""
import argparse,hashlib,json,urllib.request
from datetime import datetime,timezone
from pathlib import Path
def sha(path):
 h=hashlib.sha256()
 with path.open("rb") as f:
  for chunk in iter(lambda:f.read(1024*1024),b""):h.update(chunk)
 return h.hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument("cycle",help="YYYYMMDD");p.add_argument("start",help="UTC ISO time");p.add_argument("stop",help="UTC ISO time");p.add_argument("--interval-hours",type=int,default=3);p.add_argument("--out",type=Path,default=Path(".cache/environmental-validation/rtofs"));a=p.parse_args();cycle=datetime.strptime(a.cycle,"%Y%m%d").replace(tzinfo=timezone.utc);start=datetime.fromisoformat(a.start.replace("Z","+00:00"));stop=datetime.fromisoformat(a.stop.replace("Z","+00:00"));first=max(0,round((start-cycle).total_seconds()/3600));last=min(192,round((stop-cycle).total_seconds()/3600));a.out.mkdir(parents=True,exist_ok=True);rows=[]
 for lead in range(first,last+1,a.interval_hours):
  name=f"rtofs_glo_2ds_f{lead:03d}_prog.nc";url=f"https://nomads.ncep.noaa.gov/pub/data/nccf/com/rtofs/prod/rtofs.{a.cycle}/{name}";path=a.out/name
  if not path.exists():
   req=urllib.request.Request(url,headers={"User-Agent":"bcod-sim environmental validation"})
   with urllib.request.urlopen(req,timeout=300) as response,path.open("wb") as out:
    while chunk:=response.read(1024*1024):out.write(chunk)
  rows.append({"path":str(path),"url":url,"bytes":path.stat().st_size,"checksum_sha256":sha(path)})
 manifest=a.out/"manifest.json";manifest.write_text(json.dumps({"cycle":a.cycle,"interval_hours":a.interval_hours,"files":rows},indent=2)+"\n");print(json.dumps({"manifest":str(manifest),"files":len(rows),"total_bytes":sum(x["bytes"] for x in rows)},indent=2))
if __name__=="__main__":main()
