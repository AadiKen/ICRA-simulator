#!/usr/bin/env python3
"""Resolve endpoint/retention checks and rank feasible 3-day windows by tide range."""
import argparse,json,re,urllib.parse,urllib.request
from datetime import datetime,timedelta,timezone
from pathlib import Path

UA={"User-Agent":"bcod-sim environmental validation contact: local research run"}
def get(url):
    with urllib.request.urlopen(urllib.request.Request(url,headers=UA),timeout=60) as r:return r.read().decode("utf-8","replace")
def rank_windows(start,end,days,station):
    params=urllib.parse.urlencode({"product":"predictions","application":"bcod-sim","begin_date":start.strftime("%Y%m%d"),"end_date":end.strftime("%Y%m%d"),"datum":"MLLW","station":station,"time_zone":"gmt","units":"metric","interval":"h","format":"json"});url=f"https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?{params}";pred=json.loads(get(url)).get("predictions",[]);values=[(datetime.strptime(x["t"],"%Y-%m-%d %H:%M").replace(tzinfo=timezone.utc),float(x["v"])) for x in pred];out=[]
    for i in range(max(0,len(values)-24*days+1)):
      window=values[i:i+24*days]
      if len(window)==24*days and window[0][0]>=start and window[-1][0]<=end:out.append({"start":window[0][0].isoformat().replace("+00:00","Z"),"stop":window[-1][0].isoformat().replace("+00:00","Z"),"predicted_tidal_range_m":max(v for _,v in window)-min(v for _,v in window)})
    out.sort(key=lambda x:(-x["predicted_tidal_range_m"],x["start"]));return url,out
def main():
    p=argparse.ArgumentParser();p.add_argument("--days",type=int,default=3);p.add_argument("--station",default="9414290");p.add_argument("--out",type=Path,default=Path("artifacts/environmental-validation/preflight.json"));a=p.parse_args();now=datetime.now(timezone.utc)
    info_url="https://coastwatch.pfeg.noaa.gov/erddap/info/ucsdHfrW6/index.json";info=json.loads(get(info_url));rows=info["table"]["rows"]
    attrs={row[2]:row[4] for row in rows if row[0]=="attribute" and row[1]=="NC_GLOBAL"};variables=[row[1] for row in rows if row[0]=="variable"]
    nomads="https://nomads.ncep.noaa.gov/pub/data/nccf/com/rtofs/prod/";listing=get(nomads);dates=sorted(set(re.findall(r"rtofs\.(\d{8})",listing)))
    feasible_start=max(datetime.fromisoformat(str(attrs["time_coverage_start"]).replace("Z","+00:00")),datetime.strptime(dates[0],"%Y%m%d").replace(tzinfo=timezone.utc)) if dates else None
    feasible_end=min(datetime.fromisoformat(str(attrs["time_coverage_end"]).replace("Z","+00:00")),datetime.strptime(dates[-1],"%Y%m%d").replace(tzinfo=timezone.utc)+timedelta(days=1)) if dates else None
    candidates=[]
    if feasible_start and feasible_end and feasible_end-feasible_start>=timedelta(days=a.days):
      tide_url,candidates=rank_windows(feasible_start,feasible_end,a.days,a.station)
    else:tide_url=None
    forward_url=None;forward=[]
    if dates:
      cycle=datetime.strptime(dates[-1],"%Y%m%d").replace(tzinfo=timezone.utc);forward_start=max(now.replace(minute=0,second=0,microsecond=0),cycle);forward_end=cycle+timedelta(hours=192);forward_url,forward=rank_windows(forward_start,forward_end,a.days,a.station)
    out={"schema_version":1,"checked_at":now.isoformat().replace("+00:00","Z"),"hf_radar":{"metadata_url":info_url,"dataset_id":attrs.get("id"),"product_version":attrs.get("product_version"),"source_url":attrs.get("sourceUrl"),"time_coverage_start":attrs.get("time_coverage_start"),"time_coverage_end":attrs.get("time_coverage_end"),"required_variables":{name:name in variables for name in ["water_u","water_v","DOPx","DOPy","hdop","number_of_sites","number_of_radials"]}},"rtofs":{"nomads_url":nomads,"available_run_dates":dates,"oldest_run":dates[0] if dates else None,"newest_run":dates[-1] if dates else None,"observed_retention_days":(datetime.strptime(dates[-1],"%Y%m%d")-datetime.strptime(dates[0],"%Y%m%d")).days+1 if dates else 0},"feasible_overlap":{"start":feasible_start.isoformat().replace("+00:00","Z") if feasible_start else None,"end":feasible_end.isoformat().replace("+00:00","Z") if feasible_end else None,"complete_window_available":bool(candidates)},"coops":{"station":a.station,"query_url":tide_url,"ranked_windows":candidates[:10],"forward_query_url":forward_url},"recommended_window":candidates[0] if candidates else None,"recommended_forward_collection_window":forward[0] if forward else None}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(out,indent=2)+"\n");print(json.dumps(out,indent=2))
if __name__=="__main__":main()
