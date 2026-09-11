from __future__ import annotations
import json,math
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from .common import ROOT,require_files
from .fig2_environmental_grounding import wms

SITES=[("san-francisco","San Francisco","46026","9414290",497),("honolulu","Honolulu","51202","1612340",None),("miami","Miami","42095","8723214",None),("boston","Boston","44013","8443970",None)]
DEFAULTS=[ROOT/"artifacts/environmental-validation/report-3dz-20260713-15.json",ROOT/"artifacts/environmental-validation/era5-ndbc-wind-20260713-15.json",ROOT/"artifacts/environmental-validation/era5-ndbc-wind-boston-20260713-15.json",ROOT/"artifacts/environmental-validation/rtofs-mask-sf.json",ROOT/"artifacts/environmental-validation/rtofs-mask-honolulu.json",ROOT/"artifacts/environmental-validation/rtofs-mask-miami.json",ROOT/"artifacts/environmental-validation/rtofs-mask-boston.json",ROOT/"artifacts/environment-coverage/enc-catzoc-remaining-sites.json"]

def validate_inputs(current,sf_wind,boston_wind,masks,enc):
    if len(current.get("currents",{}).get("matches",[]))!=2595: raise ValueError("Figure 2 requires the retained current matches")
    if any(len(report.get("wind",{}).get("matches",[]))!=72 for report in (sf_wind,boston_wind)): raise ValueError("Figure 2 requires both 72-hour wind series")
    if set(masks)!={x[0] for x in SITES}: raise ValueError("Figure 2 requires four RTOFS masks")
    if set(enc)!={"honolulu","miami","boston"}: raise ValueError("Figure 2 requires three remaining-site ENC results")

def _scatter(ax,rows,title,color,source):
    observed=[row["reference_speed_mps"] for row in rows];modeled=[math.hypot(row["model_u"],row["model_v"]) for row in rows];limit=max(observed+modeled)*1.04
    ax.scatter(observed,modeled,s=11,alpha=.45,color=color,edgecolors="none");ax.plot([0,limit],[0,limit],"--",color="#667085",lw=1);ax.set(xlim=(0,limit),ylim=(0,limit),title=title,xlabel=f"Observed {source} speed (m/s)",ylabel="Modeled speed (m/s)");ax.grid(alpha=.2)

def _current_scatter(ax,rows):
    for zone,marker,color in (("shelf","o","#2457a7"),("nearshore","s","#667085")):
        selected=[row for row in rows if row["zone"]==zone];high=max(row["reference_speed_mps"] for row in selected);edges=np.linspace(0,high,21);points=[]
        for low,upper in zip(edges[:-1],edges[1:]):
            bucket=[row for row in selected if low<=row["reference_speed_mps"]<upper+(1e-12 if upper==high else 0)]
            if bucket: points.append((np.mean([r["reference_speed_mps"] for r in bucket]),np.mean([math.hypot(r["model_u"],r["model_v"]) for r in bucket])))
        ax.scatter([p[0] for p in points],[p[1] for p in points],marker=marker,s=38,color=color,label=zone.title())
    limit=.75;ax.plot([0,limit],[0,limit],"--",color="#667085",lw=1);ax.set(xlim=(0,limit),ylim=(0,limit),title="San Francisco current",xlabel="Observed HF-radar speed (m/s)",ylabel="Modeled speed (m/s)");ax.grid(alpha=.2);ax.legend(frameon=False,fontsize=8)

def build(data_paths=None)->Path:
    paths=require_files([Path(path) for path in (data_paths or DEFAULTS)]);current,sf_wind,boston_wind=[json.loads(path.read_text()) for path in paths[:3]]
    masks={site:json.loads(path.read_text()) for site,path in zip(("san-francisco","honolulu","miami","boston"),paths[3:7])};enc={row["site"]:row for row in json.loads(paths[7].read_text())["results"]};validate_inputs(current,sf_wind,boston_wind,masks,enc)
    fig=plt.figure(figsize=(14,9),layout="constrained");outer=fig.add_gridspec(1,2,width_ratios=(1.1,1));coverage=outer[0].subgridspec(2,2)
    for index,(site,label,ndbc,coops,sf_count) in enumerate(SITES):
        ax=fig.add_subplot(coverage[index//2,index%2]);artifact=masks[site];cells=artifact.get("cells") or artifact["products"][0]["cells"];wet=[r for r in cells if r["rtofs_mask"]=="water"];land=[r for r in cells if r["rtofs_mask"]=="land"]
        ax.scatter([r["longitude_deg"] for r in land],[r["latitude_deg"] for r in land],s=7,color="#d8c6a0",label="RTOFS land");ax.scatter([r["longitude_deg"] for r in wet],[r["latitude_deg"] for r in wet],s=7,color="#9ecae1",label="RTOFS wet")
        center_lat=artifact.get("site_location",{}).get("latitude_deg",np.mean([r["latitude_deg"] for r in cells]));center_lon=artifact.get("site_location",{}).get("longitude_deg",np.mean([r["longitude_deg"] for r in cells]))
        image,extent=wms(center_lat,center_lon);ax.imshow(image,extent=extent,origin="upper",alpha=.62)
        ax.scatter(center_lon,center_lat,marker="^",s=55,color="#2457a7",edgecolor="white",label=f"NDBC {ndbc}");ax.scatter(center_lon+.025,center_lat-.025,marker="s",s=42,color="#2f855a",edgecolor="white",label=f"CO-OPS {coops}")
        count=sf_count if site=="san-francisco" else enc[site]["obstacle_count"];fraction=1 if site=="san-francisco" else enc[site]["catzoc_coverage_fraction"]
        ax.text(.02,.98,f"{count:,} obstacles\nCATZOC {fraction:.0%}",transform=ax.transAxes,va="top",fontsize=8,bbox={"facecolor":"white","alpha":.85,"edgecolor":"#667085"});ax.set_title(label,fontweight="bold");ax.tick_params(labelsize=7)
        if index==0: ax.legend(fontsize=6,loc="lower left")
    accuracy=outer[1].subgridspec(3,1);_current_scatter(fig.add_subplot(accuracy[0]),current["currents"]["matches"]);_scatter(fig.add_subplot(accuracy[1]),sf_wind["wind"]["matches"],"San Francisco wind","#2f855a","NDBC");_scatter(fig.add_subplot(accuracy[2]),boston_wind["wind"]["matches"],"Boston wind","#d97706","NDBC")
    fig.suptitle("Environmental coverage and accuracy",fontsize=17,fontweight="bold")
    output=ROOT/"artifacts/figures/fig2_geography.png";output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=220);plt.close(fig);return output
