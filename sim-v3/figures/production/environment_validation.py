"""Spatial RTOFS availability and current-speed validation."""
from __future__ import annotations
import hashlib,io,json,math,subprocess,urllib.parse,urllib.request
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from PIL import Image,ImageEnhance

ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/"artifacts/environmental-validation/report-3dz-20260713-15.json"
AUDIT=ROOT/"artifacts/environment-coverage/historical-archive-fidelity.json"
OUTPUT=ROOT/"figures/out/publication/environment-validation.png"
SITES=[("San Francisco","sf",37.806,-122.465,"46026","9414290"),("Honolulu","honolulu",21.289,-157.865,"OOUH1","1612340"),("Miami","miami",25.731,-80.162,"VAKF1","8723214"),("Boston","boston",42.354,-70.989,"44013","8443970")]
STATIONS={"sf":((37.759,-122.833),(37.806,-122.465)),"honolulu":((21.303,-157.867),(21.307,-157.867)),"miami":((25.731,-80.162),(25.732,-80.162)),"boston":((42.346,-70.651),(42.354,-71.050))}

def mask_path(key):return ROOT/f"artifacts/environmental-validation/rtofs-mask-{key}.json"
def mask_cells(path):
 data=json.loads(path.read_text());cells=data.get("cells") or data["products"][0]["cells"];unique={}
 for cell in cells:unique[(round(cell["latitude_deg"],6),round(cell["longitude_deg"],6))]=cell
 return list(unique.values())
def wms(lat,lon):
 d=.52;b=(lon-d,lat-d,lon+d,lat+d);q={"service":"WMS","version":"1.3.0","request":"GetMap","layers":"GEBCO_LATEST","styles":"","crs":"CRS:84","bbox":','.join(map(str,b)),"width":600,"height":600,"format":"image/png"};req=urllib.request.Request("https://wms.gebco.net/mapserv?"+urllib.parse.urlencode(q),headers={"User-Agent":"bcod-sim figure build"});im=Image.open(io.BytesIO(urllib.request.urlopen(req,timeout=90).read())).convert("L").convert("RGB");return ImageEnhance.Contrast(im).enhance(.65),(b[0],b[2],b[1],b[3])
def speed_rmse(rows):
 errors=[math.hypot(r["model_u"],r["model_v"])-r["reference_speed_mps"] for r in rows];return math.sqrt(sum(e*e for e in errors)/len(errors))
def bins(rows,n=20):
 hi=max(r["reference_speed_mps"] for r in rows);edges=np.linspace(0,hi,n+1);out=[]
 for i,(lo,up) in enumerate(zip(edges[:-1],edges[1:])):
  selected=[r for r in rows if lo<=r["reference_speed_mps"]<up or i==n-1 and r["reference_speed_mps"]==up]
  if selected:out.append((np.mean([r["reference_speed_mps"] for r in selected]),np.mean([math.hypot(r["model_u"],r["model_v"]) for r in selected])))
 return out
def draw_coverage(ax,site):
 name,key,lat,lon,ndbc,coops=site;im,b=wms(lat,lon);ax.imshow(im,extent=b,origin="upper",alpha=.72,zorder=0);cells=mask_cells(mask_path(key));wet=[c for c in cells if c.get("rtofs_mask")=="water"];dry=[c for c in cells if c.get("rtofs_mask")!="water"]
 for group,color,z in ((dry,"#8f969d",1),(wet,"#2f9e64",2)):
  if group:ax.scatter([c["longitude_deg"] for c in group],[c["latitude_deg"] for c in group],s=62 if key=="sf" else 86,marker="s",color=color,alpha=.58,linewidths=0,zorder=z)
 npos,cpos=STATIONS[key];ax.scatter([npos[1]],[npos[0]],marker="^",s=48,color="#2457a7",edgecolor="white",lw=.7,zorder=4);ax.scatter([cpos[1]],[cpos[0]],marker="o",s=42,color="#d97706",edgecolor="white",lw=.7,zorder=4);ax.text(.02,.03,f"NDBC {ndbc} △   CO-OPS {coops} ●",transform=ax.transAxes,fontsize=6.4,color="white",bbox={"facecolor":"#172033","alpha":.82,"pad":2},zorder=5);ax.set(xlim=b[:2],ylim=b[2:]);ax.set_title(name,fontsize=10,fontweight="bold");ax.tick_params(labelsize=6);return len(wet),len(dry)
def build(output=OUTPUT):
 rows=json.loads(REPORT.read_text())["currents"]["matches"];fig=plt.figure(figsize=(15,7.7),layout="constrained");outer=fig.add_gridspec(1,2,width_ratios=[1.28,1]);left=outer[0].subgridspec(3,2,height_ratios=[.12,1,1]);head=fig.add_subplot(left[0,:]);head.axis("off");head.text(0,.74,"A | Spatial Data Availability",fontsize=15,fontweight="bold");head.text(0,.08,"RTOFS native wet mask over muted GEBCO geography",fontsize=8,color="#667085");coverage={}
 for i,site in enumerate(SITES):coverage[site[1]]=draw_coverage(fig.add_subplot(left[1+i//2,i%2]),site)
 head.legend(handles=[Line2D([0],[0],marker="s",color="none",markerfacecolor="#2f9e64",markersize=9,label="RTOFS water value available"),Line2D([0],[0],marker="s",color="none",markerfacecolor="#8f969d",markersize=9,label="RTOFS masked / unavailable")],loc="center right",frameon=False,fontsize=7,ncol=2)
 right=outer[1].subgridspec(2,1,height_ratios=[.12,1]);title=fig.add_subplot(right[0]);title.axis("off");title.text(0,.74,"B | Current-Speed Validation",fontsize=15,fontweight="bold");title.text(0,.08,"RTOFS surface proxy vs. QC-passing HF radar",fontsize=8,color="#667085");ax=fig.add_subplot(right[1]);limit=.75;ax.fill_between([0,limit],[0,limit],[0,0],color="#d97706",alpha=.09,zorder=0)
 for zone,label,marker,face in (("shelf","Shelf","o","white"),("nearshore","Nearshore","s","#667085")):
  selected=[r for r in rows if r["zone"]==zone];points=bins(selected);ax.scatter([p[0] for p in points],[p[1] for p in points],marker=marker,facecolor=face,edgecolor="#172033",s=48,label=f"{label} · RMSE {speed_rmse(selected):.2f} m/s",zorder=3)
 ax.plot([0,limit],[0,limit],ls="--",color="#475467",lw=1.2);ax.text(.39,.08,"Below dashed line:\nmodel under-predicts speed",color="#9a5b08",fontsize=9,ha="center",bbox={"facecolor":"white","alpha":.8,"edgecolor":"none"});ax.text(.02,.98,"No QC-passing observations ≥0.75 m/s",transform=ax.transAxes,va="top",fontsize=8,color="#667085");ax.set(xlim=(0,limit),ylim=(0,limit),aspect="equal",xlabel="Observed HF-radar speed (m/s)",ylabel="RTOFS modeled speed (m/s)");ax.grid(alpha=.18);ax.legend(frameon=False,loc="upper left",bbox_to_anchor=(0,.91),fontsize=9);fig.suptitle("Environmental Validation",x=.02,ha="left",fontsize=19,fontweight="bold")
 output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=240,bbox_inches="tight",pad_inches=.08);fig.savefig(output.with_suffix(".svg"),bbox_inches="tight",pad_inches=.08);plt.close(fig);sources=[REPORT,AUDIT]+[mask_path(s[1]) for s in SITES];prov={"schema_version":1,"figure":"environment-validation","git_sha":subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),"contract_hash":None,"resolved_metrics":{"shelf_speed_rmse_mps":speed_rmse([r for r in rows if r["zone"]=="shelf"]),"nearshore_speed_rmse_mps":speed_rmse([r for r in rows if r["zone"]=="nearshore"]),"observations_ge_0_75_mps":sum(r["reference_speed_mps"]>=.75 for r in rows)},"coverage_cells":{k:{"wet":v[0],"masked":v[1]} for k,v in coverage.items()},"sources":[{"path":str(p.relative_to(ROOT)),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources]};output.with_suffix(".provenance.json").write_text(json.dumps(prov,indent=2)+"\n");return output
if __name__=="__main__":print(build())
