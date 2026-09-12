#!/usr/bin/env python3
import argparse,json,math,os,tempfile
from pathlib import Path
os.environ.setdefault("MPLCONFIGDIR",tempfile.mkdtemp(prefix="bcod-current-figure-"))
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
COLORS={"shelf":"#2474b5","nearshore":"#e18a2d","inside_mask":"#b93b45","unclassified":"#777777"}
def main():
 p=argparse.ArgumentParser();p.add_argument("report",type=Path);p.add_argument("--coverage",type=Path,default=Path("artifacts/environment-coverage/coverage-matrix.json"));p.add_argument("--out",type=Path,default=Path("artifacts/environmental-validation/current-validation.png"));a=p.parse_args();data=json.loads(a.report.read_text());cur=data.get("currents")
 if not cur:raise ValueError("report has no current validation")
 matches=cur["matches"];masked=cur["inside_mask"];fig=plt.figure(figsize=(11,7),layout="constrained");gs=fig.add_gridspec(2,2,height_ratios=[3,1.35]);ax=fig.add_subplot(gs[0,0]);mp=fig.add_subplot(gs[0,1]);hist=fig.add_subplot(gs[1,0]);note=fig.add_subplot(gs[1,1]);note.axis("off")
 for zone in sorted(set(x["zone"] for x in matches)):
  rows=[x for x in matches if x["zone"]==zone];obs=[math.hypot(x["reference_u"],x["reference_v"]) for x in rows];model=[math.hypot(x["model_u"],x["model_v"]) for x in rows];ax.scatter(obs,model,s=8,alpha=.35,label=zone,color=COLORS.get(zone,COLORS["unclassified"]))
 lim=max([1,*[math.hypot(x[k+"_u"],x[k+"_v"]) for x in matches for k in ("reference","model")]]);ax.plot([0,lim],[0,lim],color="#222",lw=.8);ax.set(xlabel="HF radar speed (m/s)",ylabel="RTOFS speed (m/s)",title="Agreement where RTOFS is present");ax.legend(frameon=False);stats=cur.get("overall") or {};ax.text(.03,.97,f"RMSE {stats.get('speed',{}).get('rmse',float('nan')):.3f} m/s\nBias {stats.get('speed',{}).get('bias',float('nan')):.3f} m/s\nN {stats.get('n',0)}",transform=ax.transAxes,va="top")
 for zone,rows in (("shelf",[x for x in matches if x["zone"]=="shelf"]),("nearshore",[x for x in matches if x["zone"]=="nearshore"]),("inside_mask",masked)):
  if rows:mp.scatter([x["longitude_deg"] for x in rows],[x["latitude_deg"] for x in rows],s=5,alpha=.5,label=zone,color=COLORS[zone])
 mp.set(xlabel="Longitude",ylabel="Latitude",title="RTOFS availability over observed envelope");mp.legend(frameon=False)
 angles=[]
 for x in matches:
  if math.hypot(x["model_u"],x["model_v"]) and math.hypot(x["reference_u"],x["reference_v"]):angles.append(((math.degrees(math.atan2(x["model_u"],x["model_v"]))-math.degrees(math.atan2(x["reference_u"],x["reference_v"])))+180)%360-180)
 hist.hist(angles,bins=24,range=(-180,180),color="#5b6f91");hist.set(xlabel="Circular direction error (degrees)",ylabel="Cell-hours")
 coverage=cur["coverage"];fault="not loaded"
 if a.coverage.exists():
  cov=json.loads(a.coverage.read_text());s=cov.get("summary",{});fault=f"{s.get('failure_injection_passed','?')}/{s.get('failure_cells','?')} injected failures contained"
 depths=cur.get("depths",{});temporal=cur.get("temporal_sampling",{});depth_text=depths.get("disclosure","HF radar and RTOFS effective depths differ.");times=temporal.get("unique_model_times","?")
 note.text(0,1,f"RTOFS absent: {coverage['rtofs_absent_cell_hours']}/{coverage['eligible_radar_cell_hours']} eligible radar cell-hours\nAbsent fraction: {coverage['rtofs_absent_fraction']:.1%}\n\nPipeline integrity: {fault}\n\n{depth_text}\n\nTemporal sampling: {times} six-hourly model times, versus 72 in the original hourly design. Cell-hour N does not restore temporal degrees of freedom.",va="top",wrap=True)
 fig.suptitle("San Francisco surface-current characterization · RTOFS vs independent IOOS HF radar",fontweight="bold");a.out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.out,dpi=180);print(a.out)
if __name__=="__main__":main()
