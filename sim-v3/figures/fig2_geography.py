#!/usr/bin/env python3
"""Render Figure 2 and its evidence-table fallback. Usage: python figures/fig2_geography.py [--out PATH]."""
import os,tempfile
_CACHE=tempfile.mkdtemp(prefix="bcod-figure-");os.environ.setdefault("MPLCONFIGDIR",_CACHE);os.environ.setdefault("XDG_CACHE_HOME",_CACHE)
from pathlib import Path
import argparse,json,statistics
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.patches import Wedge
import cartopy.crs as ccrs
import cartopy.io.shapereader as shpreader
from lib.style import COLORS,apply_style,savefig
from lib.geo import load_gebco,add_coastline
REPO_ROOT=Path(__file__).resolve().parents[1]
SOURCES=("ndbc","coops","nws","rtofs","era5","bathymetry_cascade")

def matrix(data): return {(r["source"],r["site"]):[] for r in data["coverage_matrix"]}
def grouped(data):
    g=matrix(data)
    for r in data["coverage_matrix"]:g[(r["source"],r["site"])].append(r)
    if len(data["coverage_matrix"])!=48:raise ValueError("Figure 2 requires exactly 48 coverage records")
    return g
def status(rows): return all(r["success"] and r["provenance_complete"] for r in rows)
def render_table(data,out,mdout):
    sites=[s["id"] for s in data["methodology"]["sites"]];g=grouped(data);body=[]
    for source in SOURCES:
        row=[]
        for site in sites:
            rs=g[(source,site)];ok=status(rs);lat=statistics.median(r["retrieval_latency_ms"] for r in rs)
            row.append(f'{"PASS" if ok else "FAIL"}\n2/2 checks · {lat:.2f} ms')
        body.append(row)
    md="| Source | "+" | ".join(s.replace("-"," ").title() for s in sites)+" |\n|---|"+"---|"*len(sites)+"\n"
    for source,row in zip(SOURCES,body):md+=f'| {source} | '+" | ".join(x.replace("\n","; ") for x in row)+" |\n"
    md+="\nValues report two seasonal simulator retrieval/provenance checks and median adapter latency; the source artifact contains no paired numeric observed-value field.\n";Path(mdout).parent.mkdir(parents=True,exist_ok=True);Path(mdout).write_text(md)
    apply_style();fig,ax=plt.subplots(figsize=(10.5,4));ax.axis("off");tab=ax.table(cellText=body,rowLabels=[s.replace("_"," ").upper() for s in SOURCES],colLabels=[s.replace("-"," ").title() for s in sites],cellLoc="center",loc="center");tab.auto_set_font_size(False);tab.set_fontsize(8);tab.scale(1,1.65)
    for (r,c),cell in tab.get_celld().items():cell.set_edgecolor(COLORS["grid"]);cell.set_linewidth(.6);cell.set_facecolor(COLORS["ink"] if r==0 else ("#edf7f1" if c>=0 else "#f5f7fa"));cell.get_text().set_color("white" if r==0 else COLORS["ink"]);cell.get_text().set_weight("bold" if r==0 or c==-1 else "normal")
    ax.set_title("Figure 2 fallback · environmental source coverage",loc="left",weight="bold",fontsize=14,pad=12)
    fig.text(.08,.015,"Each cell: pass/fail across two seasonal fixtures · median adapter retrieval/parse latency. All 48 records retain complete provenance.",fontsize=8,color=COLORS["muted"])
    savefig(fig,out);plt.close(fig)
def _plot_effect_panel(ax,effects,ids,title):
    colors={"idealized-zero":COLORS["ink"],"slack-water":"#8293a8","retrieved-wind":COLORS["expected"],"peak-ebb":"#6f42c1","peak-flood":COLORS["negative"]}
    labels={"idealized-zero":"Zero environment","slack-water":"Slack water","retrieved-wind":"Retrieved wind","peak-ebb":"Peak ebb","peak-flood":"Peak flood"}
    route=[[0,0],*effects["scenario"]["route_ned_m"]]
    ax.plot([p[1] for p in route],[p[0] for p in route],"o--",color=COLORS["ink"],lw=1.1,ms=3,label="Frozen route",zorder=5)
    by_id={arm["id"]:arm for arm in effects["arms"]}
    for arm_id in ids:
        arm=by_id[arm_id];samples=arm["samples"];sep=arm["final_separation_from_idealized_m"]
        state="complete" if arm["route_completed"] else "incomplete"
        ax.plot([s["east_m"] for s in samples],[s["north_m"] for s in samples],color=colors[arm_id],lw=1.7,label=f'{labels[arm_id]} · {sep:.2f} m · {state}')
        ax.scatter(samples[-1]["east_m"],samples[-1]["north_m"],s=22,color=colors[arm_id],edgecolor="white",linewidth=.5,zorder=6)
    ax.set_title(title,loc="left",weight="bold");ax.set_xlabel("East displacement (m)");ax.set_ylabel("North displacement (m)");ax.grid(color=COLORS["grid"],lw=.45,alpha=.7);ax.set_aspect("equal",adjustable="datalim")
    ax.legend(loc="best",fontsize=7,frameon=True,framealpha=.92)

def render_map(data,effects,out,data_root):
    if effects.get("status")!="PREREGISTERED_CHECKS_PASSED":raise ValueError("Panel B requires a sweep that cleared preregistered checks")
    sites=data["methodology"]["sites"];g=grouped(data);apply_style();proj=ccrs.PlateCarree();fig=plt.figure(figsize=(15,8.4));gs=fig.add_gridspec(2,3,width_ratios=[1,1,1.38],wspace=.16,hspace=.2)
    map_axes=[fig.add_subplot(gs[i//2,i%2],projection=proj) for i in range(4)]
    for ax,site in zip(map_axes,sites):
        sid=site["id"];lon,lat,z=load_gebco(Path(data_root)/"gebco_regional"/f"{sid}.txt")
        ax.set_extent([lon.min(),lon.max(),lat.min(),lat.max()],crs=proj);ax.set_facecolor(COLORS["ocean"])
        relief=LightSource(azdeg=315,altdeg=38).shade(z,cmap=plt.get_cmap("Blues_r"),vert_exag=.08,blend_mode="soft")
        ax.imshow(relief,extent=[lon.min(),lon.max(),lat.min(),lat.max()],origin="lower",transform=proj,zorder=1)
        add_coastline(ax,Path(data_root)/"naturalearth",ccrs,shpreader,COLORS)
        ax.plot(site["longitude_deg"],site["latitude_deg"],marker="o",ms=5,color=COLORS["negative"],mec="white",transform=proj,zorder=5)
        x=.82;y=.18;r=.115
        for i,source in enumerate(SOURCES[:5]):
            ok=status(g[(source,sid)]);ax.add_patch(Wedge((x,y),r,i*72+3,(i+1)*72-3,transform=ax.transAxes,facecolor=COLORS["good"] if ok else COLORS["negative"],edgecolor="white",lw=.6,zorder=8))
        ax.add_patch(plt.Circle((x,y),r*.36,transform=ax.transAxes,color="white",zorder=9));ax.text(x,y,"5/5",transform=ax.transAxes,ha="center",va="center",fontsize=7,weight="bold",zorder=10)
        gl=ax.gridlines(draw_labels=True,linewidth=.35,color=COLORS["grid"],alpha=.8);gl.top_labels=False;gl.right_labels=False;gl.xlabel_style={"size":7};gl.ylabel_style={"size":7}
        ax.text(.025,.965,sid.replace("-"," ").title(),transform=ax.transAxes,ha="left",va="top",weight="bold",fontsize=9,bbox={"facecolor":"white","edgecolor":"none","alpha":.82,"pad":2},zorder=12)
    detail=gs[:,2].subgridspec(2,1,hspace=.3);current_ax=fig.add_subplot(detail[0]);detail_ax=fig.add_subplot(detail[1])
    _plot_effect_panel(current_ax,effects,["idealized-zero","peak-ebb","peak-flood"],"B1 · Peak-current response")
    _plot_effect_panel(detail_ax,effects,["idealized-zero","slack-water","retrieved-wind"],"B2 · Slack and wind detail")
    fig.suptitle("Geographic coverage and controlled environmental response",x=.055,ha="left",fontsize=16,weight="bold")
    fig.text(.055,.925,"A · GEBCO 2026 regional bathymetry and verified source coverage",weight="bold",fontsize=10)
    fig.text(.69,.925,"B · 180 s LOS-controlled route · separation from zero-environment arm",weight="bold",fontsize=10)
    fig.text(.055,.012,"A: glyphs summarize NDBC, CO-OPS, NWS, RTOFS, and ERA5 retrieval/provenance checks. B: peak ebb and flood prevent route completion; zero, slack, and wind complete. Wind uses explicitly uncalibrated placeholder coefficient amplitudes (CX=1, CY=0.5, CN=0.25); coupling evidence is not Otter aerodynamic validation.",fontsize=7.5,color=COLORS["muted"])
    fig.subplots_adjust(left=.055,right=.985,top=.885,bottom=.075);savefig(fig,out);plt.close(fig)
def main():
    p=argparse.ArgumentParser();p.add_argument("--source",type=Path,default=REPO_ROOT/"artifacts/environment-coverage/coverage-matrix.json");p.add_argument("--effect-source",type=Path,default=REPO_ROOT/"artifacts/environment-coverage/environment-effect-sweep.json");p.add_argument("--data",type=Path,default=REPO_ROOT/"figures/data");p.add_argument("--out",type=Path,default=REPO_ROOT/"figures/out/fig2_geography.png");p.add_argument("--table-out",type=Path,default=REPO_ROOT/"figures/out/fig2_geography_table.png");p.add_argument("--markdown",type=Path,default=REPO_ROOT/"figures/out/fig2_geography_table.md");a=p.parse_args();data=json.loads(a.source.read_text());effects=json.loads(a.effect_source.read_text());render_table(data,a.table_out,a.markdown);render_map(data,effects,a.out,a.data)
if __name__=="__main__":main()
