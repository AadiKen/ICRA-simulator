#!/usr/bin/env python3
"""Render the determinism matrix. Usage: python figures/fig4_determinism.py [--out PATH]."""
import os,tempfile
_CACHE=tempfile.mkdtemp(prefix="bcod-figure-");os.environ.setdefault("MPLCONFIGDIR",_CACHE);os.environ.setdefault("XDG_CACHE_HOME",_CACHE)
from pathlib import Path
import argparse,json
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from lib.style import COLORS,apply_style,savefig
REPO_ROOT=Path(__file__).resolve().parents[1]

def render(source,out):
    report=json.loads(Path(source).read_text());cells=report["cells"];summary=report["summary"]
    actual={"bit_identical":sum(c["status"]=="passed" for c in cells),"expected_float_divergence":sum(c["status"]=="expected-float-divergence" for c in cells),"negative_control_detected":sum(c["status"]=="negative-control-detected" for c in cells),"not_applicable":sum(c["status"]=="not-applicable" for c in cells)}
    if len(cells)!=summary["cell_count"] or any(actual[k]!=summary[k] for k in actual): raise ValueError(f"determinism summary mismatch: {actual}")
    axes=list(dict.fromkeys(c["axis"] for c in cells));cols=list(dict.fromkeys((c["vehicle"],c["plant"]) for c in cells));idx={(c["axis"],c["vehicle"],c["plant"]):c for c in cells}
    palette={"passed":COLORS["good"],"expected-float-divergence":COLORS["expected"],"negative-control-detected":COLORS["negative"],"not-applicable":COLORS["na"]}
    apply_style();fig,ax=plt.subplots(figsize=(9.3,5.7));ax.set_xlim(0,len(cols));ax.set_ylim(len(axes),0)
    for y,a in enumerate(axes):
        for x,(v,p) in enumerate(cols):
            cell=idx.get((a,v,p));status=cell["status"] if cell else "not-applicable"
            ax.add_patch(plt.Rectangle((x+.04,y+.06),.92,.88,color=palette[status],ec="white",lw=.7))
            label={"passed":"✓","expected-float-divergence":"≈","negative-control-detected":"!","not-applicable":"—"}[status]
            ax.text(x+.5,y+.53,label,ha="center",va="center",weight="bold",fontsize=11,color="white" if status!="not-applicable" else COLORS["muted"])
    ax.set_xticks([i+.5 for i in range(len(cols))],[f'{v.replace("vehicle-","").replace("-"," ").title()}\n{p}' for v,p in cols])
    ax.set_yticks([i+.5 for i in range(len(axes))],[a.replace("-"," ") for a in axes]);ax.tick_params(length=0);ax.set_frame_on(False)
    ax.set_title("Determinism under controlled perturbations",loc="left",weight="bold",fontsize=14,pad=32)
    ax.text(0,1.035,"55 cells preserve the full (axis, vehicle, plant) key",transform=ax.transAxes,color=COLORS["muted"])
    handles=[Patch(color=palette[k],label=l) for k,l in [("passed","Bit-identical (32)"),("expected-float-divergence","Expected float divergence (1)"),("negative-control-detected","Negative control detected (1)"),("not-applicable","Not applicable (21)")]]
    fig.legend(handles=handles,ncol=4,loc="lower center",bbox_to_anchor=(.55,.055),frameon=False,fontsize=8)
    fig.text(.2,.018,"SHA-256 comparisons use canonical trace rows; N/A cells retain their recorded applicability reason.",fontsize=8,color=COLORS["muted"])
    fig.subplots_adjust(bottom=.2)
    return savefig(fig,out)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--source",type=Path,default=REPO_ROOT/"artifacts/determinism-sweep/report.json");p.add_argument("--out",type=Path,default=REPO_ROOT/"figures/out/fig4_determinism_matrix.png");a=p.parse_args();render(a.source,a.out)
