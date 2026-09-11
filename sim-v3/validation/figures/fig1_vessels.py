from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .common import ROOT, require_files

DEFAULTS = [ROOT / "artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison.json", ROOT / "artifacts/gazebo/vehicle-c/open-loop-trajectory-comparison-thrust-corrected.json"]

def validate_inputs(reports):
    for report, count in zip(reports, (6, 4)):
        if len(report.get("scenarios", [])) != count: raise ValueError(f"Expected {count} maneuver scenarios")
        for scenario in report["scenarios"]:
            for key in ("bcod_sim", "gazebo_harmonic"):
                rows = scenario.get(key)
                if not rows or not all(all(field in sample for field in ("north_m", "east_m", "heading_rad")) for sample in rows): raise ValueError(f"{scenario.get('id')} lacks full {key} trajectory samples")

def _trace(ax, rows, color, label):
    east=np.asarray([row["east_m"] for row in rows]);north=np.asarray([row["north_m"] for row in rows]);ax.plot(east,north,color=color,lw=1.8,label=label)
    scale=max(np.ptp(east),np.ptp(north),.1)*.075
    for index in np.unique(np.linspace(0,len(rows)-1,min(5,len(rows)),dtype=int)):
        heading=float(rows[index]["heading_rad"]);ax.arrow(east[index],north[index],scale*np.sin(heading),scale*np.cos(heading),color=color,width=scale*.035,head_width=scale*.25,length_includes_head=True,alpha=.8)

def build(data_paths=None) -> Path:
    paths=require_files([Path(path) for path in (data_paths or DEFAULTS)]);reports=[json.loads(path.read_text()) for path in paths];validate_inputs(reports)
    fig,axes=plt.subplots(2,6,figsize=(16,6.2),layout="constrained")
    for row,(report,vehicle,model) in enumerate(zip(reports,("Vehicle A","Vehicle C"),("planar3","coupled6"))):
        for column in range(6):
            ax=axes[row,column]
            if column>=len(report["scenarios"]): ax.axis("off");continue
            scenario=report["scenarios"][column];_trace(ax,scenario["bcod_sim"],"#2457a7","bcod-sim");_trace(ax,scenario["gazebo_harmonic"],"#d97706","Gazebo Harmonic")
            ax.set_title(scenario["id"].replace("-"," ").title(),fontsize=9);ax.set_aspect("equal",adjustable="datalim");ax.grid(alpha=.2);ax.tick_params(labelsize=7)
            if column==0: ax.set_ylabel(f"{vehicle} · {model}\nNorth (m)",fontweight="bold")
            if row==1: ax.set_xlabel("East (m)")
    axes[0,0].legend(frameon=False,fontsize=8,loc="best");fig.suptitle("Native vessel trajectory agreement",fontsize=16,fontweight="bold")
    output=ROOT/"artifacts/figures/fig1_vessels.png";output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=220);plt.close(fig);return output
