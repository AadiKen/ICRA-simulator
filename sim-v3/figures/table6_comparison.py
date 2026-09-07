#!/usr/bin/env python3
"""Render Table 6 to PNG and Markdown. Usage: python figures/table6_comparison.py [--out PATH]."""
import os,tempfile
_CACHE=tempfile.mkdtemp(prefix="bcod-figure-");os.environ.setdefault("MPLCONFIGDIR",_CACHE);os.environ.setdefault("XDG_CACHE_HOME",_CACHE)
from pathlib import Path
import argparse,json,textwrap
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from lib.style import COLORS,apply_style,savefig
REPO_ROOT=Path(__file__).resolve().parents[1]

def rows(data):
    b=data["construction"]["bcod"];v=data["construction"]["vrx"];m=data["measured_setup"]
    return [("Scenario-specific files",str(b["scenario_specific_files"]),str(v["scenario_specific_world_files"])),("Configuration lines",str(b["configuration_nonblank_lines"]),f'{v["world_nonblank_lines"]} world + {v["generic_launch_nonblank_lines"]} launch'),("External assets",str(b["required_external_assets"]),str(v["required_fuel_assets"])),("Setup commands",str(b["setup_commands"]),"2 container build stages"),("Measured setup","Not separately timed",f'{m["base_image_build_seconds"]:.1f}s base + {m["builder_image_build_seconds"]:.0f}s builder'),("Runtime cross-check","Production scenario resolved",f'WAM-V spawned; sensors/scoring active (<{m["persistent_cache_launch_bound_seconds"]}s bound)')]
def render(source,out,markdown):
    data=json.loads(Path(source).read_text());table=rows(data);claim=data["claim_limit"]
    md="| Measure | BCOD | VRX 3.1.2 |\n|---|---:|---:|\n"+"".join(f"| {a} | {b} | {c} |\n" for a,b,c in table)+f"\n*Claim limit: {claim}*\n";Path(markdown).parent.mkdir(parents=True,exist_ok=True);Path(markdown).write_text(md)
    apply_style();fig,ax=plt.subplots(figsize=(10,3.5));ax.axis("off");wrapped=[[textwrap.fill(str(x),32) for x in row] for row in table]
    t=ax.table(cellText=wrapped,colLabels=["Measure","BCOD","VRX 3.1.2"],cellLoc="left",colLoc="left",loc="upper center",colWidths=[.25,.25,.5]);t.auto_set_font_size(False);t.set_fontsize(8.5);t.scale(1,1.75)
    for (r,c),cell in t.get_celld().items(): cell.set_edgecolor(COLORS["grid"]);cell.set_linewidth(.6);cell.set_facecolor(COLORS["ink"] if r==0 else ("#f7f9fc" if r%2 else "white"));cell.get_text().set_color("white" if r==0 else COLORS["ink"]);cell.get_text().set_weight("bold" if r==0 or c==0 else "normal")
    ax.set_title("Table 6 · scenario construction comparison",loc="left",weight="bold",fontsize=14,pad=16)
    fig.text(.05,.025,"Claim limit: "+textwrap.fill(claim,145),fontsize=8,color=COLORS["muted"])
    return savefig(fig,out)
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--source",type=Path,default=REPO_ROOT/"artifacts/vrx/construction-comparison.json");p.add_argument("--out",type=Path,default=REPO_ROOT/"figures/out/table6_comparison.png");p.add_argument("--markdown",type=Path,default=REPO_ROOT/"figures/out/table6_comparison.md");a=p.parse_args();render(a.source,a.out,a.markdown)
