"""Camera-ready matplotlib styling shared by all publication figures."""
from pathlib import Path
import matplotlib as mpl

COLORS = {"ink":"#172033","muted":"#667085","paper":"#ffffff","grid":"#d0d5dd",
          "good":"#26734d","expected":"#d97706","negative":"#b42318","na":"#e4e7ec",
          "blue":"#2457a7","ocean":"#dcecf4","land":"#eee8da"}
DPI = 240

def apply_style():
    mpl.rcParams.update({"font.family":"DejaVu Sans","font.size":9,"axes.titlesize":11,
        "axes.labelsize":9,"axes.edgecolor":COLORS["grid"],"axes.linewidth":.7,
        "xtick.labelsize":8,"ytick.labelsize":8,"figure.facecolor":COLORS["paper"],
        "axes.facecolor":COLORS["paper"],"savefig.facecolor":COLORS["paper"]})

def savefig(fig, path, **kwargs):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,dpi=kwargs.pop("dpi",DPI),bbox_inches="tight",pad_inches=.08,**kwargs)
    return path
