"""Figure 3: production-plugin sensor outputs under fixed-scene weather changes."""
from __future__ import annotations
import argparse,json,math,os,tempfile
from pathlib import Path
import hashlib,subprocess
os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"bcod-matplotlib"))
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from matplotlib.patches import Circle,FancyBboxPatch,Polygon,Wedge

ROOT=Path(__file__).resolve().parents[2];DEFAULT=ROOT/"artifacts/fig3-sensing/manifest.json";OUTPUT=ROOT/"figures/out/publication/fig3_sensing.png"
INK="#172033";MUTED="#667085";TEAL="#008C95";GREEN="#26734D";BLUE="#2457A7";CHROME="#DCECF4";RED="#B42318"
FONT_SHA256="bd47314d301e50ff4d109bff28dfcf637cb7eb13945480259878b848875acc65";EXPOSURE_STOPS=2.0

def configure_font():
 matches=sorted((ROOT/".venv/lib").glob("python*/site-packages/gymnasium/envs/toy_text/font/Minecraft.ttf"))
 if not matches:raise FileNotFoundError("Minecraft.ttf is unavailable")
 font=matches[0]
 if hashlib.sha256(font.read_bytes()).hexdigest()!=FONT_SHA256:raise ValueError("Minecraft.ttf hash mismatch")
 font_manager.fontManager.addfont(font);plt.rcParams["font.family"]=font_manager.FontProperties(fname=font).get_name();plt.rcParams["axes.unicode_minus"]=False;return font

def write_sidecars(manifest:Path,output:Path,font:Path):
 data=json.loads(manifest.read_text());prov={"schema_version":1,"figure":"fig3_sensing","git_sha":subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),"contract_hash":data.get("contract_hash"),"render_resources":{"embedded_font":"Minecraft.ttf","sha256":hashlib.sha256(font.read_bytes()).hexdigest()},"sources":[{"path":str(manifest.resolve().relative_to(ROOT)),"sha256":hashlib.sha256(manifest.read_bytes()).hexdigest()}],"display_transform":{"camera_exposure_stops":EXPOSURE_STOPS,"applied_uniformly_to_conditions":["clear","fog","rain"],"raw_statistics_modified":False}}
 output.with_suffix(".provenance.json").write_text(json.dumps(prov,indent=2)+"\n")
 stats={"schema_version":1,"figure":"fig3_sensing","statistics_source":"raw 8x6 policy camera and production lidar/radar payloads; no display transform applied","conditions":{}}
 for row in data["weather"]:
  camera=row["camera"];lidar=row["lidar"];radar=row["radar"]
  stats["conditions"][row["condition"]]={"camera":{"transmission_at_80_m":camera["transmission_at_target"],"mean_intensity_0_255":camera["mean_intensity_0_255"],"min_intensity_0_255":camera.get("min_intensity_0_255"),"max_intensity_0_255":camera.get("max_intensity_0_255"),"illumination_lux":camera["payload"]["illumination_lux"]},"lidar":{"detected_count":lidar["detected_count"],"ray_count":16,"max_range_m":lidar["payload"]["max_range_m"],"range_noise_std_m":lidar["payload"]["range_noise_std_m"],"transmission_at_80_m":lidar["transmission_at_target"]},"radar":{"detected_count":radar["detected_count"],"beam_count":12,"clutter_db":radar["clutter_db"],"center_beam_transmission_loss_db":radar["beams"][6]["transmission_loss_db"]}}
 output.with_suffix(".stats.json").write_text(json.dumps(stats,indent=2)+"\n")

def load(path:Path):
 data=json.loads(path.read_text())
 if data["schema_version"]<2 or not data["scene"]["fixed_across_cells"]:raise ValueError("Figure 3 requires production outputs at fixed geometry")
 if [x["condition"] for x in data["weather"]] != ["clear","fog","rain"]:raise ValueError("weather rows must be clear, fog, rain")
 if "config" in data:raise ValueError("unsupported configuration illustrations must not be present")
 return data

def vessel(ax,x,y,scale=1):
 ax.add_patch(FancyBboxPatch((x-9*scale,y-4*scale),18*scale,8*scale,boxstyle=f"round,pad=0.1,rounding_size={3*scale}",facecolor="#EEF7FB",edgecolor=INK,lw=1.3));ax.add_patch(Polygon([[x+11*scale,y],[x+7*scale,y+4*scale],[x+7*scale,y-4*scale]],closed=True,facecolor="#EEF7FB",edgecolor=INK,lw=1.3))

def scene_panel(ax,data):
 scene=data["scene"];bearings=scene["radar_beam_bearings_rad"];ranges=scene["radar_geometric_ranges_m"];ax.set_aspect("equal");ax.set(xlim=(-155,155),ylim=(-25,235));ax.set_facecolor("#F5FAFC")
 ax.add_patch(Wedge((0,0),225,90-math.degrees(max(bearings)),90-math.degrees(min(bearings)),facecolor=CHROME,alpha=.55,edgecolor=BLUE,lw=1,ls="--"))
 for bearing,rng in zip(bearings,ranges):
  east=rng*math.sin(bearing);north=rng*math.cos(bearing);ax.plot([0,east],[0,north],color="#BCD7E4",lw=.5);ax.add_patch(Circle((east,north),4.5,facecolor="#E8B86D",edgecolor="#8A5A14",lw=.8))
 vessel(ax,0,0,1);ax.scatter([0],[80],s=120,marker="*",color=RED,edgecolor="white",zorder=5);ax.annotate("Reference target\n80 m | 0 deg bearing",xy=(0,80),xytext=(45,100),fontsize=8,color=INK,arrowprops={"arrowstyle":"-","color":MUTED})
 ax.scatter([0],[0],s=16,color=INK,zorder=6);ax.text(14,8,"Co-located sensors",fontsize=8,color=INK);ax.text(-145,220,"Fixed geometry",fontsize=10,fontweight="bold",color=INK);ax.text(-145,207,"Identical pose and targets",fontsize=8,color=MUTED)
 ax.set_xlabel("East (m)");ax.set_ylabel("North (m)");ax.grid(color="white",lw=.8);ax.spines[["top","right"]].set_visible(False)

def camera_cell(ax,row):
 camera=row["camera"];p=camera["diagnostic_payload"];rgba=np.asarray(p["rgba"],dtype=float).reshape(p["height"],p["width"],4)/255
 displayed=rgba.copy();boost=displayed[:,:,:3]*(2**EXPOSURE_STOPS);displayed[:,:,:3]=boost/(1+boost)
 ax.imshow(displayed,interpolation="nearest",vmin=0,vmax=1);ax.set_xticks([]);ax.set_yticks([])
 for spine in ax.spines.values():spine.set_color(CHROME);spine.set_linewidth(1)
 ax.text(.02,.98,"80x60 diagnostic render | policy input remains 8x6",transform=ax.transAxes,va="top",fontsize=6.1,color="white",bbox={"facecolor":INK,"edgecolor":"none","alpha":.78,"pad":2})
 ax.text(.02,.04,f"T80 {camera['transmission_at_target']:.2f} | mean {camera['mean_intensity_0_255']:.0f}/255 | {p['illumination_lux']:.0f} lux",transform=ax.transAxes,fontsize=6.2,color="white",bbox={"facecolor":INK,"edgecolor":"none","alpha":.78,"pad":2})

def polar_style(ax,angle=43,rmax=500):
 ax.set_theta_zero_location("N");ax.set_theta_direction(-1);ax.set_thetamin(-angle);ax.set_thetamax(angle);ax.set_ylim(0,rmax);ax.set_yticks([rmax/2,rmax]);ax.set_yticklabels([f"{rmax/2:.0f}",f"{rmax:.0f} m"],fontsize=5);ax.set_xticks(np.radians([-30,0,30]));ax.set_xticklabels(["-30 deg","0 deg","30 deg"]);ax.tick_params(labelsize=5,pad=1);ax.grid(color=CHROME,lw=.6)

def lidar_cell(fig,spec,row):
 ax=fig.add_subplot(spec,projection="polar");lidar=row["lidar"];payload=lidar["payload"];polar_style(ax,43,float(payload["max_range_m"]));points=[p for p in lidar["points"] if p["detected"]]
 if len(points)!=lidar["detected_count"]:raise ValueError(f"{row['condition']} lidar payload/count mismatch")
 if points:
  markers=ax.scatter([p["bearing_rad"] for p in points],[p["range_m"] for p in points],s=20,c=GREEN,marker="o",edgecolors=INK,linewidths=.4,zorder=4);markers.set_gid(f"lidar-markers-{row['condition']}")
 else:ax.text(0,float(payload["max_range_m"])*.52,"0/16 returns",ha="center",va="center",fontsize=7,color=RED)
 ax.text(.02,.02,f"{lidar['detected_count']}/16 returns | max {payload['max_range_m']:.1f} m\nsigma_r {payload['range_noise_std_m']:.3f} m | T80 {lidar['transmission_at_target']:.2f}",transform=ax.transAxes,fontsize=5.8,color=INK)
 return ax

def radar_cell(fig,spec,row,audit):
 ax=fig.add_subplot(spec,projection="polar");radar=row["radar"];polar_style(ax,34.4,float(radar["max_range_m"]));det=radar["payload"]["detections"]
 if len(det)!=radar["detected_count"]:raise ValueError(f"{row['condition']} radar payload/count mismatch")
 if det:
  markers=ax.scatter([p["bearing_rad"] for p in det],[p["range_m"] for p in det],s=20,c=TEAL,marker="D",edgecolors=INK,linewidths=.4,zorder=4);markers.set_gid(f"radar-markers-{row['condition']}")
 else:ax.text(0,260,f"Seed {audit['display_seed']}: 0/12",ha="center",va="center",fontsize=7,color=RED)
 center=radar["beams"][6];label=f"{radar['detected_count']}/12 hits | TL {center['transmission_loss_db']:.1f} dB\nclutter {radar['clutter_db']:.1f} dB"
 if row["condition"]==audit["condition"]:label+=f" | seed mean {audit['mean_hits']:.2f}"
 ax.text(.02,.02,label,transform=ax.transAxes,fontsize=5.7,color=INK)
 return ax

def build(manifest:Path=DEFAULT,output:Path=OUTPUT):
 font=configure_font();data=load(manifest);weather={x["condition"]:x for x in data["weather"]};audit={**data["radar_seed_audit"],"display_seed":data["seed"]}
 fig=plt.figure(figsize=(14.5,7.25),facecolor="white");outer=fig.add_gridspec(1,2,width_ratios=[.82,2.18],left=.055,right=.985,bottom=.105,top=.85,wspace=.13)
 ax_scene=fig.add_subplot(outer[0]);scene_panel(ax_scene,data);ax_scene.set_title("A | Scene Context",loc="left",fontsize=12,fontweight="bold",pad=13,color=INK)
 grid=outer[1].subgridspec(4,4,width_ratios=[.30,1,1,1],height_ratios=[.23,1,1,1],wspace=.09,hspace=.11)
 label=fig.add_subplot(grid[0,0]);label.axis("off");label.text(0,1,"Weather",fontsize=10,fontweight="bold",color=INK,va="top")
 for j,(name,domain,kind) in enumerate((("Camera","LightDomain","80x60 diagnostic"),("Lidar","LightDomain","16-ray returns"),("Radar","RfDomain","12-beam detections")),1):
  ax=fig.add_subplot(grid[0,j]);ax.axis("off");ax.text(.5,.88,name,ha="center",fontsize=10,fontweight="bold",color=INK);ax.text(.5,.33,f"{domain} | {kind}",ha="center",fontsize=7,color=MUTED)
 for i,condition in enumerate(("clear","fog","rain"),1):
  lab=fig.add_subplot(grid[i,0]);lab.axis("off");lab.text(.98,.5,condition.title(),ha="right",va="center",fontsize=9,fontweight="bold",color=INK)
  camera_cell(fig.add_subplot(grid[i,1]),weather[condition]);lidar_cell(fig,grid[i,2],weather[condition]);radar_cell(fig,grid[i,3],weather[condition],audit)
 fig.text(.365,.865,"B | Production Sensor Output",fontsize=12,fontweight="bold",color=INK)
 fig.text(.055,.055,f"Camera panels: native 80x60 diagnostic capture with +{EXPOSURE_STOPS:g}-stop tone mapping; policy observation remains 8x6; T80 and mean/255 report the raw 8x6 output.",fontsize=7,color=MUTED)
 fig.text(.985,.035,f"Rain radar audit: {audit['zero_hit_fraction']:.1%} zero-hit scans; mean {audit['mean_hits']:.3f}/12 across seeds {audit['seed_start']}-{audit['seed_end']}.",ha="right",fontsize=7,color=MUTED)
 fig.suptitle("Environmentally Responsive Sensing",x=.055,y=.965,ha="left",fontsize=18,fontweight="bold",color=INK);fig.text(.055,.915,f"bcod-sim only | fixed scene geometry | displayed stochastic draw seed {data['seed']}",fontsize=10,color=MUTED)
 output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=240,bbox_inches="tight",pad_inches=.08);fig.savefig(output.with_suffix(".svg"),bbox_inches="tight",pad_inches=.08);plt.close(fig);write_sidecars(manifest,output,font);return output

def main():
 p=argparse.ArgumentParser();p.add_argument("--manifest",type=Path,default=DEFAULT);p.add_argument("--output",type=Path,default=OUTPUT);a=p.parse_args();print(build(a.manifest,a.output))
if __name__=="__main__":main()
