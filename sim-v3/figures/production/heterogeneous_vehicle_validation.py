"""Two-row path validation figure rebuilt directly from authoritative JSON traces."""
from __future__ import annotations
import argparse,hashlib,json,os,subprocess,tempfile
from pathlib import Path
os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"bcod-matplotlib"))
import matplotlib;matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
VEHICLE_A=ROOT/"artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison.json"
VEHICLE_C=ROOT/"artifacts/gazebo/vehicle-c/open-loop-trajectory-comparison-thrust-corrected.json"
VEHICLE_C_GENERIC=ROOT/"artifacts/gazebo/vehicle-c-generic/comparison.json"
ROTATION_DIAGNOSTIC=ROOT/"artifacts/gazebo/vehicle-c/rotation-in-place-diagnostic.json"
OUTPUT=ROOT/"figures/out/publication/fig1_vessels.png"
TEAL="#008C95";ORANGE="#D97706";RED="#B42318";INK="#172033";GREY="#667085";GRID="#DCECF4"
FONT_SHA256="bd47314d301e50ff4d109bff28dfcf637cb7eb13945480259878b848875acc65"
NAMES={"constant-thrust":"Constant Thrust","turning-circle":"Turning Circle","yaw-turn":"Yaw Turn","zig-zag":"Zig Zag","coast-down":"Coast Down","current-drift":"Current Drift","straight-ahead":"Straight Ahead","pure-lateral":"Pure Lateral","rotation-in-place":"Rotation In Place","allocation-chirp":"Allocation Chirp"}

def configure_font():
 matches=sorted((ROOT/".venv/lib").glob("python*/site-packages/gymnasium/envs/toy_text/font/Minecraft.ttf"))
 if not matches:raise FileNotFoundError("Minecraft.ttf is unavailable")
 font=matches[0];digest=hashlib.sha256(font.read_bytes()).hexdigest()
 if digest!=FONT_SHA256:raise ValueError("Minecraft.ttf hash mismatch")
 font_manager.fontManager.addfont(font);plt.rcParams["font.family"]=font_manager.FontProperties(fname=font).get_name();plt.rcParams["axes.unicode_minus"]=False;return font

def load(path:Path):
 data=json.loads(path.read_text())
 if not data.get("timing",{}).get("full_resolution"):raise ValueError(f"full-resolution source required: {path}")
 for scenario in data.get("scenarios",[]):
  if not scenario.get("bcod_sim") or not scenario.get("gazebo_harmonic"):raise ValueError(f"missing traces: {scenario.get('id')}")
 return data

def limits(scenario):
 x=np.array([p["east_m"] for source in ("bcod_sim","gazebo_harmonic") for p in scenario[source]],float);y=np.array([p["north_m"] for source in ("bcod_sim","gazebo_harmonic") for p in scenario[source]],float)
 span=max(float(np.ptp(x)),float(np.ptp(y)),.015);return (float(x.min()-span*.12),float(x.max()+span*.12),float(y.min()-span*.12),float(y.max()+span*.12))

def arrows(ax,trace,color):
 for i in sorted(set((len(trace)//3,2*len(trace)//3))):
  a,b=trace[max(0,i-1)],trace[min(len(trace)-1,i+1)];dx=b["east_m"]-a["east_m"];dy=b["north_m"]-a["north_m"]
  if abs(dx)+abs(dy)>1e-12:ax.annotate("",xy=(b["east_m"],b["north_m"]),xytext=(a["east_m"],a["north_m"]),arrowprops={"arrowstyle":"-|>","color":color,"lw":1.2,"mutation_scale":8})

def panel(ax,scenario,divergence_callout=False):
 b=scenario["bcod_sim"];g=scenario["gazebo_harmonic"]
 ax.plot([p["east_m"] for p in b],[p["north_m"] for p in b],color=TEAL,lw=2.1);ax.plot([p["east_m"] for p in g],[p["north_m"] for p in g],color=ORANGE,lw=1.8,ls="--")
 arrows(ax,b,TEAL);arrows(ax,g,ORANGE);ax.scatter([b[0]["east_m"]],[b[0]["north_m"]],s=15,c=INK,zorder=5);lim=limits(scenario)
 ax.set_title(NAMES[scenario["id"]],fontsize=9.3,fontweight="bold",color=INK,pad=6);ax.set(xlim=lim[:2],ylim=lim[2:],xlabel="East (m)",ylabel="North (m)");ax.set_aspect("equal",adjustable="box");ax.grid(color=GRID,lw=.65);ax.tick_params(labelsize=6.3);ax.spines[["top","right"]].set_visible(False)
 maxerr=scenario["summary"]["horizontal_position_error_m"]["max"];ax.text(.03,.04,f"max dp {maxerr*100:.1f} cm",transform=ax.transAxes,fontsize=6.3,color=GREY)
 if scenario["id"]=="rotation-in-place":ax.text(.97,.97,"Anomalous relative behavior\nunder investigation",transform=ax.transAxes,ha="right",va="top",fontsize=5.8,color=GREY)
 if divergence_callout:
  end=np.array([b[-1]["east_m"],b[-1]["north_m"]]);other=np.array([g[-1]["east_m"],g[-1]["north_m"]]);center=(end+other)/2
  ax.scatter([center[0]],[center[1]],s=95,facecolors="none",edgecolors=RED,lw=1.4,clip_on=False,zorder=8);ax.annotate(f"Largest expanded-battery\ndivergence {maxerr:.2f} m",xy=center,xytext=(22,-24),textcoords="offset points",ha="left",va="top",fontsize=6.3,color=RED,arrowprops={"arrowstyle":"-","ls":"--","color":RED,"lw":1})

def write_provenance(output:Path,font:Path,expanded_worst_callout:bool,vehicle_a:Path,vehicle_c:Path,vehicle_c_generic:Path):
 c=load(vehicle_c);rotation=next(s for s in c["scenarios"] if s["id"]=="rotation-in-place")
 def trace_audit(trace):
  payload=json.dumps(trace,separators=(",",":"),sort_keys=True).encode();east=np.array([p["east_m"] for p in trace]);north=np.array([p["north_m"] for p in trace])
  return {"samples":len(trace),"sha256":hashlib.sha256(payload).hexdigest(),"east_span_m":float(np.ptp(east)),"north_span_m":float(np.ptp(north))}
 sources=(vehicle_a,vehicle_c,vehicle_c_generic,ROTATION_DIAGNOSTIC)
 record={"schema_version":1,"figure":"fig1_vessels","git_sha":subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip(),"contract_hash":None,"render_resources":{"embedded_font":"Minecraft.ttf","sha256":hashlib.sha256(font.read_bytes()).hexdigest()},"sources":[{"path":str(p.resolve().relative_to(ROOT)),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources],"render_decisions":{"axis_scaling":"natural per panel; full trace extent with 12% padding","expanded_battery_worst_callout":{"enabled":expanded_worst_callout,"maneuver":"vehicle-c zig-zag","max_divergence_m":3.581482750752226},"rotation_in_place_annotation":"neutral under-investigation note","aligned_maneuvers":["straight-ahead/constant-thrust","turning-circle","yaw-turn","zig-zag","coast-down"],"vehicle_c_current_drift":"dropped because matched Gazebo current forcing was unavailable","row_subtitles":"omitted pending explicit author approval"},"worst_case_attribution":{"original_vehicle_c_suite":{"horizontal_position_error":{"maneuver":"straight-ahead","value_cm":28.418},"heading_error":{"maneuver":"pure-lateral","value_deg":2.9135}},"expanded_generic_battery":{"horizontal_position_error":{"maneuver":"zig-zag","value_m":3.581482750752226}}},"rotation_in_place_diagnostic":{"units":"meters","classification":"real cross-model dynamics difference; root cause not isolated","known_drift_signature_match":False,"shared_actuator_pairing_root_cause_ruled_out":False,"command_interpretation":"exactly opposed +/-40.215182 N pod thrust at zero azimuth; zero net force and 65.148595 N m yaw torque","max_horizontal_position_error_cm":rotation["summary"]["horizontal_position_error_m"]["max"]*100,"bcod_sim":trace_audit(rotation["bcod_sim"]),"gazebo_harmonic":trace_audit(rotation["gazebo_harmonic"]),"long_horizon_bcod":"60 s diagnostic ends 3.64 cm from origin after an 8.12 cm maximum east excursion; the startup bowl does not settle to zero","comparison":"Gazebo also moves toward negative north at millimeter scale, but its east component has the opposite sign. The models share only part of the drift direction."},"vehicle_c_mixed_provenance":{"analytically_sign_corrected_pre_fix_captures":["straight-ahead","pure-lateral","rotation-in-place"],"physically_rerun":["allocation-chirp","turning-circle","yaw-turn","zig-zag","coast-down"],"note":"The original three symmetric/rotation traces retain pre-fix captures with source-declared analytical normalization. Allocation Chirp and all four generic-battery Gazebo traces were physically run against the corrected SDF."}}
 output.with_suffix(".provenance.json").write_text(json.dumps(record,indent=2)+"\n")

def build(vehicle_a:Path=VEHICLE_A,vehicle_c:Path=VEHICLE_C,output:Path=OUTPUT,*,worst_case_callout:bool=True,vehicle_c_generic:Path=VEHICLE_C_GENERIC)->Path:
 font=configure_font();a=load(vehicle_a);c=load(vehicle_c);g=load(vehicle_c_generic);expected_a=["constant-thrust","turning-circle","yaw-turn","zig-zag","coast-down","current-drift"];expected_c=["straight-ahead","pure-lateral","rotation-in-place","allocation-chirp"];expected_g=["turning-circle","yaw-turn","zig-zag","coast-down"]
 if [s["id"] for s in a["scenarios"]]!=expected_a or [s["id"] for s in c["scenarios"]]!=expected_c or [s["id"] for s in g["scenarios"]]!=expected_g:raise ValueError("authoritative maneuver set or ordering changed")
 fig=plt.figure(figsize=(23,7.2),facecolor="white");outer=fig.add_gridspec(2,1,left=.045,right=.99,bottom=.14,top=.84,hspace=.58);top=outer[0].subgridspec(1,9,wspace=.48);bottom=outer[1].subgridspec(1,9,wspace=.48)
 for i,s in enumerate(a["scenarios"]):panel(fig.add_subplot(top[0,i]),s)
 panel(fig.add_subplot(bottom[0,0]),c["scenarios"][0])
 for i,s in enumerate(g["scenarios"],1):panel(fig.add_subplot(bottom[0,i]),s,divergence_callout=worst_case_callout and s["id"]=="zig-zag")
 for i,s in enumerate(c["scenarios"][1:],6):panel(fig.add_subplot(bottom[0,i]),s)
 fig.suptitle("Heterogeneous Vehicle Validation",x=.055,y=.965,ha="left",fontsize=18,fontweight="bold",color=INK);fig.text(.055,.91,"Open-loop path agreement with independent Gazebo Harmonic implementations",fontsize=10,color=GREY);fig.text(.012,.69,"Vehicle A\nplanar3",ha="left",va="center",fontsize=9,fontweight="bold",color=INK);fig.text(.012,.31,"Vehicle C\ncoupled6",ha="left",va="center",fontsize=9,fontweight="bold",color=INK)
 fig.legend(handles=[Line2D([0],[0],color=TEAL,lw=2.1,label="bcod-sim"),Line2D([0],[0],color=ORANGE,lw=1.8,ls="--",label="Gazebo Harmonic")],loc="lower center",ncol=2,frameon=False,fontsize=8.5);fig.text(.985,.055,"Natural scale per panel | full path extents shown | arrows indicate travel direction",ha="right",fontsize=7,color=GREY)
 output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=240,bbox_inches="tight",pad_inches=.08);fig.savefig(output.with_suffix(".svg"),bbox_inches="tight",pad_inches=.08);plt.close(fig);write_provenance(output,font,worst_case_callout,vehicle_a,vehicle_c,vehicle_c_generic);return output

def main():
 p=argparse.ArgumentParser();p.add_argument("--vehicle-a",type=Path,default=VEHICLE_A);p.add_argument("--vehicle-c",type=Path,default=VEHICLE_C);p.add_argument("--vehicle-c-generic",type=Path,default=VEHICLE_C_GENERIC);p.add_argument("--output",type=Path,default=OUTPUT);p.add_argument("--no-worst-case-callout",action="store_true");a=p.parse_args();print(build(a.vehicle_a,a.vehicle_c,a.output,worst_case_callout=not a.no_worst_case_callout,vehicle_c_generic=a.vehicle_c_generic))
if __name__=="__main__":main()
