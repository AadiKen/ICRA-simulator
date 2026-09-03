import json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];report=json.loads((ROOT/"artifacts/rl-campaign/long-horizon-parity.json").read_text());W,H=1100,480;colors=["#2563eb","#dc2626","#059669","#7c3aed","#ea580c"]
def panel(x0,key,title):
 out=[f'<rect x="{x0}" y="45" width="500" height="380" fill="white" stroke="#bbb"/>',f'<text x="{x0+250}" y="28" text-anchor="middle" font-size="16">{title}</text>']
 for tick in range(-18,2,3):
  y=405-(tick+18)/20*340;out+= [f'<line x1="{x0+45}" y1="{y}" x2="{x0+480}" y2="{y}" stroke="#ddd"/>',f'<text x="{x0+40}" y="{y+4}" text-anchor="end" font-size="10">1e{tick}</text>']
 for idx,cell in enumerate(report["cells"]):
  pts=[]
  for s in cell["log_samples"]:
   x=x0+45+math.log10(s["step"])/math.log10(2400)*435;v=max(s[key],1e-18);y=405-(math.log10(v)+18)/20*340;pts.append(f"{x:.1f},{max(65,min(405,y)):.1f}")
  out.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{colors[idx]}" stroke-width="2"/>')
 return out
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}"><rect width="100%" height="100%" fill="white"/>']+panel(35,"raw_max_abs","Literal predeclared raw-state divergence")+panel(565,"circular_yaw_physical_max_abs","Diagnostic using circular yaw distance")
for i,c in enumerate(report["cells"]):
 x=80+i*195;svg.append(f'<line x1="{x}" y1="445" x2="{x+20}" y2="445" stroke="{colors[i]}"/><text x="{x+25}" y="449" font-size="10">{c["initial_condition"]}</text>')
svg.append('<text x="550" y="470" text-anchor="middle" font-size="12">Step (log scale); maximum absolute divergence (log scale)</text></svg>');target=ROOT/"artifacts/rl-campaign/long-horizon-parity-log.svg";target.write_text("".join(svg));print(target)
