from __future__ import annotations
from pathlib import Path
from .common import COLORS, ROOT, load_json, svg_text, write_svg

DEFAULTS=[ROOT/"artifacts/environment-coverage/coverage-matrix.json",ROOT/"artifacts/environment-coverage/live-confirmatory-pass.json",ROOT/"artifacts/environment-coverage/gebco-live-confirmatory-pass.json",ROOT/"artifacts/environment-coverage/real-vs-idealized-trajectory.json"]
SOURCES=("ndbc","coops","nws","rtofs","era5","gebco")

def validate_inputs(coverage,live,gebco,pair):
    sites=coverage.get("methodology",{}).get("sites")
    if not isinstance(sites,list) or not sites: raise ValueError("Figure 2 requires coverage sites with coordinates")
    if any(not isinstance(s.get("latitude_deg"),(int,float)) or not isinstance(s.get("longitude_deg"),(int,float)) for s in sites): raise ValueError("Every Figure 2 coverage site requires latitude/longitude")
    statuses={r.get("source"):r.get("status") for r in live.get("results",[])}
    if not all(s in statuses for s in SOURCES[:-1]): raise ValueError("Live verification artifact lacks a requested source")
    if not gebco.get("results"): raise ValueError("GEBCO live verification results are missing")
    if pair.get("artifact_kind")!="paired-real-vs-idealized-environment-trajectory": raise ValueError("Figure 2 requires the paired real-vs-idealized trajectory artifact")
    if pair.get("scenario",{}).get("seed") is None or "Only environment.current_mps differs" not in pair.get("scenario",{}).get("invariant_between_runs",""): raise ValueError("Paired trajectories do not record the required controlled comparison")

def _wrap(text,width=145):
    lines=[];line=""
    for word in text.split():
        if line and len(line)+len(word)+1>width: lines.append(line);line=word
        else: line=f"{line} {word}".strip()
    return lines+[line]

def build(data_paths=None)->Path:
    paths=[Path(p) for p in (data_paths or DEFAULTS)]
    if len(paths)!=4: raise ValueError("Figure 2 requires coverage, live-source, live-GEBCO, and paired-trajectory artifacts")
    coverage,live,gebco,pair=[load_json(p) for p in paths];validate_inputs(coverage,live,gebco,pair)
    sites=coverage["methodology"]["sites"];status={r["source"]:r["status"] for r in live["results"]};status["gebco"]="success" if all(r.get("status")=="success" for r in gebco["results"]) else "partial"
    live_sites={(r["site"],r["source"]):r["status"] for r in live["results"]};live_sites.update({(r["site"],"gebco"):r["status"] for r in gebco["results"]})
    width,height=1120,650;out=[svg_text(24,34,"Geographic grounding and environmental response",size=22,weight=700),svg_text(24,58,"Coverage locations · retained live verification · controlled trajectory pair",size=13,fill=COLORS["muted"])]
    mx,my,mw,mh=30,100,520,360;out.append(f'<rect x="{mx}" y="{my}" width="{mw}" height="{mh}" rx="8" fill="#eef4f8" stroke="{COLORS["grid"]}"/>');out.append(svg_text(mx+12,my+25,"Coverage sites (schematic lon/lat projection)",size=13,weight=700));lons=[s["longitude_deg"] for s in sites];lats=[s["latitude_deg"] for s in sites]
    for site in sites:
        x=mx+45+(site["longitude_deg"]-min(lons))/(max(lons)-min(lons))*(mw-90);y=my+55+(max(lats)-site["latitude_deg"])/(max(lats)-min(lats))*(mh-100)
        out.extend([f'<circle cx="{x:.2f}" cy="{y:.2f}" r="8" fill="{COLORS["blue"]}" stroke="#fff" stroke-width="2"/>',svg_text(x+11,y+4,site["id"].replace("-"," "),size=11,weight=700),svg_text(x+11,y+20,f'{site["latitude_deg"]:.3f}, {site["longitude_deg"]:.3f}',size=9,fill=COLORS["muted"])])
        for j,source in enumerate(SOURCES):
            state=live_sites.get((site["id"],source));color=COLORS["identical"] if state=="success" else COLORS["divergence"] if state in ("partial","blocked") else COLORS["na"]
            out.append(f'<circle cx="{x+14+j*12:.2f}" cy="{y+32:.2f}" r="4" fill="{color}" stroke="#fff"/>')
    out.append(svg_text(30,490,"Per-site source order: NDBC · CO-OPS · NWS · RTOFS · ERA5 · GEBCO",size=11,weight=700))
    out.extend([f'<circle cx="37" cy="516" r="6" fill="{COLORS["identical"]}"/>',svg_text(49,520,"live verified",size=10),f'<circle cx="147" cy="516" r="6" fill="{COLORS["divergence"]}"/>',svg_text(159,520,"attempted: blocked/partial",size=10),f'<circle cx="330" cy="516" r="6" fill="{COLORS["na"]}"/>',svg_text(342,520,"fixture coverage only",size=10)])
    ix,iy,iw,ih=600,100,480,360;out.append(f'<rect x="{ix}" y="{iy}" width="{iw}" height="{ih}" rx="8" fill="#fbfcfe" stroke="{COLORS["grid"]}"/>');out.append(svg_text(ix+12,iy+25,f'Controlled inset · {pair["selection"]["selected_site"].replace("-"," ").title()}',size=13,weight=700));real=pair["conditions"]["real"]["samples"];ideal=pair["conditions"]["idealized"]["samples"];all_samples=real+ideal;ns=[p["north_m"] for p in all_samples];es=[p["east_m"] for p in all_samples];n0,n1,e0,e1=min(ns),max(ns),min(es),max(es);px,py,pw,ph=ix+45,iy+55,iw-75,ih-100
    def points(samples): return " ".join(f'{px+(p["east_m"]-e0)/max(e1-e0,1e-9)*pw:.2f},{py+ph-(p["north_m"]-n0)/max(n1-n0,1e-9)*ph:.2f}' for p in samples)
    out.extend([f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" fill="#fff" stroke="{COLORS["grid"]}"/>',f'<polyline points="{points(ideal)}" fill="none" stroke="{COLORS["muted"]}" stroke-width="3"/>',f'<polyline points="{points(real)}" fill="none" stroke="{COLORS["blue"]}" stroke-width="3"/>',svg_text(px,py+ph+22,"East displacement →",size=10,fill=COLORS["muted"]),svg_text(px-10,py-10,"North ↑",size=10,fill=COLORS["muted"]),f'<line x1="{ix+25}" y1="{iy+ih-22}" x2="{ix+48}" y2="{iy+ih-22}" stroke="{COLORS["blue"]}" stroke-width="3"/>',svg_text(ix+55,iy+ih-18,"retained live RTOFS current",size=10),f'<line x1="{ix+245}" y1="{iy+ih-22}" x2="{ix+268}" y2="{iy+ih-22}" stroke="{COLORS["muted"]}" stroke-width="3"/>',svg_text(ix+275,iy+ih-18,"idealized zero current",size=10)])
    out.append(svg_text(600,500,f'Selection score: {pair["selection"]["selected_score_mps"]:.4f} m/s · final separation: {pair["comparison"]["final_separation_m"]:.3f} m',size=12,weight=700))
    caption="Site selection maximizes the retained live horizontal-current magnitude relative to zero current among sites with a numeric live vector. Both production-core runs share seed 7319, scenario, action, timing, and initial state; only current differs. Map coordinates come from the coverage artifact; live-status markers come from retained confirmatory artifacts."
    for i,line in enumerate(_wrap(caption)): out.append(svg_text(30,580+i*17,line,size=10,fill=COLORS["muted"]))
    return write_svg("fig2_geography","\n".join(out),width,height,paths,caption)
