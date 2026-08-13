#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC_PATH = ROOT / "validation/hulls/hull-spec.json"
DENSITIES = {"p0800": (20, 10), "p1600": (28, 14), "p3200": (40, 20), "p6400": (56, 28)}


def analytic(spec):
    bx = math.gamma(0.5) * math.gamma(spec["a"] + 1) / math.gamma(spec["a"] + 1.5)
    bz = math.gamma(0.5) * math.gamma(spec["b"] + 1) / math.gamma(spec["b"] + 1.5)
    one_volume = spec["L"] * spec["B"] * spec["T"] * bx * bz / 4
    one_awp = spec["L"] * spec["B"] * bx / 2
    n = len(spec["hull_offsets_y_m"])
    volume, awp = n * one_volume, n * one_awp
    kb = spec["T"] * (1 - 1 / ((spec["b"] + 1) * bz))
    # Waterplane moments are evaluated deterministically by high-order midpoint quadrature.
    count = 200000
    dx = spec["L"] / count
    it_one = il_one = 0.0
    for index in range(count):
        x = -spec["L"] / 2 + (index + 0.5) * dx
        half = spec["B"] / 2 * max(0.0, 1 - (2 * x / spec["L"]) ** 2) ** spec["a"]
        it_one += (2 * half ** 3 / 3) * dx
        il_one += (2 * half * x * x) * dx
    it = sum(it_one + one_awp * offset * offset for offset in spec["hull_offsets_y_m"])
    il = n * il_one
    bm_t, bm_l = it / volume, il / volume
    return {"Cb":one_volume/(spec["L"]*spec["B"]*spec["T"]),"Cwp":one_awp/(spec["L"]*spec["B"]),"volume_m3":volume,"waterplane_area_m2":awp,"KB_m":kb,"BM_T_m":bm_t,"BM_L_m":bm_l,"GM_T_m":kb+bm_t-spec["KG_m"],"GM_L_m":kb+bm_l-spec["KG_m"]}


def add_vertex(vertices, lookup, point):
    key = tuple(round(value, 14) for value in point)
    if key not in lookup:
        lookup[key] = len(vertices)
        vertices.append(tuple(float(value) for value in point))
    return lookup[key]


def mesh(spec, nx, nz):
    vertices, lookup, faces = [], {}, []
    s_values = [-1 + 2 * index / nx for index in range(nx + 1)]
    x_values = [spec["L"] / 2 * math.sin(math.pi * s / 2) for s in s_values]
    v_values = [index / nz for index in range(nz + 1)]
    z_values = [-spec["T"] * (1 - (1 - v) ** 2) for v in v_values]
    for offset in spec["hull_offsets_y_m"]:
        grid = {}
        for side in (-1, 1):
            for ix, x in enumerate(x_values):
                fx = max(0.0, 1 - (2*x/spec["L"])**2) ** spec["a"]
                for iz, z in enumerate(z_values):
                    fz = max(0.0, 1 - (z/spec["T"])**2) ** spec["b"]
                    # Spec y-starboard is reflected into Capytaine y-left.
                    y_left = -(offset + side * spec["B"] / 2 * fx * fz)
                    grid[side, ix, iz] = add_vertex(vertices, lookup, (x, y_left, z))
        for side in (-1, 1):
            for ix in range(nx):
                for iz in range(nz):
                    q = [grid[side, ix, iz], grid[side, ix+1, iz], grid[side, ix+1, iz+1], grid[side, ix, iz+1]]
                    faces.extend([(q[0],q[1],q[2]),(q[0],q[2],q[3])])
        for ix in range(nx):
            q = [grid[-1,ix,0],grid[1,ix,0],grid[1,ix+1,0],grid[-1,ix+1,0]]
            faces.extend([(q[0],q[1],q[2]),(q[0],q[2],q[3])])
    clean = []
    seen_faces = set()
    interior = (0.0, 0.0, -spec["T"]/2)
    for face in faces:
        a,b,c = (vertices[index] for index in face)
        ab=tuple(b[i]-a[i] for i in range(3)); ac=tuple(c[i]-a[i] for i in range(3))
        normal=(ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0])
        area=math.sqrt(sum(value*value for value in normal))/2
        centre=tuple((a[i]+b[i]+c[i])/3 for i in range(3))
        nearest_offset=min(spec["hull_offsets_y_m"],key=lambda offset:abs(-centre[1]-offset))
        if area < 1e-12 or tuple(sorted(face)) in seen_faces or all(abs(-point[1]-nearest_offset)<1e-12 for point in (a,b,c)): continue
        local_interior=(0.0,-nearest_offset,-spec["T"]/2)
        if sum(normal[i]*(centre[i]-local_interior[i]) for i in range(3)) < 0: face=(face[0],face[2],face[1])
        seen_faces.add(tuple(sorted(face)))
        clean.append(face)
    return vertices, clean


def triangle_volume(vertices, face):
    a,b,c=(vertices[i] for i in face)
    cross=(b[1]*c[2]-b[2]*c[1],b[2]*c[0]-b[0]*c[2],b[0]*c[1]-b[1]*c[0])
    return sum(a[i]*cross[i] for i in range(3))/6


def verify(vertices, faces, spec):
    edge_counts={}
    volume=0.0; moment=[0.0,0.0,0.0]; awp=it=il=0.0; minimum_area=math.inf
    for face in faces:
        for a,b in zip(face,(face[1],face[2],face[0])): edge_counts[tuple(sorted((a,b)))]=edge_counts.get(tuple(sorted((a,b))),0)+1
        points=[vertices[i] for i in face]; a,b,c=points
        ab=[b[i]-a[i] for i in range(3)]; ac=[c[i]-a[i] for i in range(3)]
        cross=[ab[1]*ac[2]-ab[2]*ac[1],ab[2]*ac[0]-ab[0]*ac[2],ab[0]*ac[1]-ab[1]*ac[0]]
        minimum_area=min(minimum_area,math.sqrt(sum(v*v for v in cross))/2)
        dv=triangle_volume(vertices,face); volume+=dv
        for i in range(3): moment[i]+=dv*sum(point[i] for point in points)/4
        if all(abs(point[2])<1e-12 for point in points):
            area=abs((b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]))/2
            awp+=area
            xs=[p[0] for p in points]; ys=[p[1] for p in points]
            il+=area/6*(sum(x*x for x in xs)+xs[0]*xs[1]+xs[1]*xs[2]+xs[2]*xs[0])
            it+=area/6*(sum(y*y for y in ys)+ys[0]*ys[1]+ys[1]*ys[2]+ys[2]*ys[0])
    if volume < 0: raise ValueError("mesh normals are inward")
    if any(count != 2 for count in edge_counts.values()): raise ValueError(f"mesh is not watertight: {sum(count!=2 for count in edge_counts.values())} open/nonmanifold edges")
    if minimum_area <= 1e-12: raise ValueError("mesh contains a degenerate panel")
    zcb=moment[2]/volume; kb=zcb+spec["T"]
    n=len(spec["hull_offsets_y_m"])
    result={"Cb":volume/(n*spec["L"]*spec["B"]*spec["T"]),"Cwp":awp/(n*spec["L"]*spec["B"]),"volume_m3":volume,"waterplane_area_m2":awp,"KB_m":kb,"BM_T_m":it/volume,"BM_L_m":il/volume,"GM_T_m":kb+it/volume-spec["KG_m"],"GM_L_m":kb+il/volume-spec["KG_m"]}
    return result,{"watertight":True,"outward_normals":True,"minimum_panel_area_m2":minimum_area,"vertices":len(vertices),"panels":len(faces)}


def write_mar(path, vertices, faces):
    lines=["2 0"]
    lines.extend(f"{i+1} {x:.12e} {y:.12e} {z:.12e}" for i,(x,y,z) in enumerate(vertices)); lines.append("0 0 0 0")
    lines.extend(f"{a+1} {b+1} {c+1} {c+1}" for a,b,c in faces); lines.append("0 0 0 0")
    path.write_text("\n".join(lines)+"\n")


def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--output",type=Path,default=ROOT/"artifacts/generated/hulls"); args=parser.parse_args()
    source=json.loads(SPEC_PATH.read_text())
    if source["frame"]!={"origin":"midship-centreline-design-waterline","x":"forward","y":"starboard","z":"up"}: raise ValueError("Hull frame convention mismatch")
    prose_spec=ROOT/"validation/hulls/vehicle-hull-specification.md"
    report={"schema_version":1,"artifact_kind":"parametric-wigley-mesh-verification","source":str(prose_spec.relative_to(ROOT)),"source_sha256":hashlib.sha256(prose_spec.read_bytes()).hexdigest(),"machine_spec":str(SPEC_PATH.relative_to(ROOT)),"machine_spec_sha256":hashlib.sha256(SPEC_PATH.read_bytes()).hexdigest(),"frame_assertion":"midship/centreline/DWL x-forward y-starboard z-up; generated solver mesh reflects y to Capytaine y-left","vehicles":{}}
    for vehicle_id,spec in source["vehicles"].items():
        analytic_values=analytic(spec); expected=spec["expected"]
        for key,target in expected.items():
            if abs(analytic_values[key]-target)/abs(target)>0.005: raise ValueError(f"{vehicle_id} analytic {key} disagrees with specification: {analytic_values[key]} vs {target}")
        vehicle={"analytic":analytic_values,"spec_table":expected,"densities":{}}
        directory=args.output/vehicle_id; directory.mkdir(parents=True,exist_ok=True)
        for label,(nx,nz) in DENSITIES.items():
            vertices,faces=mesh(spec,nx,nz); mesh_values,quality=verify(vertices,faces,spec); output=directory/f"{vehicle_id}-{label}.mar"; write_mar(output,vertices,faces)
            vehicle["densities"][label]={"requested_panels_per_hull":int(label[1:]),"nx":nx,"nz":nz,"actual_panels_total":len(faces),"actual_panels_per_hull":len(faces)//len(spec["hull_offsets_y_m"]),"mesh_sha256":hashlib.sha256(output.read_bytes()).hexdigest(),"hydrostatics":mesh_values,"quality":quality,"path":str(output.relative_to(ROOT))}
        finest=vehicle["densities"]["p6400"]["hydrostatics"]
        vehicle["finest_relative_error"]={key:(finest[key]-expected[key])/expected[key] for key in expected}
        if any(abs(value)>0.03 for value in vehicle["finest_relative_error"].values()): raise ValueError(f"{vehicle_id} finest mesh hydrostatics exceed 3%: {vehicle['finest_relative_error']}")
        report["vehicles"][vehicle_id]=vehicle
    report_path=ROOT/"validation/hulls/mesh-verification.json"; report_path.write_text(json.dumps(report,indent=2)+"\n"); print(json.dumps({"report":str(report_path),"vehicles":{key:{label:value["actual_panels_per_hull"] for label,value in vehicle["densities"].items()} for key,vehicle in report["vehicles"].items()}},indent=2))


if __name__=="__main__": main()
