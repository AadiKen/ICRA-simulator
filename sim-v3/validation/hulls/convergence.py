#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
SPEC=json.loads((ROOT/"validation/hulls/hull-spec.json").read_text())
MESH_REPORT=ROOT/"validation/hulls/mesh-verification.json"
FREQUENCIES=(1.5,2.0)
DOFS=("Surge","Sway","Heave","Roll","Pitch","Yaw")

def solve(vehicle_id,label):
    import capytaine as cpt
    report=json.loads(MESH_REPORT.read_text()); entry=report["vehicles"][vehicle_id]["densities"][label]
    mesh_path=ROOT/entry["path"]; cache=ROOT/".cache/capytaine/parametric-hull-convergence"/vehicle_id
    cache.mkdir(parents=True,exist_ok=True); output=cache/f"{label}.json"
    if output.exists():
        cached=json.loads(output.read_text())
        if cached.get("mesh_sha256")==entry["mesh_sha256"] and cached.get("solver_mesh_policy")=="wetted-surface-lid-removed-v1": return cached
    spec=SPEC["vehicles"][vehicle_id]; zg=spec["KG_m"]-spec["T"]
    closed_mesh=cpt.load_mesh(str(mesh_path),name=f"{vehicle_id}-{label}-closed")
    # The generated artifact is watertight. Capytaine reconstructs the
    # waterplane from the z=0 boundary; its solver mesh must omit that lid.
    mesh=closed_mesh.extract_faces(np.where(closed_mesh.faces_centers[:,2] < -1e-10)[0],name=f"{vehicle_id}-{label}-wetted")
    body=cpt.FloatingBody(mesh=mesh,center_of_mass=(0,0,zg),name=f"{vehicle_id}-{label}"); body.add_all_rigid_body_dofs()
    problems=[cpt.RadiationProblem(body=body,omega=omega,water_depth=math.inf,rho=1025,g=9.80665,radiating_dof=dof) for omega in FREQUENCIES for dof in DOFS]
    dataset=cpt.assemble_dataset(cpt.BEMSolver().solve_all(problems,n_jobs=1))
    frequencies=[]
    for omega in FREQUENCIES:
        added=dataset["added_mass"].sel(omega=omega); damping=dataset["radiation_damping"].sel(omega=omega)
        matrix=lambda data:[[float(data.sel(radiating_dof=r,influenced_dof=i)) for r in DOFS] for i in DOFS]
        frequencies.append({"omega_rad_s":omega,"added_mass":matrix(added),"radiation_damping":matrix(damping)})
    hydro=body.compute_hydrostatic_stiffness(rho=1025,g=9.80665)
    volume=spec["expected"]["volume_m3"]; denominator=1025*9.80665*volume
    result={"vehicle_id":vehicle_id,"density":label,"mesh_sha256":entry["mesh_sha256"],"solver_mesh_policy":"wetted-surface-lid-removed-v1","closed_panels_total":closed_mesh.nb_faces,"panels_total":mesh.nb_faces,"panels_per_hull":entry["actual_panels_per_hull"],"capytaine_version":cpt.__version__,"frequencies":frequencies,"capytaine_hydrostatics":{"GM_T_m":float(hydro.sel(influenced_dof="Roll",radiating_dof="Roll"))/denominator,"GM_L_m":float(hydro.sel(influenced_dof="Pitch",radiating_dof="Pitch"))/denominator}}
    output.write_text(json.dumps(result,indent=2)+"\n"); return result

def relative_norm(a,b):
    numerator=math.sqrt(sum((x-y)**2 for row_a,row_b in zip(a,b) for x,y in zip(row_a,row_b)))
    denominator=math.sqrt(sum(y*y for row in b for y in row))
    return numerator/denominator if denominator else numerator

def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--vehicle",choices=list(SPEC["vehicles"])); parser.add_argument("--density",choices=["p0800","p1600","p3200","p6400"]); args=parser.parse_args()
    vehicles=[args.vehicle] if args.vehicle else list(SPEC["vehicles"]); labels=[args.density] if args.density else ["p0800","p1600","p3200","p6400"]
    for vehicle in vehicles:
        for label in labels:
            result=solve(vehicle,label); print(json.dumps({"vehicle":vehicle,"density":label,"panels":result["panels_total"]}))
    if args.vehicle or args.density: return
    results={vehicle:[solve(vehicle,label) for label in labels] for vehicle in vehicles}
    artifact={"schema_version":1,"artifact_kind":"parametric-hull-capytaine-convergence","frequencies_rad_s":list(FREQUENCIES),"frequency_basis":"4 s regular-wave operating point and its interpolation bracket","chosen_density":"p6400","is_validation_evidence":False,"validation_status_unchanged":True,"vehicles":{}}
    for vehicle,rows in results.items():
        finest=rows[-1]; comparisons=[]
        for row in rows:
            frequencies=[]
            for current,target in zip(row["frequencies"],finest["frequencies"]): frequencies.append({"omega_rad_s":current["omega_rad_s"],"added_mass_relative_frobenius_error":relative_norm(current["added_mass"],target["added_mass"]),"radiation_damping_relative_frobenius_error":relative_norm(current["radiation_damping"],target["radiation_damping"])})
            comparisons.append({"density":row["density"],"panels_total":row["panels_total"],"panels_per_hull":row["panels_per_hull"],"frequencies":frequencies})
        previous=rows[-2]; residual=[]
        for current,prior in zip(finest["frequencies"],previous["frequencies"]): residual.append({"omega_rad_s":current["omega_rad_s"],"added_mass_relative_change":relative_norm(prior["added_mass"],current["added_mass"]),"radiation_damping_relative_change":relative_norm(prior["radiation_damping"],current["radiation_damping"])})
        spec=SPEC["vehicles"][vehicle]["expected"]
        cap=finest["capytaine_hydrostatics"]
        artifact["vehicles"][vehicle]={"densities":comparisons,"chosen_density_residual":residual,"hydrostatics_no_overwrite":{"analytic":{"GM_T_m":spec["GM_T_m"],"GM_L_m":spec["GM_L_m"]},"capytaine_mesh_derived":cap,"signed_delta":{"GM_T_m":cap["GM_T_m"]-spec["GM_T_m"],"GM_L_m":cap["GM_L_m"]-spec["GM_L_m"]},"resolution_policy":"report-both-no-automatic-overwrite"}}
    output=ROOT/"validation/hulls/capytaine-convergence.json"; output.write_text(json.dumps(artifact,indent=2)+"\n"); print(json.dumps({"output":str(output)},indent=2))

if __name__=="__main__": main()
