#!/usr/bin/env python3
"""Generate potential-flow coefficients with Capytaine.

This tool produces solver output, not validation evidence.  Viscous damping is
deliberately absent from the artifact and must be supplied independently.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
DOFS = ("Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw")
SIGN = (1.0, -1.0, -1.0, 1.0, -1.0, -1.0)  # Capytaine -> body NED


def load_config(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    validate_config(value, path.parent)
    return value


def validate_config(value: dict[str, Any], base: Path, require_mesh: bool = True) -> None:
    if value.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("schema_version must be 1")
    mesh = value.get("mesh", {})
    if mesh.get("coordinate_system") != "capytaine_x_forward_y_left_z_up":
        raise ValueError("mesh.coordinate_system must use Capytaine's x-forward/y-left/z-up convention")
    if "path" in mesh:
        mesh_path = base / str(mesh.get("path", ""))
        if require_mesh and (not mesh_path.is_file() or mesh_path.is_symlink()):
            raise ValueError(f"mesh must be an existing regular file: {mesh_path}")
    elif mesh.get("generator") == "parallelepiped":
        for key in ("size_m", "center_m", "resolution"):
            values = mesh.get(key)
            if not isinstance(values, list) or len(values) != 3 or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
                raise ValueError(f"mesh.{key} must contain three finite values")
        if any(x <= 0 for x in mesh["size_m"]) or any(not isinstance(x, int) or x < 2 for x in mesh["resolution"]):
            raise ValueError("parametric mesh size must be positive and resolution entries must be integers >= 2")
        if mesh.get("provenance", {}).get("status") not in {"bootstrap", "reviewed"}:
            raise ValueError("parametric mesh provenance must explicitly identify bootstrap or reviewed status")
    else:
        raise ValueError("mesh must provide a file path or supported parametric generator")
    frequencies = value.get("frequency_grid_rad_s")
    if not isinstance(frequencies, list) or len(frequencies) < 2:
        raise ValueError("frequency_grid_rad_s must contain at least two samples")
    if any(not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0 for x in frequencies):
        raise ValueError("frequencies must be finite and positive")
    if any(b <= a for a, b in zip(frequencies, frequencies[1:])):
        raise ValueError("frequencies must be strictly increasing")
    headings = value.get("wave_heading_grid_rad")
    if not isinstance(headings, list) or not headings:
        raise ValueError("wave_heading_grid_rad must be non-empty")
    if any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in headings):
        raise ValueError("headings must be finite")
    for key in ("water_density_kg_m3", "gravity_m_s2"):
        if not isinstance(value.get(key), (int, float)) or value[key] <= 0:
            raise ValueError(f"{key} must be positive")
    comparison = value.get("hydrostatics_comparison", {})
    for key in ("reference_displaced_volume_m3", "geometry_bootstrap_gm_transverse_m", "geometry_bootstrap_gm_longitudinal_m"):
        if not isinstance(comparison.get(key), (int, float)) or not math.isfinite(comparison[key]) or comparison[key] <= 0:
            raise ValueError(f"hydrostatics_comparison.{key} must be finite and positive")
    if value.get("water_depth_m") != "infinite" and (not isinstance(value.get("water_depth_m"), (int, float)) or value["water_depth_m"] <= 0):
        raise ValueError("water_depth_m must be positive or 'infinite'")


def transform_matrix(matrix: list[list[float]]) -> list[list[float]]:
    return [[SIGN[i] * float(matrix[i][j]) * SIGN[j] for j in range(6)] for i in range(6)]


def transform_vector(vector: list[complex]) -> list[list[float]]:
    return [[SIGN[i] * float(v.real), SIGN[i] * float(v.imag)] for i, v in enumerate(vector)]


def hydrostatics_comparison(config: dict[str, Any], stiffness: list[list[float]]) -> dict[str, Any]:
    comparison = config["hydrostatics_comparison"]
    volume = float(comparison["reference_displaced_volume_m3"])
    denominator = float(config["water_density_kg_m3"]) * float(config["gravity_m_s2"]) * volume
    capytaine = {
        "gm_transverse_m": stiffness[3][3] / denominator,
        "gm_longitudinal_m": stiffness[4][4] / denominator,
    }
    bootstrap = {
        "gm_transverse_m": float(comparison["geometry_bootstrap_gm_transverse_m"]),
        "gm_longitudinal_m": float(comparison["geometry_bootstrap_gm_longitudinal_m"]),
    }
    return {
        "reference_displaced_volume_m3": volume,
        "geometry_bootstrap": bootstrap,
        "capytaine_mesh_derived": capytaine,
        "capytaine_minus_bootstrap": {key: capytaine[key] - bootstrap[key] for key in bootstrap},
        "resolution_policy": "report-both-no-automatic-overwrite",
    }


def _matrix(dataset: Any, variable: str, omega: float) -> list[list[float]]:
    selected = dataset[variable].sel(omega=omega)
    return [[float(selected.sel(radiating_dof=r, influenced_dof=i)) for r in DOFS] for i in DOFS]


def build_mesh(config: dict[str, Any], base: Path, cpt: Any) -> tuple[Any, str, dict[str, Any]]:
    specification = config["mesh"]
    if "path" in specification:
        mesh_path = (base / specification["path"]).resolve()
        mesh = cpt.load_mesh(str(mesh_path), name=config["vehicle_id"])
        if specification.get("remove_waterplane_lid", False):
            import numpy as np
            mesh = mesh.extract_faces(np.where(mesh.faces_centers[:, 2] < -1e-10)[0], name=config["vehicle_id"])
        checksum = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
        return mesh, checksum, {"kind": "analytic-parametric-file", "path": specification["path"], "format": specification["format"], "solver_mesh_policy": "wetted-surface-lid-removed-v1" if specification.get("remove_waterplane_lid", False) else "as-loaded", "promotion_blocked": True}
    mesh = cpt.mesh_parallelepiped(size=tuple(specification["size_m"]), center=tuple(specification["center_m"]), resolution=tuple(specification["resolution"]), missing_sides={"top"}, name=config["vehicle_id"])
    digest = hashlib.sha256()
    topology = "wetted-open-top-v1"
    digest.update(json.dumps({**{key: specification[key] for key in ("generator", "size_m", "center_m", "resolution")}, "topology": topology}, sort_keys=True, separators=(",", ":")).encode())
    return mesh, digest.hexdigest(), {"kind": "parametric-bootstrap", "topology": topology, "specification": specification, "promotion_blocked": specification["provenance"]["status"] != "reviewed"}


def generate(config_path: Path, output_path: Path) -> None:
    config = load_config(config_path)
    base = config_path.parent
    os.environ.setdefault("CAPYTAINE_CACHE_DIR", str((base / ".cache").resolve()))
    import capytaine as cpt

    mesh, mesh_checksum, mesh_provenance = build_mesh(config, base, cpt)
    body = cpt.FloatingBody(mesh=mesh, center_of_mass=tuple(config["mesh"]["center_of_mass_m"]), name=config["vehicle_id"])
    body.add_all_rigid_body_dofs()
    depth = math.inf if config["water_depth_m"] == "infinite" else float(config["water_depth_m"])
    rho, gravity = float(config["water_density_kg_m3"]), float(config["gravity_m_s2"])
    problems = []
    for omega in config["frequency_grid_rad_s"]:
        for dof in DOFS:
            problems.append(cpt.RadiationProblem(body=body, omega=omega, water_depth=depth, rho=rho, g=gravity, radiating_dof=dof))
        for heading in config["wave_heading_grid_rad"]:
            problems.append(cpt.DiffractionProblem(body=body, omega=omega, water_depth=depth, rho=rho, g=gravity, wave_direction=heading))
    dataset = cpt.assemble_dataset(cpt.BEMSolver().solve_all(problems, n_jobs=int(config.get("solver", {}).get("jobs", 1))))
    frequencies = []
    for omega in config["frequency_grid_rad_s"]:
        frequencies.append({
            "omega_rad_s": omega,
            "added_mass": transform_matrix(_matrix(dataset, "added_mass", omega)),
            "radiation_damping": transform_matrix(_matrix(dataset, "radiation_damping", omega)),
        })
    excitation = []
    for omega in config["frequency_grid_rad_s"]:
        for heading in config["wave_heading_grid_rad"]:
            values = dataset["excitation_force"].sel(omega=omega, wave_direction=heading)
            excitation.append({"omega_rad_s": omega, "heading_rad": -heading,
                               "complex_force": transform_vector([complex(values.sel(influenced_dof=d)) for d in DOFS])})
    hydro = body.compute_hydrostatic_stiffness(rho=rho, g=gravity)
    hydro_matrix = [[float(hydro.sel(influenced_dof=i, radiating_dof=r)) for r in DOFS] for i in DOFS]
    transformed_hydro = transform_matrix(hydro_matrix)
    artifact = {
        "schema_version": 1,
        "artifact_kind": "capytaine-potential-flow-solver-output",
        "is_validation_evidence": False,
        "capytaine_version": cpt.__version__,
        "vehicle_id": config["vehicle_id"],
        "mesh_checksum_sha256": mesh_checksum,
        "mesh_provenance": mesh_provenance,
        "input_config_checksum_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "water_density_kg_m3": rho, "gravity_m_s2": gravity,
        "coordinate_system": "body_x_forward_y_starboard_z_down",
        "frequencies": frequencies, "hydrostatic_stiffness": transformed_hydro,
        "hydrostatics_comparison": hydrostatics_comparison(config, transformed_hydro),
        "wave_excitation": excitation,
        "provenance": {"role": "potential-flow-only", "excludes": ["skin friction", "eddy-making", "lift", "appendages", "viscous roll damping"]},
        "limitations": ["No viscous damping.", "No experimental validation claim.", "Frequency-domain coefficients require a documented time-domain approximation or radiation-memory model.", *( (["Representative analytic parametric geometry is not physical-vessel validation."] if mesh_provenance.get("kind") == "analytic-parametric-file" else ["Parametric bootstrap mesh is not reviewed hydrodynamic geometry."]) if mesh_provenance.get("promotion_blocked") else [] )],
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text(json.dumps(artifact, indent=2) + "\n")
    temporary.replace(output_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    generate(args.config.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
