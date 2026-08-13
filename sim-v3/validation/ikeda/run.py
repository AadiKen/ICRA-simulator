#!/usr/bin/env python3
"""Zero-speed Ikeda-style lower-bound roll damping for the analytic hulls.

Friction follows Kato/Tamiya as reported by Ikeda and the sectional eddy
calculation follows Ikeda's Lewis-form method. Potential wave-making damping is
read from Capytaine and is never added to the viscous subtotal. Bilge-keel and
forward-speed lift terms are zero by construction for this scoped lower bound.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RHO = 1025.0
NU = 1.19e-6
PHI_A = math.radians(10.0)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def section_shape(spec: dict, count: int = 401):
    x = np.linspace(-spec["L"] / 2 * (1 - 1e-5), spec["L"] / 2 * (1 - 1e-5), count)
    fx = np.maximum(0.0, 1 - (2 * x / spec["L"]) ** 2) ** spec["a"]
    beam = spec["B"] * fx
    vertical_integral = math.gamma(0.5) * math.gamma(spec["b"] + 1) / (2 * math.gamma(spec["b"] + 1.5))
    area = beam * spec["T"] * vertical_integral
    sigma = np.divide(area, beam * spec["T"], out=np.zeros_like(area), where=beam > 0)
    h = beam / (2 * spec["T"])
    c1 = (3 + 4 * sigma / math.pi) + (1 - 4 * sigma / math.pi) * ((h - 1) / (h + 1)) ** 2
    a3 = (-c1 + 3 + np.sqrt(np.maximum(0, 9 - 2 * c1))) / c1
    a1 = (1 + a3) * (h - 1) / (h + 1)
    return x, beam, area, sigma, h, a1, a3


def friction(spec: dict, omega: float, wetted_surface: float) -> float:
    og = spec["T"] - spec["KG_m"]  # positive downward, as in Ikeda
    cb = spec["expected"]["Cb"]
    rf = ((0.887 + 0.145 * cb) * (wetted_surface / spec["L"]) - 2 * og) / math.pi
    if rf <= 0:
        raise ValueError("nonphysical Ikeda friction radius")
    base = 0.787 * RHO * wetted_surface * rf**2 * math.sqrt(omega * NU)
    return base * (1 + 0.00814 * (rf**2 * PHI_A**2 * omega / NU) ** 0.386)


def eddy(spec: dict, omega: float) -> tuple[float, dict]:
    x, beam, area, sigma, h0, a1, a3 = section_shape(spec)
    draft = spec["T"]
    og = draft - spec["KG_m"]
    m = beam / (2 * (1 + a1 + a3))
    factor = np.divide(a1 * (1 + a3), 4 * a3, out=np.zeros_like(a1), where=np.abs(a3) > 1e-12)
    psi2 = 0.5 * np.arccos(np.clip(factor, -1, 1))

    def rmax(psi):
        return m * np.sqrt(((1 + a1) * np.sin(psi) - a3 * np.sin(3 * psi)) ** 2 + ((1 - a1) * np.cos(psi) + a3 * np.cos(3 * psi)) ** 2)

    r0, r2 = rmax(np.zeros_like(x)), rmax(psi2)
    psi = np.where(r0 >= r2, 0.0, psi2)
    r = rmax(psi)
    aa = (-2*a3*np.cos(5*psi) + a1*(1-a3)*np.cos(3*psi) + ((6-3*a1)*a3**2 + (a1**2-3*a1)*a3 + a1**2)*np.cos(psi))
    bb = (-2*a3*np.sin(5*psi) + a1*(1-a3)*np.sin(3*psi) + ((6+3*a1)*a3**2 + (3*a1+a1**2)*a3 + a1**2)*np.sin(psi))
    hh = 1 + a1**2 + 9*a3**2 + 2*a1*(1-3*a3)*np.cos(2*psi) - 6*a3*np.cos(4*psi)
    f1 = 0.5 * (1 + np.tanh(20 * (sigma - 0.7)))
    f2 = 0.5 * (1 - np.cos(math.pi * sigma)) - 1.5 * (1 - np.exp(-5 * (1 - sigma))) * np.sin(math.pi * sigma) ** 2
    f3 = 1 + 4 * np.exp(-1.65e5 * (1 - sigma) ** 2)
    hprime = h0 * draft / (draft - og)
    sprime = (sigma * draft - og) / (draft - og)
    gamma = np.sqrt(math.pi) * f3 * (r + 2*m/np.maximum(hh,1e-12)*np.sqrt(aa**2+bb**2)) / (2*draft*(1-og/draft)*np.sqrt(np.maximum(hprime*sprime,1e-12)))
    cp = 0.5 * (0.87*np.exp(-gamma) - 4*np.exp(-0.187*gamma) + 3)
    rb = 2*draft*np.sqrt(np.maximum(0, h0*(sigma-1)/(math.pi-4)))
    rb = np.where((h0 >= 1) & (rb/draft > 1), draft, rb)
    rb = np.where((h0 < 1) & (rb/draft > h0), beam/2, rb)
    mre = 0.5*RHO*r**2*draft**2*cp*((1-f1*rb/draft)*(1-og/draft-f1*rb/draft) + f2*(h0-f1*rb/draft)**2)
    cr = mre / (0.5*RHO*draft**4)
    per_length = 4/(3*math.pi)*draft**4*omega*PHI_A*cr
    one_hull = float(np.trapezoid(per_length, x))
    count = len(spec["hull_offsets_y_m"])
    return count * one_hull, {"sections_per_hull":len(x),"section_area_coefficient":float(sigma[len(sigma)//2]),"demihull_count":count}


def interpolate_roll_radiation(resolved: dict) -> tuple[float, float]:
    roll = resolved["free_decay"]["roll"]["hydrodynamics"]
    return roll["evaluation_frequency_rad_s"], roll["damping"]["potentialRadiationDamping"][1][1]


def main():
    hull_path = ROOT / "validation/hulls/hull-spec.json"
    hulls = json.loads(hull_path.read_text())
    output = {"schema_version":1,"artifact_kind":"ikeda-zero-speed-lower-bound","status":"software-estimate-not-physical-validation","method_scope":"Kato-Tamiya friction plus Ikeda sectional eddy damping at zero speed; Capytaine radiation reported separately; no bilge keel or lift.","source_checksums":{"hull_spec_sha256":sha(hull_path),"ikeda_himeno_tanaka_1978_sha256":sha(ROOT/"validation/external-references/kvlcc2/cache/related-work/Ikeda_Himeno_Tanaka_1978_Roll_Damping.pdf")},"reference_implementation":{"repository":"https://github.com/martinlarsalbert/rolldecay-estimators","commit":"c74642ce2b5299d4aa849c277fbaf8688f79f760","license":"BSD-3-Clause","license_sha256":"29e87305af8bb7f1b3991b1f100cb960e93a29902ab3c173bcedf9174d2ebdc7","use":"Equations independently adapted into this tracked implementation; runtime does not depend on the repository."},"assumptions":{"water_density_kg_m3":RHO,"kinematic_viscosity_m2_s":NU,"roll_amplitude_rad":PHI_A,"forward_speed_mps":0,"bilge_keel_component":0,"lift_component":0,"radiation_double_counted":False},"vehicles":{}}
    for vehicle, spec in hulls["vehicles"].items():
        resolved_path = ROOT / f"artifacts/capytaine/{'vehicle-b' if vehicle == 'vehicle-b-rudder' else 'vehicle-c'}-parametric-resolved.json"
        resolved = json.loads(resolved_path.read_text())
        omega, radiation = interpolate_roll_radiation(resolved)
        mesh = json.loads((ROOT/"validation/hulls/mesh-verification.json").read_text())["vehicles"][vehicle]["densities"]["p6400"]
        # Closed-mesh area includes the waterplane lid; subtract it for wetted area.
        # Deterministic analytic approximation used by the reference implementation.
        wetted_one = spec["L"] * (1.7*spec["T"] + spec["expected"]["Cb"]*spec["B"])
        wetted = len(spec["hull_offsets_y_m"]) * wetted_one
        bf = friction(spec, omega, wetted)
        be, sectional = eddy(spec, omega)
        quadratic_eddy = 3*math.pi*be/(8*omega*PHI_A)
        output["vehicles"][vehicle] = {"evaluation_frequency_rad_s":omega,"components_Nms_per_rad":{"potential_radiation":radiation,"friction_equivalent_linear":bf,"eddy_equivalent_linear":be,"bilge_keel":0,"lift":0},"viscous_linear_roll_Nms_per_rad":bf,"viscous_quadratic_roll_Nms2_per_rad2":quadratic_eddy,"combined_equivalent_linear_Nms_per_rad":radiation+bf+be,"sectional_geometry":sectional,"wetted_surface_approximation_m2":wetted,"provenance":{"potential_radiation":"Capytaine parametric-hull solve","friction":"Kato-Tamiya/Ikeda engineering estimate","eddy":"Ikeda Lewis-form sectional integration","geometry":"analytic generalized Wigley design assumption"},"limitations":["Representative design geometry is not measured.","No bilge keel is specified, so its component is zero.","Zero-speed lift is zero.","The result is an unvalidated lower-bound estimate, not free-decay validation.","Potential wave-making/radiation damping is supplied by Capytaine and excluded from the viscous subtotal."]}
    destination = ROOT / "artifacts/ikeda/roll-damping-lower-bound.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(output, indent=2)+"\n")
    print(json.dumps({"output":str(destination),"vehicles":output["vehicles"]},indent=2))


if __name__ == "__main__":
    main()
