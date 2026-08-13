# Capytaine coefficient preparation

This directory defines the reproducible input contract and generator for production potential-flow coefficients. It does **not** contain an approved production hull mesh or experimental validation evidence.

Copy `example-input.json`, provide a licensed wetted-hull mesh in Capytaine's x-forward/y-left/z-up frame, record the real center of mass, and have the frequency and heading grids reviewed before generation. Inputs must be immutable for an evidence campaign. Do not commit a restricted mesh merely to make the example run.

Run:

```sh
CAPYTAINE_CACHE_DIR=.cache/capytaine .venv/bin/python validation/capytaine/generate.py \
  validation/capytaine/vehicle-b.json artifacts/capytaine/vehicle-b.json

.venv/bin/python -m unittest discover -s validation/capytaine -p 'test_*.py'
```

The generator calculates added mass, radiation damping, diffraction plus Froude–Krylov excitation, and hydrostatic stiffness. It converts Capytaine axes to BCOD's body NED convention and records mesh/config checksums. The artifact reports mesh-derived `GM_T`/`GM_L` beside the geometry-derived bootstrap and their signed deltas. It never silently selects or overwrites either value: disagreement is a review signal about mesh, displacement, center-of-mass, or bootstrap assumptions. Its artifact is deliberately marked `is_validation_evidence: false`; promotion requires mesh review, convergence studies, provenance review, and independent free-decay evidence.

The simulator currently uses a constant-coefficient approximation evaluated at a documented frequency. It does not implement Cummins radiation-memory convolution. Capytaine radiation damping must never be substituted for the separately sourced viscous damping model.

`npm run generate:parametric-hulls`, `npm run converge:parametric-hulls`, and `npm run generate:capytaine-parametric` implement the analytic Vehicle B/C generalised-Wigley path. Generated meshes remain under `artifacts/generated/`. The tracked source specification, independent mesh-hydrostatic verification, convergence result, and no-overwrite GM comparison make the geometry reproducible without promoting it to physical validation evidence. Verification meshes are closed with a waterplane lid; the Capytaine loader removes that lid before solving because Capytaine reconstructs the waterplane from the wetted boundary.

`npm run generate:capytaine-bootstrap` executes the complete integration path for Vehicles B and C using displaced-volume-consistent parametric meshes. These meshes are intentionally tagged `bootstrap`, and their resolved artifacts remain `integration-complete-validation-blocked-bootstrap-mesh`. They prove the executable coefficient, coordinate-transform, encounter-frequency, wave-excitation, free-decay fixed-point, damping-composition, provenance, and manifest path. They are not reviewed hull hydrodynamics.

For each regular-wave episode, the resolved manifest stores the fixed encounter frequency, intrinsic frequency, reference speed, encounter angle, interpolation bracket, source artifact checksum, rejection-only extrapolation policy, and the no-radiation-memory warning. Free-decay artifacts store every fixed-point iteration with modal assurance, mode vector, added-mass matrix, and radiation-damping matrix. Heave, roll, and pitch checks use combined radiation and independently provenanced viscous/quadratic damping.
