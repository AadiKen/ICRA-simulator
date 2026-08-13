# Capytaine integration status

The Capytaine 2.3.1 integration is executable end to end for Vehicles B and C. It generates body-NED added-mass and radiation-damping matrices, complex wave-excitation wrenches, hydrostatic stiffness, mesh/config checksums, and side-by-side mesh/bootstrap GM values. The runtime resolver applies the resulting fixed matrices to the actual coupled6 added-mass, damping, hydrostatic, and regular-wave forcing interfaces.

## Time-domain approximation

The simulator uses one constant coefficient set per episode. Regular-wave episodes evaluate and interpolate `A(omega)`, `B_rad(omega)`, and excitation at the fixed encounter frequency computed from period, heading, direction, reference speed, and deep- or finite-water dispersion. Free-decay tests use the documented relaxed fixed-point solve with MAC mode tracking. Cummins radiation-memory convolution and time-varying encounter frequency are not implemented.

Every regular-wave manifest records the evaluation frequency, intrinsic frequency, reference speed, encounter angle, wave number, interpolation bracket, artifact checksum, and rejection-only extrapolation policy. Every free-decay iteration records its input/modal/relaxed frequencies, modal assurance, mode vector, added mass, and radiation damping.

## Damping decomposition

Capytaine supplies potential radiation damping only. Runtime damping retains three separate terms:

- Capytaine potential radiation damping;
- independently empirical linear viscous damping;
- independently empirical quadratic viscous damping.

The coupled6 resolver rejects an absent or identically zero viscous contribution. Heave, roll, and pitch free-decay artifacts simulate the combined model and report period, logarithmic decrement, damping ratio, settling time, and equilibrium error.

The dedicated combined-damping gate additionally compares the production model against a radiation-only diagnostic, samples dissipated power, and checks deterministic trace checksums. The current empirical coefficients are engineering bootstraps, not fitted free-decay coefficients and not a complete Ikeda-method calculation. The Ikeda component framework motivates the required decomposition, but physical validation remains blocked on reviewed meshes and independent Vehicle B/C free-decay data.

## Current artifacts and claim boundary

The committed Vehicle B and Vehicle C artifacts use displaced-volume-consistent parametric open-waterline boxes. Their status is `integration-complete-validation-blocked-bootstrap-mesh`, and `is_validation_evidence` remains false. The meshes demonstrate integration and expose large, informative differences between mesh-derived and geometry-bootstrap GM values; they do not represent reviewed monohull or dual-hull geometry.

Promotion requires reviewed wetted-hull meshes, panel-convergence runs, high-frequency/irregular-frequency review, and independent free-decay comparison. Replacing a mesh requires only an input configuration change and artifact regeneration; runtime code is unchanged.

## Analytic Vehicle B/C hulls

The bootstrap boxes have now been supplemented by reproducible generalised-Wigley meshes from `validation/hulls/vehicle-hull-specification.md`. Four-density convergence uses approximately 800, 1600, 3200, and 6400 panels per hull. Vehicle C includes both demihulls explicitly in every BEM solve; no transverse symmetry reduction is used.

Closed meshes are used for watertightness and surface-integral hydrostatic checks. The Capytaine solver receives the same mesh with its `z=0` verification lid removed because Capytaine reconstructs the waterplane from its boundary; retaining that lid double-counts the waterplane. Capytaine GM values are reported beside, and never overwrite, the analytic values.

These hulls replace undocumented box geometry but remain representative design assumptions rather than physical-vessel scans. Solver artifacts remain non-validation evidence and neither vehicle's validation status is upgraded.
