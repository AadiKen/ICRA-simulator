# External validation datasets

This directory contains acquisition metadata and integrity tooling, not experimental
evidence. A dataset becomes admissible only after its files have been obtained from
the primary source, inventoried in `checksums.sha256`, and verified locally.

## Current gates

| Dataset | Primary source | License/access status | Evidence status |
| --- | --- | --- | --- |
| AERO4River v1 | DOI `10.17632/tf4ym6fsvc.1` (Mendeley Data) | CC BY 4.0; six public files acquired and SHA-256 locked | Imported with versioned coordinate transform; Vehicle C campaign is descriptive, not same-vessel validation |
| Yasukawa & Yoshimura 2015 KVLCC2 | DOI `10.1007/s00773-014-0293-y` | Published paper selected | Vehicle B Stage 1 reference; coefficient/result extraction and convention review pending |
| SIMMAN 2014 KVLCC2/KCS | SIMMAN 2014 workshop | Official site directs users to request dataset access | Optional cross-check; no longer on the Vehicle B Stage-1 critical path |
| wPCC v4 | DOI `10.17632/j5zdrhr9bf.4` (Mendeley Data) | CC BY 4.0; supplied archive acquired and SHA-256 locked | Raw measured twin-screw zig-zag trajectories; architecture-variant evidence only |
| KVLCC2 MARIN/HSVA | SIMMAN 2008 via Alexandersson reproduction repository | No source-repository license; cache-only, never redistributed | Six MARIN raw trajectories imported; HSVA available pending required preprocessing |

Vehicle B Stage 1 uses the published KVLCC2 MMG derivatives and maneuver results
from Yasukawa & Yoshimura (2015). SIMMAN remains useful but optional. The SIMMAN
2020 organizers warn that much of the 2008/2014 model-test data was
replaced. Do not mix releases. The selected workshop release, hull, water depth,
model scale, propulsion condition, and test identifier must be recorded per trace.

## Workflow

1. Obtain the exact release from the primary source under its stated access terms.
2. Place files under `validation/datasets/raw/<dataset-id>/`. `raw/` is ignored.
3. Inspect the source documentation before defining any coordinate transform.
4. Create a SHA-256 lock file:

   ```bash
   node validation/datasets/tool.mjs lock aero4river-v1
   ```

5. Review and commit `validation/datasets/locks/<dataset-id>.sha256`.
6. Verify before every import or comparison:

   ```bash
   node validation/datasets/tool.mjs verify aero4river-v1
   ```

`lock` refuses a dataset whose manifest does not permit local acquisition. A lock
records relative paths and content hashes; it does not establish scientific
meaning. Importers must additionally enforce the conventions and unresolved gates
in the corresponding manifest.

## Coordinate policy

- Never infer handedness, heading sign, origin, body axes, or dimensional scaling
  from variable names alone.
- Preserve original variables and units in the immutable raw layer.
- Put every axis/unit/time transform in a versioned import artifact.
- AERO4River's primary dataset page documents `x`, `y` [m], yaw [rad], `u`, `v`
  [m/s], `r` [rad/s], `fx`, `fy` [N], `Tn` [N m], and `t` [s]. The associated
  publication's Section II-B equations and raw-track kinematic consistency resolve
  the transform in `validation/aero4river/aero4river-coordinate-transform-v1.json`.
- SIMMAN conventions are test-package-specific. No trajectory may be imported
  until its official specification and column definitions are present locally.
- wPCC v4 motions use `[Lpp/2, 0, WL]` with positive forward, starboard, and
  into-water axes. Only position, orientation, and accelerometers are gating;
  model-conditioned EKF velocity channels are diagnostic-only.

## Sources

- AERO4River dataset: https://doi.org/10.17632/tf4ym6fsvc.1
- Associated AERO4River article: https://doi.org/10.1109/ACCESS.2021.3067448
- KVLCC2 MMG Stage-1 reference: https://doi.org/10.1007/s00773-014-0293-y
- SIMMAN 2014: https://simman2014.dk/
- SIMMAN 2020 release warning: https://simman2020.kr/contents/Purpose.php?me_code=10
- wPCC v4 dataset: https://doi.org/10.17632/j5zdrhr9bf.4
- SIMMAN 2008 / Alexandersson handoff provenance: `validation/external-references/kvlcc2-marin/SOURCE_PROVENANCE.md`
