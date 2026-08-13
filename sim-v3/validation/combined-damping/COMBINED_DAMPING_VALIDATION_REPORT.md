# Combined-damping validation report

## Outcome

The deterministic software gate passes for Vehicles B and C in heave, roll, and pitch. This validates coefficient composition, the nonzero-viscous guard, sampled dissipativity, numerical decay extraction, and the observable effect of viscous damping. It does not validate the coefficient values against a physical vessel.

| Vehicle | Mode | Evaluation frequency (rad/s) | Viscous increase in logarithmic decrement | Settling-time change (s) |
| --- | --- | ---: | ---: | ---: |
| B | Heave | 4.25355 | 0.07446 | -0.016 |
| B | Roll | 5.48741 | 0.31444 | -2.204 |
| B | Pitch | 4.65329 | 0.05995 | -0.068 |
| C | Heave | 4.05682 | 0.04291 | -0.018 |
| C | Roll | 5.34964 | 0.16684 | -0.600 |
| C | Pitch | 4.70248 | 0.06874 | -0.566 |

Negative settling-time changes mean the combined model enters and remains within the two-percent displacement band earlier than the radiation-only diagnostic.

## Model and checks

For each mode, the free-decay fixed-point solver selects a frequency using Capytaine added mass. Capytaine radiation damping at that frequency is then combined with independently configured linear and quadratic viscous matrices. Coefficients remain fixed for the 30-second RK4 episode; Cummins radiation-memory convolution is not implemented.

The gate requires:

- separate potential, linear-viscous, and quadratic-viscous provenance;
- a nonzero viscous contribution;
- finite deterministic traces and trace checksums;
- nonnegative damping power over 234 signed velocity samples per mode;
- at least two extractable positive peaks; and
- combined logarithmic decrement strictly above the radiation-only diagnostic.

Regression tests remove both viscous matrices and require configuration rejection. A deliberately non-dissipative coupled matrix must also fail.

## Evidence scope and blockers

Status is `software-gate-passed-physical-validation-blocked`. The Vehicle B and C viscous values are uncalibrated engineering bootstrap estimates with no fit dataset. The current Capytaine meshes are also bootstrap geometry. Although the Ikeda, Himeno, and Tanaka component framework supports separating potential and viscous roll damping, the current inputs do not support a complete Ikeda-method calculation and no such claim is made.

Physical promotion requires reviewed wetted-hull meshes, mesh convergence, and independent heave/roll/pitch free-decay measurements with documented coordinate, sensor, operating-range, and uncertainty metadata. The measured decay must then be routed through the same artifact contract and used for a reviewed fit or comparison.
