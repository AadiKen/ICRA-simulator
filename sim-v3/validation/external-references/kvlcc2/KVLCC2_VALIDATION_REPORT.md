# KVLCC2 validation status

Vehicle B's MMG evidence is now divided into operating-point configurations that cannot silently exchange MARIN, L3, HMRI, HSVA, or full-scale values.

## MARIN L7 experiment

`marin_l7_experiment` runs the 7 m model at 1.17954 m/s and 10.3333333 rps (620 rpm), using the documented Froude-scaled rudder rate. Both 35-degree turning signs and the port/starboard 10/10 and 20/20 zig-zags execute reproducibly. Results are compared with the published MARIN experimental scalar indices and tagged `experimental_validation`.

The measured MARIN resistance curve is not available. The current configuration therefore reports an assumed straight-ahead trim resistance equal to effective propeller thrust at 620 rpm. This creates zero initial surge residual without fitting any maneuver metric. It must not be interpreted as a measured resistance model, full-trajectory validation, or Vehicle B USV-scale validation. No experimental acceptance tolerance has been retrofitted after seeing the comparison.

## Yasukawa L7 calculation

`yasukawa_l7_unresolved` is tagged `blocked_missing_primary_source`. It refuses to execute until the exact propeller rate, extrapolated L7 resistance, resistance decomposition/correlation allowances, and operating-point wake/thrust-deduction assumptions are supplied. The insertion point is documented in `UNRESOLVED_YASUKAWA_L7.md`; future primary values require configuration changes only.

Earlier guessed-operating-point artifacts remain retained as historical diagnostics. They are not validation evidence and are not silently superseded.

## Printed-coefficient diagnostic

`l3_coefficient_diagnostic` uses printed `R0_prime = 0.022` and solves effective-thrust/resistance equilibrium. With the published rounded inputs it resolves:

- shaft rate: 11.8569921 rps (711.4195 rpm);
- advance coefficient: 0.2763342;
- thrust coefficient: 0.2064492;
- normalized surge residual: approximately `1.05e-13`.

The supplied approximate reference tuple (11.8337729 rps, `JP = 0.2772336`, `KT = 0.2061327`) differs by less than 0.3%. Substituting that tuple would imply `R0_prime` near 0.021824 rather than the printed 0.022, so the code preserves the printed coefficient and reports the discrepancy. This is tagged `diagnostic`, never historical L7 reproduction.

## Sensitivity

The `sensitivity_analysis` artifact independently crosses shaft rates 9.9, 10.233, 10.3333333, 11.0, and 11.8338 rps with resistance factors 0.9, 1.0, and 1.1. It reports initial force residual, turning indices and duration, zig-zag overshoots, and speed loss. No best-performing row is selected as the historical operating point.

Vehicle B is now `model-structure-trajectory-scored-fail`: cache-only MARIN trajectories provide the closest single-screw experimental reference, and measured rudder/RPM inputs have been replayed through the MMG simulator. The current parameterization fails the precommitted trajectory and maneuver-index limits. Independent USV-scale maneuver data is still required before claiming behavioral validation of Vehicle B itself.
