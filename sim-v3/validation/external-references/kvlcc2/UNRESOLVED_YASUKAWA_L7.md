# Unresolved Yasukawa L7 operating point

Exact reproduction of the calculated L7 maneuvers remains blocked on primary-source inputs that are not printed in the paper:

- final extrapolated L7 resistance or `R0_prime`;
- exact fixed propeller rate at the 15.5-knot approach condition;
- resistance form factor and friction/residual decomposition;
- scale/model correlation and appendage allowances;
- operating-point wake and thrust-deduction assumptions;
- ideally, the original maneuver input deck or initialization output.

The blocked insertion point is `cases/yasukawa_l7_unresolved.json`. Replace the `null` fields with fully provenanced parameter objects and select one of the resistance models implemented by `resistance.mjs`. No MMG simulator-code change is required. Until then, the case exits with `blocked_missing_primary_source` and cannot run maneuvers.

The MARIN 620 rpm campaign and the printed-coefficient diagnostic are separate configurations. Neither supplies or estimates the missing Yasukawa inputs.
