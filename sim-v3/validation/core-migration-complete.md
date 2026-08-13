# Core migration completion

All seven gated physics components now execute from `packages/core`: mass/inertia,
Coriolis, damping, hydrostatics, actuators, wind/current, and waves. Production
imports are checked by `validation/production-core-wiring.test.mjs`.

The files under `core/forces` that re-export migrated classes are not production
implementations. They remain narrowly as compatibility facades so the immutable
pre-migration generators and historical validation tests can still run against
their original import paths. Removing those paths would invalidate the permanent
comparison harness without changing production architecture.

`08-post-migration.json` is the final comparison against the original, never
rebaselined goldens. A green migration report proves behavioral preservation only;
it does not substitute for Capytaine, KVLCC2/MMG, AERO4River, or Gazebo evidence.
