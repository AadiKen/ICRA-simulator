# Narrow behavioral-golden supersession

Rebaselining remains forbidden. The only allowed path is declared in
`policy.json` and retains the original golden permanently.

The protocol uses two commits:

1. Commit the reviewed capability implementation after the original migration
   gate passes.
2. With a clean worktree at that commit or a descendant, run `create.mjs` with the
   exact implementation commit, a candidate trace, and reviewed path-prefix
   justifications. Commit the new golden and generated delta artifact separately.

The tool verifies the old golden against its original immutable manifest, refuses
unknown policies and existing outputs, requires every changed path to have a
justification, and records both checksums plus the capability commit. It never
modifies the legacy golden or its manifest.

Example after the capability implementation commit exists:

```sh
node validation/behavior-supersession/create.mjs \
  --policy actuator-integrated-capabilities-v1 \
  --capability_commit <implementation-commit> \
  --candidate <captured-candidate.json> \
  --justifications <reviewed-justifications.json>
```
