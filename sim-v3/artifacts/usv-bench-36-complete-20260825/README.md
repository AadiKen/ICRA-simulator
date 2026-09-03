# USV-Bench-36 complete controller campaign

This directory contains 756 runs: 36 scenarios × 3 vehicles × one seed for PID and MPC (216 runs), plus 36 scenarios × 3 vehicles × five seeds for PPO (540 runs).

The PID/MPC phase passed an exact, field-for-field comparison against `artifacts/baselines/usv-bench-policy-report.json` before PPO execution began. Every directory below `runs/` contains a finalized T1.1 manifest with git state, seed, host and runtime metadata, lockfile/config hashes, timestamp, artifact inventory, and aggregate output checksum.

Figure inputs:

- `heatmap.csv`: scenario × vehicle × controller means and success rates.
- `ppo-variance.csv` and `ppo-variance.json`: mean, population standard deviation, minimum, maximum, and all five seed values for each scenario × vehicle cell.
- `failure-taxonomy.json`: every failed run with category, failure step, and full terminal state.
- `rows.json`: all raw campaign metrics.
- `report.json`: campaign dimensions, exact-match gate, manifest count, and failure-category totals.

Failure classification is intentionally conservative. A non-finite terminal trajectory is `instability`; reaching the mission threshold is success; a finite run that reaches the fixed 100-step episode limit without reaching the mission threshold is `timeout`. The current policy-campaign interface emits no collision or allocation-saturation event, so neither category is inferred from unrelated state values. No failures were assigned to `other`.

Re-run from the repository root with:

```sh
node --experimental-strip-types validation/policy-campaign/full-campaign.ts artifacts/usv-bench-36-complete-20260825
```
