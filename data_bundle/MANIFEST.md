# Data bundle manifest

Aggregated 2026-09-11 from repository HEAD `2b27f08`. Files are physical copies; no symlinks are used. Raw upstream data dumps, including RTOFS NetCDF cache content, are intentionally excluded.

## Critical provenance notes

1. **Throughput retraction.** Every file under [`07_throughput/e2-throughput/RETRACTED-see-manifest/`](07_throughput/e2-throughput/RETRACTED-see-manifest/) is retracted. The claimed A30 provenance could not be trusted, and the root cause was never found. The local M1 pilot, claimed-A30 files, and scheduler log are retained together so the provenance record is not scrubbed.
2. **Missing 69-trial MCP report.** The earlier 69-trial MCP evaluation report is confirmed unrecoverable after checks of the workspace, broader project tree, and Git history. It is absent because no recoverable artifact exists, not because aggregation overlooked it.
3. **Untracked sources.** Any row marked `untracked` was untracked in the source repository at aggregation time. Copying it into this bundle does not substitute for committing the source file. The newly generated `data_bundle/` is itself also untracked until explicitly added to Git.
4. **Blocked original wind stations.** The Honolulu station 51202 and Miami station 42095 results from commit `0f86dc6` are retained under `03_environmental_wind/BLOCKED-original-stations/`. Both had `insufficient_reference_data` and were later superseded by OOUH1 and VAKF1 in commit `201342f`.
5. **Baseline coverage.** The current baseline artifact contains San Francisco current baselines and San Francisco/Boston wind baselines. No Honolulu/Miami zero-wind or persistence baseline outputs exist in the repository.
6. **Cluster sweep.** No output from a newer real cluster throughput sweep was present at aggregation time. The existing batch-64 cluster-status artifact is included and remains blocked.
7. **Paper sources.** No canonical `.tex`, `references.bib`, or `ieeeconf.cls` files exist in the current repository tree or any reachable Git history. `10_paper/` is therefore intentionally empty.

## Artifact inventory

| Bundle path | Source provenance | Git status | Status |
|---|---|---:|---|
| `01_vehicle_fidelity/fig1_vessels.py` | `sim-v3/validation/figures/fig1_vessels.py` | af41b61 | source — current working-tree state |
| `01_vehicle_fidelity/open-loop-trajectory-comparison-thrust-corrected.json` | `sim-v3/artifacts/gazebo/vehicle-c/open-loop-trajectory-comparison-thrust-corrected.json` | af41b61 | real, verified |
| `01_vehicle_fidelity/open-loop-trajectory-comparison.json` | `sim-v3/artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison.json` | 554ca0e | real, verified |
| `01_vehicle_fidelity/open-loop-trajectory-comparison.png` | `sim-v3/artifacts/gazebo/vehicle-a/open-loop-trajectory-comparison.png` | 554ca0e | real, verified |
| `01_vehicle_fidelity/stability-diagnostic.json` | `sim-v3/artifacts/gazebo/vehicle-c/timestep-diagnostic/stability-diagnostic.json` | af41b61 | real, verified |
| `02_environmental_currents/CURRENT_VALIDATION_REPORT.md` | `sim-v3/validation/environmental-validation/CURRENT_VALIDATION_REPORT.md` | af41b61 | real, verified |
| `02_environmental_currents/enc-catzoc-remaining-sites.json` | `sim-v3/artifacts/environment-coverage/enc-catzoc-remaining-sites.json` | af41b61 | real, verified |
| `02_environmental_currents/enc-live-confirmatory-pass.json` | `sim-v3/artifacts/environment-coverage/enc-live-confirmatory-pass.json` | 2fef2eb | real, verified |
| `02_environmental_currents/environmental-baselines-20260713-15.json` | `sim-v3/artifacts/environmental-validation/environmental-baselines-20260713-15.json` | af41b61 | real, verified — current baselines are San Francisco only |
| `02_environmental_currents/report-3dz-20260713-15.json` | `sim-v3/artifacts/environmental-validation/report-3dz-20260713-15.json` | af41b61 | real, verified |
| `02_environmental_currents/rtofs-mask-boston.json` | `sim-v3/artifacts/environmental-validation/rtofs-mask-boston.json` | af41b61 | real, verified |
| `02_environmental_currents/rtofs-mask-honolulu.json` | `sim-v3/artifacts/environmental-validation/rtofs-mask-honolulu.json` | af41b61 | real, verified |
| `02_environmental_currents/rtofs-mask-miami.json` | `sim-v3/artifacts/environmental-validation/rtofs-mask-miami.json` | af41b61 | real, verified |
| `02_environmental_currents/rtofs-mask-sf.json` | `sim-v3/artifacts/environmental-validation/rtofs-mask-sf.json` | af41b61 | real, verified |
| `03_environmental_wind/BLOCKED-original-stations/ERA5_WIND_VALIDATION_REPORT_HONOLULU.md` | `0f86dc6:sim-v3/validation/environmental-validation/ERA5_WIND_VALIDATION_REPORT_HONOLULU.md` | 0f86dc6 | blocked — original station had insufficient reference data; superseded by current replacement |
| `03_environmental_wind/BLOCKED-original-stations/ERA5_WIND_VALIDATION_REPORT_MIAMI.md` | `0f86dc6:sim-v3/validation/environmental-validation/ERA5_WIND_VALIDATION_REPORT_MIAMI.md` | 0f86dc6 | blocked — original station had insufficient reference data; superseded by current replacement |
| `03_environmental_wind/BLOCKED-original-stations/era5-ndbc-wind-honolulu-20260713-15.json` | `0f86dc6:sim-v3/artifacts/environmental-validation/era5-ndbc-wind-honolulu-20260713-15.json` | 0f86dc6 | blocked — original station had insufficient reference data; superseded by current replacement |
| `03_environmental_wind/BLOCKED-original-stations/era5-ndbc-wind-miami-20260713-15.json` | `0f86dc6:sim-v3/artifacts/environmental-validation/era5-ndbc-wind-miami-20260713-15.json` | 0f86dc6 | blocked — original station had insufficient reference data; superseded by current replacement |
| `03_environmental_wind/ERA5_WIND_VALIDATION_REPORT.md` | `sim-v3/validation/environmental-validation/ERA5_WIND_VALIDATION_REPORT.md` | af41b61 | real, verified |
| `03_environmental_wind/ERA5_WIND_VALIDATION_REPORT_BOSTON.md` | `sim-v3/validation/environmental-validation/ERA5_WIND_VALIDATION_REPORT_BOSTON.md` | af41b61 | real, verified |
| `03_environmental_wind/ERA5_WIND_VALIDATION_REPORT_HONOLULU.md` | `sim-v3/validation/environmental-validation/ERA5_WIND_VALIDATION_REPORT_HONOLULU.md` | 201342f | real, verified replacement station OOUH1 |
| `03_environmental_wind/ERA5_WIND_VALIDATION_REPORT_MIAMI.md` | `sim-v3/validation/environmental-validation/ERA5_WIND_VALIDATION_REPORT_MIAMI.md` | 201342f | real, verified replacement station VAKF1 |
| `03_environmental_wind/TIER_0_1_CLEANUP_REPORT.md` | `sim-v3/validation/environmental-validation/TIER_0_1_CLEANUP_REPORT.md` | af41b61 | real, verified |
| `03_environmental_wind/environmental-baselines-20260713-15.json` | `sim-v3/artifacts/environmental-validation/environmental-baselines-20260713-15.json` | af41b61 | real, verified where present — SF/Boston only; Honolulu/Miami wind baselines absent |
| `03_environmental_wind/era5-ndbc-wind-20260713-15.json` | `sim-v3/artifacts/environmental-validation/era5-ndbc-wind-20260713-15.json` | 4ebd8ce | real, verified |
| `03_environmental_wind/era5-ndbc-wind-boston-20260713-15.json` | `sim-v3/artifacts/environmental-validation/era5-ndbc-wind-boston-20260713-15.json` | 0f86dc6 | real, verified |
| `03_environmental_wind/era5-ndbc-wind-honolulu-20260713-15.json` | `sim-v3/artifacts/environmental-validation/era5-ndbc-wind-honolulu-20260713-15.json` | 201342f | real, verified replacement station OOUH1 |
| `03_environmental_wind/era5-ndbc-wind-miami-20260713-15.json` | `sim-v3/artifacts/environmental-validation/era5-ndbc-wind-miami-20260713-15.json` | 201342f | real, verified replacement station VAKF1 |
| `03_environmental_wind/historical-archive-fidelity.json` | `sim-v3/artifacts/environment-coverage/historical-archive-fidelity.json` | 4ebd8ce | real, verified |
| `04_determinism/fig4_determinism.svg` | `sim-v3/artifacts/figures/fig4_determinism.svg` | 91097b3 (working tree modified) | real, verified |
| `04_determinism/fig4_determinism_matrix.png` | `sim-v3/figures/out/fig4_determinism_matrix.png` | 12c5c7b | real, verified |
| `04_determinism/report.json` | `sim-v3/artifacts/determinism-sweep/report.json` | 91097b3 | real, verified |
| `05_mcp_agent_evaluation/mcp-84-trials.json` | `sim-v3/artifacts/raw-data-export-20260909/mcp-84-trials.json` | untracked | real, verified; source untracked at aggregation time |
| `05_mcp_agent_evaluation/report.json` | `sim-v3/artifacts/agent-evaluation/report.json` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/bcod-sim/evaluation-episodes.json` | `sim-v3/artifacts/common-suite/native-three-policy-20260909/bcod-sim/evaluation-episodes.json` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/gazebo/evaluation-episodes.json` | `sim-v3/artifacts/common-suite/native-three-policy-20260909/gazebo/evaluation-episodes.json` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/holoocean/evaluation-episodes.json` | `sim-v3/artifacts/common-suite/native-three-policy-20260909/holoocean/evaluation-episodes.json` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/preliminary-ppo-learning-curves.png` | `sim-v3/artifacts/figures/preliminary-ppo-learning-curves.png` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/report.md` | `sim-v3/artifacts/common-suite/native-three-policy-20260909/report.md` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/results.csv` | `sim-v3/artifacts/rl-campaign/ppo-learning-curves/results.csv` | untracked | real, verified; source untracked at aggregation time |
| `06_rl_campaign_native_eval/stonefish/evaluation-episodes.json` | `sim-v3/artifacts/common-suite/native-three-policy-20260909/stonefish/evaluation-episodes.json` | untracked | real, verified; source untracked at aggregation time |
| `07_throughput/batch64-gpu-profile-cluster-status.json` | `sim-v3/artifacts/rl-campaign/batch64-gpu-profile-cluster-status.json` | 2fef2eb | blocked — no cluster endpoint or GPU allocation |
| `07_throughput/e2-throughput/RETRACTED-see-manifest/a30-heuristic-pilot.csv` | `sim-v3/artifacts/e2-throughput/a30-heuristic-pilot.csv` | untracked | retracted — provenance/claimed hardware not trustworthy; root cause never found; source untracked at aggregation time |
| `07_throughput/e2-throughput/RETRACTED-see-manifest/a30-heuristic-pilot.json` | `sim-v3/artifacts/e2-throughput/a30-heuristic-pilot.json` | untracked | retracted — provenance/claimed hardware not trustworthy; root cause never found; source untracked at aggregation time |
| `07_throughput/e2-throughput/RETRACTED-see-manifest/heuristic-pilot.csv` | `sim-v3/artifacts/e2-throughput/heuristic-pilot.csv` | untracked | retracted — provenance/claimed hardware not trustworthy; root cause never found; source untracked at aggregation time |
| `07_throughput/e2-throughput/RETRACTED-see-manifest/heuristic-pilot.json` | `sim-v3/artifacts/e2-throughput/heuristic-pilot.json` | untracked | retracted — provenance/claimed hardware not trustworthy; root cause never found; source untracked at aggregation time |
| `07_throughput/e2-throughput/RETRACTED-see-manifest/slurm-19927.log` | `sim-v3/artifacts/e2-throughput/slurm-19927.log` | untracked | retracted — provenance/claimed hardware not trustworthy; root cause never found; source untracked at aggregation time |
| `07_throughput/gate_d_throughput.json` | `sim-v3/stonefish/gate_d_throughput.json` | 8dd1c33 | real, verified |
| `07_throughput/holoocean-vehicle-a-throughput.json` | `sim-v3/artifacts/rl-campaign/holoocean-vehicle-a-throughput.json` | c4bac81 | real, verified |
| `07_throughput/vrx-gate-d-throughput.json` | `sim-v3/artifacts/rl-campaign/vrx-gate-d-throughput.json` | 727452b | real, verified |
| `08_task_hardening_common_suite/REPORT.md` | `sim-v3/artifacts/common-suite/task-hardening/REPORT.md` | 9d9532c (working tree modified) | real, verified |
| `08_task_hardening_common_suite/common-suite-README.md` | `sim-v3/eval/common_suite/README.md` | 9d9532c | real, verified |
| `08_task_hardening_common_suite/gazebo-selected-nominal-evaluation-summary.json` | `sim-v3/artifacts/common-suite/task-hardening/gazebo-selected-nominal/evaluation-summary.json` | untracked | real, verified; source untracked at aggregation time |
| `08_task_hardening_common_suite/task-hardening-verification.json` | `sim-v3/artifacts/common-suite/task-hardening/task-hardening-verification.json` | 9d9532c (working tree modified) | real, verified |
| `09_figures/artifacts-figures/fig0_architecture.provenance.json` | `sim-v3/artifacts/figures/fig0_architecture.provenance.json` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig0_architecture.svg` | `sim-v3/artifacts/figures/fig0_architecture.svg` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig1_vessels.png` | `sim-v3/artifacts/figures/fig1_vessels.png` | af41b61 | real, verified |
| `09_figures/artifacts-figures/fig2_environmental_grounding.png` | `sim-v3/artifacts/figures/fig2_environmental_grounding.png` | untracked | real, verified; source untracked at aggregation time |
| `09_figures/artifacts-figures/fig2_geography.png` | `sim-v3/artifacts/figures/fig2_geography.png` | af41b61 | real, verified |
| `09_figures/artifacts-figures/fig2_geography.provenance.json` | `sim-v3/artifacts/figures/fig2_geography.provenance.json` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig2_geography.svg` | `sim-v3/artifacts/figures/fig2_geography.svg` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig3_environmental_sensing.png` | `sim-v3/artifacts/figures/fig3_environmental_sensing.png` | untracked | real, verified; source untracked at aggregation time |
| `09_figures/artifacts-figures/fig3_sensing.provenance.json` | `sim-v3/artifacts/figures/fig3_sensing.provenance.json` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig3_sensing.svg` | `sim-v3/artifacts/figures/fig3_sensing.svg` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig4_determinism.provenance.json` | `sim-v3/artifacts/figures/fig4_determinism.provenance.json` | 91097b3 (working tree modified) | real, verified |
| `09_figures/artifacts-figures/fig4_determinism.svg` | `sim-v3/artifacts/figures/fig4_determinism.svg` | 91097b3 (working tree modified) | real, verified |
| `09_figures/artifacts-figures/fig5_validation.provenance.json` | `sim-v3/artifacts/figures/fig5_validation.provenance.json` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/fig5_validation.svg` | `sim-v3/artifacts/figures/fig5_validation.svg` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/heterogeneous-vehicle-validation.png` | `sim-v3/artifacts/figures/heterogeneous-vehicle-validation.png` | untracked | real, verified; source untracked at aggregation time |
| `09_figures/artifacts-figures/preliminary-ppo-learning-curves.png` | `sim-v3/artifacts/figures/preliminary-ppo-learning-curves.png` | untracked | real, verified; source untracked at aggregation time |
| `09_figures/artifacts-figures/table6_comparison.provenance.json` | `sim-v3/artifacts/figures/table6_comparison.provenance.json` | 91097b3 | real, verified |
| `09_figures/artifacts-figures/table6_comparison.svg` | `sim-v3/artifacts/figures/table6_comparison.svg` | 91097b3 | real, verified |
| `09_figures/figures-out/fig2_geography.png` | `sim-v3/figures/out/fig2_geography.png` | 554ca0e | real, verified |
| `09_figures/figures-out/fig2_geography_table.md` | `sim-v3/figures/out/fig2_geography_table.md` | 12c5c7b | real, verified |
| `09_figures/figures-out/fig2_geography_table.png` | `sim-v3/figures/out/fig2_geography_table.png` | 12c5c7b | real, verified |
| `09_figures/figures-out/fig4_determinism_matrix.png` | `sim-v3/figures/out/fig4_determinism_matrix.png` | 12c5c7b | real, verified |
| `09_figures/figures-out/table6_comparison.md` | `sim-v3/figures/out/table6_comparison.md` | 12c5c7b | real, verified |
| `09_figures/figures-out/table6_comparison.png` | `sim-v3/figures/out/table6_comparison.png` | 12c5c7b | real, verified |
