# Raw figure-data export (2026-09-09)

This directory packages the raw, non-averaged evidence currently present in the
repository. It does **not** synthesize missing seeds, metrics, repetitions, or
experimental arms.

## Contents

| Requested figure | Export | Completeness |
|---|---|---|
| Fig. 1 — Architecture | `architecture-confirmation.md` | Confirmed from production source; no experimental data required. |
| Fig. 2 — Headline policy | `policy-evaluation-episodes.csv`, `policy-training-runs.csv`, and `policy-source/` | Partial. Three training simulators, one training seed (7319) each, 50 nominal evaluation seeds each. Requested OOD categories, cross-track error, completion time, and energy are absent from these campaign outputs. |
| Fig. 3a — Throughput | `throughput-heuristic-pilot.csv` and `.json` | Pilot only. One short timing sample per point, CPU/MPS only; the requested 5–10 full timing repetitions and CUDA results were explicitly deferred. |
| Fig. 3b — Gazebo agreement | `gazebo-six-maneuvers-aligned.csv`, source comparison JSON, and `gazebo-live/` | Available for six maneuvers. The aligned file contains bcod-sim and Gazebo position, heading, `u`, `v`, and `r`, plus errors. |
| Fig. 4 — Generalization/ablations | — | Not available. The four-arm training/evaluation campaign was explicitly deferred; no per-seed Full/−heterogeneity/−correlated-sensing/−grounding results exist in the repository. |
| Fig. 5 — Environmental grounding | `environmental-matched-observations.csv` and source JSON | 2,595 matched HF-radar/RTOFS rows. QC filtering and mask accounting are in the source input/report. Zero-current and persistence predictions were not stored. |
| Fig. 6a — Determinism | `determinism-55-cells.csv` and `.json` | Complete 55-cell matrix. |
| Fig. 6b — MCP | `mcp-84-trials.csv`, `.json`, and `mcp-transcripts/` | Complete 84 trials. The report stores task/category/repetition/outcome/detail/tool calls and full answers. “Safe handling,” “silent error,” and “provenance preserved” are benchmark-derived summary concepts, not independent per-trial fields, so they are not reverse-engineered here. |

## Important claim limits

- The policy export is a **single-training-seed nominal common-suite campaign**, not
  the requested multi-seed generalization experiment.
- The throughput export is labeled `heuristic-pilot-extrapolation` by its producer.
  Do not present it as a replicated throughput benchmark.
- The Gazebo export uses the Vehicle A comparison underlying the quoted aggregate
  claims: unweighted maneuver-level means of 10.4 cm position and 0.67 degrees
  heading, with maxima of 25.5 cm and 2.33 degrees. The six included maneuvers are
  constant thrust, turning circle, yaw turn, zig-zag, coast-down, and current
  drift. The extra source scenario (`impulse-hold`) has no scored error series and
  is excluded because the request specifies six maneuvers.
- Environmental matched rows are not statistically independent because multiple
  HF-radar cells may map to the same coarse RTOFS cell.

## Column notes

- `gazebo-six-maneuvers-aligned.csv`: one aligned timestamp per row; coordinates
  are north/east rather than Cartesian labels `x/y`; headings are radians.
- `policy-evaluation-episodes.csv`: `training_simulator` is the backend on which
  the corresponding policy was trained; `evaluation_condition` is `nominal`.
- `mcp-84-trials.csv`: `tool_calls_json` preserves the full ordered tool-call list
  as JSON within a CSV field.
