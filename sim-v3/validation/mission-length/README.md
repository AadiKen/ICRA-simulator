# Mission-length artifact stress result

This pre-parallelization slice executes Vehicle A for exactly 300 seconds at 0.02 seconds per step through the real `MarineSimulation` production adapter.

## Result

- 15,000 state rows.
- 14 numeric state/energy columns.
- 500-row Node JSONL write chunks.
- 2,048-row streaming PyArrow batches.
- 8 Parquet row groups.
- State JSONL: 5,536,633 bytes.
- State Parquet with Zstandard compression: 1,329,821 bytes.
- Summary Parquet: one row, 5,555 bytes.
- Peak Node RSS during the recorded run: 316,178,432 bytes.
- Wall time during the recorded run: 4,859 ms.

The first attempt exposed an off-by-one duration error caused by accumulated floating-point time. Artifact-producing fixed-duration runs now use an exact integer step budget; the summary records `fixed_step_budget` as its termination mode.

## Schema decisions

- Keep high-rate state rows narrow and numeric. Sensor payloads, events, actuators, and environmental fields belong in separate tables rather than widening every state row.
- Write JSONL in bounded chunks and convert to Parquet in streaming row groups; neither Node nor Python may collect an entire sweep in memory.
- Use integer step counts as the authoritative fixed-duration contract. Store simulated time as a reported value, not as the loop termination predicate.
- Keep run summaries in a separate one-row Parquet table suitable for benchmark aggregation.
- Record latency and memory in the summary, but do not use their exact values as deterministic physics goldens.

## Checksums

| Artifact | SHA-256 |
| --- | --- |
| `state.parquet` | `c2b1abae217d16de99bd971bb2f0a2e5f4e2c34295967caaa0de7ee5267dd73a` |
| `summary.parquet` | `afcb555c415d2d63c652818857cd543ac8e8d4da9d57726e6dd2094d4191bd92` |
| `state.jsonl` | `831a33347482ced1f967129f91a40a32aba095154a402b64e0c7c5f580721455` |
| `summary-row.jsonl` | `650094593d9fa357fb4ed0d7661bc5e03be79fe5160cde385f58bd2260acfb0e` |
