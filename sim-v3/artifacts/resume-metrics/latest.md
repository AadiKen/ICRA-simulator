# Resume metrics

Generated 2026-08-18T04:48:01.286Z in 6.51 seconds on 10 logical CPUs. These are local engineering benchmarks, not physical-validation results.

- Production simulation: 3,649.86 steps/s across 16 in-process environments (1.366x scalar throughput).
- Checkpoint/replay: 100/100 exact replay trials, 500 replayed steps, 0 mismatches; mean save/load 1.698/1.42 ms.
- Worker scaling and isolation: 970.16 steps/s at concurrency 4 (3.348x vs. concurrency 1); 2/2 injected failures isolated, 0 healthy jobs lost, and 0 ordering violations.
- UI build: 3 assets built in 2.18 s; 717.0 KiB raw / 186.5 KiB gzip JavaScript.
- Cross-runtime reference (existing gate): 1,000 steps with maximum Node/PyTorch error 4.97e-14; 8-environment embedded batch error 0.
- Migration reference (existing gate): 0 unreviewed deltas across 7 frozen behavior surfaces.
