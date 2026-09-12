# Actuator-envelope calibration

`protocol.json` freezes the common four-arm measurement contract. Each native runner must emit
`bcod-sim.json`, `gazebo.json`, `holoocean.json`, or `stonefish.json` with artifact kind
`native-actuator-envelope-capture`. `analyze.py` refuses incomplete command grids, non-steady
speed measurements, command/delivered-force disagreement, missing turning probes, or missing
arms. It flags pairwise metric gaps above 10% for review and never changes an actuator cap.

Run the analysis only after all four native captures exist:

```sh
.venv/bin/python validation/actuator-envelope/analyze.py \
  --input-dir artifacts/actuator-envelope/native \
  --output artifacts/actuator-envelope/analysis.json
```

The HoloOcean 501.1328125 N calibration is legacy evidence, not a valid matrix row: it contains
only a full-command straight-line probe and no matching 25/50/75% or turning measurements.
