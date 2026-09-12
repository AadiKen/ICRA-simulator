# Figure workspace

This directory contains the five current publication deliverables.

- `production/fig3_environmental_sensing.py` renders canonical `fig3_sensing` outputs.
- `production/heterogeneous_vehicle_validation.py` renders the vehicle-validation
  figure.
- `production/table6_comparison.py` renders the capability table.
- `production/environment_validation.py` renders environmental validation.
- `production/comparative_training.py` renders comparative training.
- `production/extract_fig3_sensing.ts` extracts the sensing source artifact from
  the production sensor plugins.
- The matching `test_*.py` files verify both pipelines.
- `out/publication/` contains current rendered figures and provenance sidecars.

From `sim-v3`, render sensing with:

```sh
npm run render:figure-sensing
npm run render:figure-capability
npm run render:figure-environment
npm run render:figure-training
```

Vehicle validation requires an explicit Rotation In Place callout decision:

```sh
npm run render:figure-vehicle -- --rotation-callout yes
npm run render:figure-vehicle -- --rotation-callout no
```
