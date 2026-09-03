# VRX stock wind-frame bug

## Finding

The stock VRX `USVWind` system applies its independent longitudinal and lateral quadratic-drag coefficients in world X/Y coordinates. It does not rotate a craft's frontal and side projected areas with hull heading. For an anisotropic craft such as Surveyor, turning the hull therefore fails to turn the wind-load model with it.

This is a simulator-plugin frame bug, not a Surveyor configuration choice. Projected-area coefficients describe body-fixed geometry: the frontal coefficient must follow the bow axis and the side coefficient must follow the beam axis. A correct implementation transforms wind-relative velocity into the body frame, evaluates the directional drag there, and rotates the resulting force back to the world frame. The stock behavior fails this frame-covariance check for oblique headings.

## Diagnostic-only correction

The conformance fork uses `vrx_surveyor::SurveyorRelativeWind`, which performs that body/world transformation while retaining the same documented Surveyor geometry-derived areas, air density, and drag coefficient. Prescribed-state tests match the Node reference force transformation to numerical precision; the stock world-axis calculation is retained as a negative control.

The correction belongs only to configuration `vrx:surveyor-patched` (`leadcat/vrx:surveyor-patched-v3.0.1`). It is not applied to `vrx:stock`, is prohibited for policy training, and cannot contribute to the paper's “trained in VRX versus trained in bcod-sim” baseline figures or tables. The baseline intentionally preserves the published VRX behavior, bug included.

## Reproducibility boundary

- Stock baseline: pinned unmodified VRX source/build, tagged `leadcat/vrx:stock-v3.0.1`; no custom plugin or model mount.
- Patched conformance runtime: stock base plus the Surveyor current-relative-velocity and body-frame wind plugins, tagged `leadcat/vrx:surveyor-patched-v3.0.1`.
- Configuration rules: `validation/rl-campaign/ports/vrx-configurations.json`.
- Diagnostic implementation: `validation/rl-campaign/ports/vrx-current-system/SurveyorRelativeWind.cc`.
- Audit evidence: `artifacts/rl-campaign/vrx-gate7-full/yaw-and-frame-audit-result.json`.

This separation permits the wind defect to be reported honestly as an independent VRX finding without silently improving the policy-training baseline.
