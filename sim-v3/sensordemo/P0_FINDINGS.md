# Sensor demo P0 verification

- V1: The sensor SDK models noisy/dropout sensors and provides ground-truth access, but no radar or AIS contact model existed. `sensors.py` is therefore new code.
- V2: The available sensor runtime exposes a `groundTruth` service (full scene), not a documented radius/k-nearest object query. The sensor model filters the supplied iterable by range.
- V3: A runtime `raycast(origin, direction, maxRangeM)` service exists. The backend adapter exposes line of sight; an explicit static-ENC polygon fallback is also included.
- V4: The Python task environment directly commands port/starboard effectors. No programmatic interface explicitly identified as Vehicle A's field-trial-validated autopilot was found. The ready-to-run local adapter therefore uses the separately frozen `LOS-PID-v2` Vehicle A controller and labels its scope as bcod-sim reference validation, not field-trial evidence. Its adapter seam can be replaced without changing the policy environment.
- V5: The sensor runtime exposes `environment(position)` and bathymetry services. `environment_at(north,east)` is captured in the backend boundary so sampling and frame conversion remain owned by the simulator.
- V6: Existing task contracts use canonical JSON plus SHA-256. The new, separate `SENSOR_DEMO_OBS_CONTRACT_HASH` applies the same convention and leaves the portable contract untouched.
- V7: Gymnasium and Stable-Baselines3 vector environments support the 320-float observation and two-float action. `SensorDemoWaypointEnv` declares both spaces.

Sensor coefficients in `config.py` are explicitly plausible-but-unvalidated demo parameters. No physical validation claim is made.
