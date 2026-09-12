# VRX Surface static restoring sweep

Pinned production algorithm: `osrf/vrx@fda3596…/vrx_gz/src/Surface.cc`. This is a fixed-pose, zero-velocity, zero-wave and zero-thrust source-exact force probe; it does not integrate vessel dynamics. Positive roll/pitch angles should produce negative restoring moments.

Equilibrium link z: **-0.071692 m**; equilibrium Surface immersion: **0.071692 m**.

## Roll sweep

| Roll (deg) | Moment about roll (N·m) | Net heave force (N) | Point immersions m (state) |
|---:|---:|---:|---|
| 0 | 0 | 0 | 0.072 (partial); 0.072 (partial); 0.072 (partial); 0.072 (partial) |
| 5 | -95.457 | 21.945 | 0.100 (partial); 0.100 (partial); 0.043 (partial); 0.043 (partial) |
| 10 | -180.996 | 92.621 | 0.129 (partial); 0.129 (partial); 0.014 (partial); 0.014 (partial) |
| 20 | -259.037 | 322.274 | 0.170 (saturated); 0.170 (saturated); 0 (dry); 0 (dry) |
| 30 | -238.730 | 322.274 | 0.170 (saturated); 0.170 (saturated); 0 (dry); 0 (dry) |
| 45 | -194.922 | 322.274 | 0.170 (saturated); 0.170 (saturated); 0 (dry); 0 (dry) |
| 60 | -137.831 | 322.274 | 0.170 (saturated); 0.170 (saturated); 0 (dry); 0 (dry) |

## Pitch sweep

| Pitch (deg) | Moment about pitch (N·m) | Net heave force (N) | Point immersions m (state) |
|---:|---:|---:|---|
| 0 | 0 | 0 | 0.072 (partial); 0.072 (partial); 0.072 (partial); 0.072 (partial) |
| 5 | -181.486 | 42.897 | 0.112 (partial); 0.032 (partial); 0.112 (partial); 0.032 (partial) |
| 10 | -323.296 | 204.497 | 0.151 (partial); 0 (dry); 0.151 (partial); 0 (dry) |
| 20 | -359.119 | 322.274 | 0.170 (saturated); 0 (dry); 0.170 (saturated); 0 (dry) |
| 30 | -330.966 | 322.274 | 0.170 (saturated); 0 (dry); 0.170 (saturated); 0 (dry) |
| 45 | -270.233 | 322.274 | 0.170 (saturated); 0 (dry); 0.170 (saturated); 0 (dry) |
| 60 | -191.083 | 322.274 | 0.170 (saturated); 0 (dry); 0.170 (saturated); 0 (dry) |

## Heave sweep

| Heave offset (m; + upward) | Net heave force (N) | Point immersions m (state) |
|---:|---:|---|
| -0.050 | 561.381 | 0.122 (partial); 0.122 (partial); 0.122 (partial); 0.122 (partial) |
| -0.020 | 213.610 | 0.092 (partial); 0.092 (partial); 0.092 (partial); 0.092 (partial) |
| 0.000 | 0 | 0.072 (partial); 0.072 (partial); 0.072 (partial); 0.072 (partial) |
| 0.020 | -192.574 | 0.052 (partial); 0.052 (partial); 0.052 (partial); 0.052 (partial) |
| 0.050 | -423.421 | 0.022 (partial); 0.022 (partial); 0.022 (partial); 0.022 (partial) |

## Finding

The force sign is restoring at every tested nonzero roll and pitch angle. The curve does not reverse through 60°. It ceases to behave approximately linearly when points first dry or saturate: between 10° and 20° roll (observed at 20°), and between 5° and 10° pitch (observed at 10°). Roll moment peaks at 20° and then declines; pitch moment peaks at 20° and then declines. Once opposing points are dry/saturated, total buoyancy is 835.337 N, producing an upward net force of 322.274 N. This strong angle-to-heave coupling is the notable discontinuous regime; the static restoring sign itself is not wrong.
