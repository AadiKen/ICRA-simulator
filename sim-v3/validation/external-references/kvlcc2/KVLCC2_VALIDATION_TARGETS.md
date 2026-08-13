# KVLCC2 validation targets

Use this file as the implementation-facing checklist for Vehicle B Stage 1. The authoritative source remains Yasukawa and Yoshimura (2015), acquired into the gitignored cache and verified by `SHA256SUMS.txt`; the PDF is intentionally not redistributed.

## 1. Match conventions before comparing numbers

The paper uses a midship-fixed body frame, with x forward, y starboard, and z downward. Forces and moments are non-dimensionalized by:

- force: `(1/2) rho Lpp d U^2`
- yaw moment: `(1/2) rho Lpp^2 d U^2`
- mass: `(1/2) rho Lpp^2 d`
- yaw inertia: `(1/2) rho Lpp^4 d`

Do not compare coefficients until the simulator uses the same coordinate signs, reference point, dimensionalization, and rudder-angle convention.

## 2. Geometry and test conditions

### L7 free-running model

- scale: 1/45.7
- Lpp: 7.00 m
- breadth B: 1.27 m
- draft d: 0.46 m
- displacement volume: 3.27 m^3
- longitudinal CG xG: 0.25 m in the paper's frame
- block coefficient Cb: 0.810
- propeller diameter DP: 0.216 m
- rudder span HR: 0.345 m
- movable rudder profile area AR: 0.0539 m^2

### Full scale

- Lpp: 320.0 m
- breadth B: 58.0 m
- draft d: 20.8 m
- displacement volume: 312,600 m^3
- longitudinal CG xG: 11.2 m
- block coefficient Cb: 0.810
- propeller diameter DP: 9.86 m
- rudder span HR: 15.80 m
- movable rudder profile area AR: 112.5 m^2

### Maneuver setup used for published comparisons

- initial full-scale-equivalent approach speed: 15.5 kn
- L3 captive-test model speed: 0.76 m/s
- full-scale rudder rate: 1.76 deg/s
- yaw radius of gyration: 0.25 Lpp
- propeller revolution held at the value corresponding to the initial speed
- maneuvers: +35 deg and -35 deg turning circles; +/-10/10 and +/-20/20 zig-zags

## 3. MMG coefficient baseline from Table 3

These are the coefficients actually used in the paper's simulation, after separating the combined captive-test quantities in Table 2.

| Coefficient | Value | Coefficient | Value |
|---|---:|---|---:|
| X_vv' | -0.040 | m_x' | 0.022 |
| X_vr' | 0.002 | m_y' | 0.223 |
| X_rr' | 0.011 | J_z' | 0.011 |
| X_vvvv' | 0.771 | t_P | 0.220 |
| Y_v' | -0.315 | t_R | 0.387 |
| Y_R' | 0.083 | a_H | 0.312 |
| Y_vvv' | -1.607 | x_H' | -0.464 |
| Y_vvr' | 0.379 | C1 | 2.0 |
| Y_vrr' | -0.391 | C2 for beta_P > 0 | 1.6 |
| Y_rrr' | 0.008 | C2 for beta_P < 0 | 1.1 |
| N_v' | -0.137 | gamma_R for beta_R < 0 | 0.395 |
| N_R' | -0.049 | gamma_R for beta_R > 0 | 0.640 |
| N_vvv' | -0.030 | l_R' | -0.710 |
| N_vvr' | -0.294 | epsilon | 1.09 |
| N_vrr' | 0.055 | kappa | 0.50 |
| N_rrr' | -0.013 | f_alpha | 2.747 |

The paper also uses propeller open-water thrust coefficients `(k0, k1, k2) = (0.2931, -0.2753, -0.1385)` and assumes the straight-ahead effective wake fraction `wP0 = 0.40` for the L7 model and `0.35` at full scale.

## 4. Experimental maneuver targets

The **experimental** column is the external target. The paper's calculated column is a useful reproduction baseline, not ground truth.

### Turning-circle indices, normalized by Lpp

| Maneuver | Published MMG calculation | Experiment |
|---|---:|---:|
| Advance, delta = +35 deg | 3.31 | 3.25 |
| Tactical diameter, delta = +35 deg | 3.36 | 3.34 |
| Advance, delta = -35 deg | 3.26 | 3.11 |
| Tactical diameter, delta = -35 deg | 3.26 | 3.08 |

### Zig-zag overshoot angles

| Maneuver metric | Published MMG calculation | Experiment |
|---|---:|---:|
| First OSA, +10/+10 | 5.2 deg | 8.2 deg |
| Second OSA, +10/+10 | 15.8 deg | 21.9 deg |
| First OSA, +20/+20 | 10.9 deg | 13.7 deg |
| First OSA, -10/-10 | 7.6 deg | 9.5 deg |
| Second OSA, -10/-10 | 10.2 deg | 15.0 deg |
| First OSA, -20/-20 | 14.5 deg | 15.1 deg |

## 5. Minimum validation outputs to save

For every run, save both time series and derived metrics:

- `t, x, y, psi, u, v, r, delta, nP`
- hull, propeller, and rudder force/moment components separately
- advance, tactical diameter, transfer, steady turning radius, steady yaw rate, and speed loss
- first and second zig-zag overshoot angles and execute times
- exact integration step, solver, interpolation method, initial conditions, water density, and all coefficient versions

Report normalized and dimensional metrics. Include port/starboard separately; the benchmark is asymmetric because the wake and rudder-inflow fits use different coefficients by sign.

## 6. Error reporting

Use at least:

- signed error: `simulation - experiment`
- relative error: `(simulation - experiment) / experiment`
- trajectory error after alignment, normalized by Lpp
- time-history RMSE for heading and yaw rate
- tolerance/sensitivity results versus timestep, mesh resolution, and coefficient perturbations

A strong paper should show two comparisons:

1. **Reproduction:** your implementation versus the paper's published MMG calculation.
2. **Validation:** your implementation versus the experimental column/free-running trajectories.

If reproduction fails, do not interpret the experimental mismatch as a physics-model limitation yet.
