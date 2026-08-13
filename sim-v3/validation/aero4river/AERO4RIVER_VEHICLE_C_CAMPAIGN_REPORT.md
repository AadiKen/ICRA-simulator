# AERO4River Vehicle C campaign report

## Outcome

All three checksum-locked AERO4River validation tracks executed through the actual Vehicle C `coupled6` plant. Heave, roll, and pitch were initialized at hydrostatic equilibrium and remained dynamically free. No DOF was constrained, every output remained finite, and the reference-neutral importer and replay were deterministic.

| Track | Samples | Position RMSE (m) | Yaw RMSE (rad) | Body-velocity RMSE (m/s) | Max roll (rad) | Max pitch (rad) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Validation 1 | 513 | 9.4346 | 1.5324 | 1.1753 | 0.000919 | 0.000206 |
| Validation 2 | 347 | 11.4045 | 1.2134 | 1.0143 | 0.001585 | 0.000543 |
| Validation 3 | 358 | 11.3790 | 1.7510 | 1.1856 | 0.001084 | 0.000498 |

## Interpretation

The campaign does not validate Vehicle C behavioral dynamics. AERO4River is a 20.8 kg platform with four aerial azimuth thrusters; Vehicle C is a 420 kg dual-water-azimuth bootstrap design. The large errors are therefore descriptive evidence of parameterization mismatch, not a failed same-vessel tolerance gate. No post-hoc tolerance is assigned and no AERO4River coefficient is transferred into Vehicle C.

What passed is narrower but useful: the source files and transform are verified, measured planar generalized wrenches flow through the common trajectory protocol into the actual unconstrained coupled6 plant, out-of-plane states are retained, and all three runs are deterministic and numerically stable.

## Coordinate resolution

The associated publication defines the conventional marine kinematics with `eta=[x,y,psi]`, `nu=[u,v,r]`, generalized propulsion wrench, and the standard planar Jacobian. Numerical differentiation of all three raw validation tracks confirms the identity sign mapping. Correct-transform RMS residuals are approximately 0.01-0.06; body-sway or yaw sign reversals increase residuals to approximately 0.25-2.17. Source `x/y` are normalized to N/E labels while retaining the experiment origin.

## Evidence status

Artifact status is `campaign-executed-descriptive-not-vehicle-c-validation`. Independent Vehicle C actuator-command, wrench, and trajectory measurements remain required before behavioral validation. AERO4River supports applied-wrench pipeline and coupled6 composability evidence only; it does not validate dual-water-azimuth propulsion or complete six-axis behavior.
