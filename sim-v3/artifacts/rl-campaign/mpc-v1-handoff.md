# MPC-v1 handoff — parked incomplete

## Confirmed defects

- The prediction model is linearized once at the current state and reuses that
  same `A`, `B`, and affine residual for every horizon step.  It is therefore
  frozen across the horizon rather than LTV/relinearized along a predicted or
  reference trajectory.
- The quadratic delivered-thrust weight is `0.02`; at realistic tens-of-newton
  commands it dominates position and velocity terms by orders of magnitude and
  does not mirror the task's reward scale.
- Heading weight is hard-coded to `0.5` and was never part of the tuning grid.
- The reference is recomputed from the vehicle's current path projection.  It
  slides forward abeam of the vehicle instead of imposing a time/path-anchored
  demand to close cross-track error.

## Confirmed not to be defects

- **Effector feedback:** the Node diagnostics emit `port` and `starboard`
  effectors with a numeric `thrust`; the runner reads those exact identifiers
  back into the delivered-thrust state.
- **Allocation bypass:** `desiredWrench` enters `resolveDesiredWrench()` then
  `allocate()`.  Vehicle A's two fixed thrusters use the direct port/starboard
  allocation path and retain the configured ±70 N actuator bounds.

## Diagnostic never run

The prescribed single-solve inspection was never executed: dump the full moving
reference, optimized control sequence, and predicted 80-step state trajectory;
overlay it with the following 80 simulator steps; and score LOS-PID's realized
trajectory with the same MPC cost.  It remains the fastest diagnostic if MPC is
resumed, but it is not a prerequisite for the portable-task/P3-v3 path.

## Status

`MPC-v1` is **parked-incomplete**.  It is not a calibration reference, a
radius-selection dependency, or a blocker for downstream portable-task work.
