# HoloOcean Vehicle A determinism and termination status

## Reset determinism: finalized limitation

Same-seed, same-action HoloOcean 2.3.0 runs are not bit-identical. Across the
eight-repeat checks, divergence is effectively negligible in every policy field
except `linear_accel_z`. Its maximum pairwise divergence is roughly 0.1–0.6
m/s² depending on settling and run configuration. This supports statistical
reproducibility only, unlike bcod-sim's float64, bit-identical determinism sweep.

The final isolation audit found a concrete buoyancy mechanism. In
`HolodeckBuoyantAgent.cpp`, lines 24–31, an agent with no authored
`SurfacePoints` generates `NumSurfacePoints` random samples inside its bounding
box during initialization. `SurfaceVessel` supplies a bounding box but no fixed
surface-point list. In `ApplyBuoyancyDragForce()`, lines 64–84, the submerged
fraction is the count of those points below the fixed `SurfaceLevel`, and the
vertical buoyant force is `Volume * Gravity * WaterDensity * ratio`.

Consequently, a fresh packaged-engine process can use a different unseeded
surface-point cloud and produce a different vertical force for the same pose.
The per-tick force calculation contains no raycast, dynamic water-height query,
or wall-clock read: after initialization it is a pure function of pose, fixed
parameters, and the process-specific sampled point cloud. No engine fix or
workaround was attempted in the isolation phase.

### Bounded packaged-runtime fix attempt

The follow-up audit found no seed parameter for buoyancy point generation and
no Python command or scenario property that sets `SurfacePoints`. HoloOcean's
developer documentation says points may be authored explicitly, but that path
is an Unreal agent Blueprint/C++ authoring operation. `SurfacePoints`,
`NumSurfacePoints`, and `BoundingBox` are plain C++ members without reflected
scenario configuration metadata, and the packaged Ocean binary exposes no
setter command. Fixing the source constructor would require rebuilding and
repackaging the Unreal world; it cannot alter the installed binary used for
these measurements. Therefore neither authorized runtime route was available,
the eight-repeat comparison was not rerun, and the quantified limitation above
stands as final for HoloOcean 2.3.0's packaged Ocean world.

## HoloOcean termination mapping

- **Collision:** native `CollisionSensor`, attached to the vehicle. It records
  `OnActorHit` as a one-tick boolean.
- **Grounding versus object collision:** unavailable as separate native outputs.
  `CollisionSensor::OnHit` receives `OtherActor` and `FHitResult` but discards
  both, exposing only a boolean. The wrapper therefore reports generic
  `collision` with `collision_type=unclassified`; it does not guess.
- **Instability:** sensor-derived combined tilt from the IMU gravity direction.
  Tilt above 60 degrees for one continuous simulated second terminates;
  non-finite IMU data terminates immediately. This avoids pose, orientation,
  rotation, dynamics, location, and velocity sensors. Translational acceleration
  can contaminate the estimate, so the one-second hold is part of the mapping.
- **Allocation failure:** requested native thruster force versus the force the
  wrapper actually passes after clipping. Relative residual above 20% for one
  continuous simulated second terminates; non-finite commands terminate
  immediately. HoloOcean exposes no achieved-thruster-force feedback, so this
  checks command-path allocation/saturation rather than physical actuator failure.
- **Precedence:** instability, generic collision, allocation failure.

This phase does not establish separate grounding/object termination parity or
physical achieved-force parity. Those remain explicit HoloOcean API limitations.
No reward implementation or training was run. Throughput and action-interface
groundwork are recorded in their separate Gate D and action-parity artifacts.
