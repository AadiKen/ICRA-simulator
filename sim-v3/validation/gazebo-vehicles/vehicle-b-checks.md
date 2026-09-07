# Vehicle B Gazebo check record

- Target runtime: Gazebo Harmonic / gz-sim 8.15.0 in the repository-pinned image.
- Inertia: uniform-box geometry derivation; all rigid-body triangle inequalities pass.
- `gz sdf --check`: pass.
- Headless world load: pass.
- Neutral-buoyancy rest check: pass; vertical displacement after 31.5 simulated seconds was approximately `3.3e-8 m`.
- Zero-input check: pass at numerical-noise scale.
- Direction checks: forward, astern, positive/negative rudder and positive/negative converted NED yaw-rate signs all agree.
- Fixed-input comparison: fail under the preregistered tolerance. Straight and chirp cases pass; both constant-rudder cases fail yaw and final-position/yaw limits.
- Rudder sweep diagnostic: a single scalar gain does not explain the response curve (yaw-rate fit relative RMSE `0.137`; speed-normalized curvature fit `0.108`, versus the diagnostic `0.05` classification threshold). This points away from correcting only `cla` and toward angle/speed-dependent structural differences.
- Turning-circle diagnostic: the normalized Vehicle-B-USV curves are recorded beside the existing MARIN KVLCC2 measurements. The bcod USV curve is numerically closer, but this is cross-hull resemblance—not a real-world validation result—because the MARIN vessel is a different geometry and parameterization.
- Required gate consequence: Vehicle C was not started.

This is a cross-implementation consistency check, not real-world validation.
Vehicle B remains `model-structure-trajectory-scored-fail`.
