# AERO4River Vehicle C campaign

The campaign imports all three checksum-locked AERO4River validation tracks into the reference-neutral NED/SNAME trajectory format and replays their measured planar generalized wrench through the actual unconstrained Vehicle C `coupled6` plant. Heave, roll, and pitch are initialized at hydrostatic equilibrium, remain dynamically free, and are retained as diagnostics. Only surge, sway, and yaw are compared to measured channels.

The coordinate transform is based on Section II-B, equations 1-5 of the associated publication and verified against the raw trajectories. Numerical differentiation matches the published Jacobian and source yaw rate with RMS residuals of approximately 0.01-0.06, while body-sway or yaw sign reversals produce residuals of approximately 0.25-2.17. Inputs are held from each previous source sample over the next interval; time is never accumulated from floating-point source timestamps.

## Evidence boundary

This is a descriptive execution campaign, not Vehicle C behavioral validation. AERO4River is a 20.8 kg vessel with four aerial azimuth thrusters. Vehicle C is a 420 kg dual-water-azimuth bootstrap design. No AERO4River coefficient is transferred, and no post-hoc error tolerance is assigned. The campaign establishes that the common trajectory pipeline and actual unconstrained coupled6 plant execute the reference wrench safely. Independent Vehicle C command-and-trajectory data remains necessary for behavioral validation.

Run `npm run campaign:aero4river` to verify/import the source files and generate the campaign artifact. Run `npm run test:aero4river` to regenerate it and enforce determinism, unconstrained DOFs, finite out-of-plane response, and evidence labeling.
