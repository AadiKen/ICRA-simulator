# ICRA 2027 implementation status

## Offline software platform complete

The typed core migration, integer-step scheduling, immutable migration goldens, MSS acceptance, combined damping contracts, Capytaine bootstrap integration, Vehicle B production MMG path, Vehicle C production dual-azimuth path, checkpoint/replay, deterministic vector execution, worker failure recovery, and Node/CPU tensor equivalence are implemented and tested.

USV-Bench-36 executes all 36 scenarios across three vehicles for 108 base configurations plus a fixed confidence subset. Every run carries immutable configuration, state/actuator/metric Parquet, events, replay, checksums, failure metadata, and the vehicle's validation scope. These results establish software execution, not physical accuracy for bootstrap vehicles.

The offline platform also includes:

- integrated Vehicle C singular, near-singular, failed-off, and stuck-thruster evidence;
- Gymnasium single/vector environments with checkpoint and oracle separation;
- conventional PID, MPC, and CPU PPO baselines with no novelty claim;
- immutable offline environmental fixtures and Natural Earth fallback;
- a strict TypeScript React/Three.js research console;
- paper tables/figures, an anonymous-release manifest, and a scripted offline demonstration;
- an explicit `field-data-unavailable` telemetry artifact.

Run `npm test` for all software gates and `npm run test:release` for the UI and anonymous-release gate. The original pre-migration goldens remain the comparison basis; the migration gate reports zero unreviewed deltas.

## External evidence and infrastructure still required

- reviewed Vehicle B/C hull meshes and Capytaine mesh convergence;
- independent Vehicle B USV maneuver and free-decay data;
- independent Vehicle C dual-azimuth command/trajectory data;
- missing KVLCC2 L7 operating-point inputs or restricted SIMMAN trajectories;
- pinned Linux ROS 2 execution;
- NVIDIA CUDA and available Apple MPS runners;
- VRX installation and equivalent-scenario measurements;
- approved live NOAA/USGS execution;
- fresh MSS/Octave regeneration from the pinned source checkout;
- genuine real-USV telemetry.

Vehicle A is validated only for the pinned MSS `planar3` scope. Vehicle B has been scored against six single-screw MARIN model-scale trajectories with measured rudder and samplewise RPM; the current parameterization fails the fixed trajectory/IMO limits, and USV-scale coefficient validation remains blocked. Vehicle C remains allocation/composability-demonstrated with behaviorally unvalidated dynamics. The complete `coupled6` simulator must not be described as physically validated until the external evidence gates pass.
