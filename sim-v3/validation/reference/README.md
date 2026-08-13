# Reference validation roles

All imported trajectories use the versioned NED/SNAME SI interchange format.

- **MSS** is the planar Otter oracle.
- **Capytaine** supplies potential-flow added mass, radiation damping, wave excitation, and hydrostatic stiffness only. Viscous damping remains separate.
- **Yasukawa & Yoshimura (2015)** supplies two separate target fixtures. `kvlcc2-mmg-reference-reproduction` has status `reference-reproduction-only` and targets the Cal. column. `kvlcc2-experimental-maneuver-indices` targets MARIN's published L7-model Exp. column and supports scalar-index validation only after execution. Both compare advance, tactical diameter, and available first/second zig-zag overshoots using precommitted per-metric percentage tolerances. Port/starboard remain separate; trajectory overlays are qualitative and non-gating. Neither tier validates Vehicle B's USV coefficients.
- **AERO4River** is replayed through unconstrained `coupled6`, with heave/roll/pitch free and retained as diagnostics. Only surge/sway/yaw is compared because those are the measured channels.
- **Gazebo Harmonic** is an implementation cross-check, not the sole physics oracle.

A constrained AERO4River replay is recorded but cannot be marked as primary Vehicle C evidence.
