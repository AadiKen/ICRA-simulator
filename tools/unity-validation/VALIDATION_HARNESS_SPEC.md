# coupled6 vs. unity_asv_sim Matching Validation — Spec

Purpose: pre-physical-testing confidence gate for Vehicle B and Vehicle C, per PI
instruction. Not intended as a paper claim (footnote at most). Goal is the closest
achievable quantitative match between coupled6 and edvartGB/unity_asv_sim's
FossenDynamics model, not just qualitative agreement.

**Current validation configuration:** the Unity result is
MMG-hull-resistance-corrected but added-mass-excluded. These are independent
modeling decisions. Vehicle B's production XH/YH/NH polynomial is ported into
Unity, while Unity Added Mass remains disabled because its Rigidbody integration
cannot solve the anisotropic combined mass matrix implicitly. Do not describe
the MMG correction as resolving the separate added-mass exclusion.

## 1. What Unity's model actually computes (read from source)

File: `Assets/Scripts/Physics/Water/Dynamics/FossenDynamics.cs`

- **Added mass**: diagonal 6x6 matrix only (`XdotU, YdotV, ZdotW, KdotP, MdotQ, NdotR`).
  Constant — not frequency-dependent. No off-diagonal coupling.
- **Coriolis**: only 4 nonzero entries — `C[0,5]=YdotV*v`, `C[1,5]=XdotU*u`,
  `C[5,0]=YdotV*v`, `C[5,1]=YdotV*u`. This is the standard simplified
  surge/sway/yaw horizontal-plane term. Heave/roll/pitch coupling is not modeled.
- **Validation exclusion — added mass**: disabled. The former finite-difference
  reaction-force implementation was numerically unstable, and Unity Rigidbody
  exposes neither an anisotropic translational mass nor the accumulated wrench
  needed for a correct implicit `(M + Ma)^-1 F` solve. A custom six-DOF plant is
  explicitly out of scope for this footnote-level comparison.
- **Vehicle B MMG hull resistance**: enabled for this comparison. The port uses
  the production normalized surge/sway/yaw polynomial from `packages/core/src/mmg.js`
  and the `vehicle-b-usv-bootstrap.json` derivatives, including the 120 N/(m/s)^2
  straight-resistance term. It replaces generic Xu/Yv/Nr only; generic
  heave/roll/pitch damping remains active.
- **Damping**: diagonal only, linear + quadratic per axis
  (`Xu, Xuu, Yv, Yvv, Zw, Zww, Kp, Kpp, Mq, Mqq, Nr, Nrr`). No split between
  potential-radiation and viscous damping — one combined per-axis term.
- All coefficients are plain public floats with generic placeholder defaults
  (e.g. `Xu = 100`) — not derived from any hull geometry, BEM solve, or captive
  test data. This is favorable for matching: we set these directly from our own
  values rather than reverse-engineering them.

File: `Assets/Scripts/Physics/Water/Statics/Buoyancy.cs`
- Computes actual submerged mesh volume each fixed step and applies
  `F = rho * g * V` at the real submerged centroid. Nonlinear, not a linearized
  stiffness — closer to the mesh-based large-angle hydrostatics approach than to
  a linearized restoring coefficient.

File: `Assets/Scripts/Controller/ShipController.cs`
- Thrust = `input * forceMultiplier`, clamped, applied **instantly** at the
  propeller position. No first-order lag, no rate limit, no propeller curve,
  no saturation dynamics beyond the clamp.

## 2. Matching strategy

### 2.1 Terms we can compare directly (both models have them)
- Surge/sway/yaw added mass (diagonal terms only)
- Surge/sway/yaw linear + quadratic damping (diagonal terms only)
- Surge/sway/yaw Coriolis coupling (the 4-entry simplified form)
- Hydrostatic restoring via submerged volume (both are volume/mesh-based —
  worth checking early whether this term already agrees without any tuning)

### 2.2 Terms that require an explicit reconciliation choice
- **coupled6 off-diagonal added-mass / damping coupling**: Unity has none.
  Decision: for matching runs, either (a) zero these terms in coupled6's
  matched-mode config, or (b) restrict test maneuvers to conditions where
  their contribution is small (pure surge, pure yaw, low-heel planar turns).
  Recommend (a) — cleaner attribution of any remaining error.
- **coupled6 roll/pitch/heave dynamics**: Unity's Coriolis/damping do not
  populate these axes meaningfully for a horizontal-plane maneuver. Matching
  runs should be planar (constant depth, near-zero heel) so this gap doesn't
  contaminate the comparison.
- **coupled6 potential radiation damping (frequency-dependent, from Capytaine)
  vs. Unity's flat quadratic damping**: not reconcilable term-for-term. For
  matching runs, sum coupled6's potential + viscous damping into a single
  best-fit linear+quadratic coefficient per axis to compare against Unity's
  combined `Xu/Xuu` etc. Document this as an approximation, not a term match.
- **Vehicle B planar hull and rudder forces**: Unity's generic `Xu/Yv/Nr`
  damping is a placeholder standing in for Vehicle B's real MMG hull-force
  model, which has no Unity equivalent. Expect close matching only on
  propeller-thrust-dominated motion such as straight-line acceleration.
  Turning and zig-zag maneuvers will show real, expected divergence
  attributable to this model gap, not to implementation bugs. Unity also
  applies the replayed rudder as a kinematic joint angle only; it does not
  model the MMG rudder hydrodynamic force, which is a second, smaller known
  gap.
- **WAM-V propulsion geometry substitution**: `WamVPrefab` has four
  asymmetric steerable outboards, not Vehicle B's centerline single-shaft
  propeller-plus-rudder layout. To avoid injecting spurious yaw and sway from
  off-center thrust placement, replay thrust is applied through the vehicle
  CG along the hull's forward axis. Rudder effect is approximated as a simple
  proportional yaw torque at the CG rather than a ported flow-deflection
  rudder model. This known geometric/mechanism substitution further reduces
  expected fidelity on turning maneuvers beyond the missing MMG hull-force
  model described above.
- **Thrust model**: coupled6 has first-order lag, rate limits, saturation.
  Unity has none. Decision: add a first-order lag component to the Unity side
  (small C# change, see `ReplayThruster.cs` below) rather than stripping
  coupled6's actuator model, since lag is physically real and shouldn't be
  removed from the vehicle under test.

### 2.3 What each matching run should report
- "Matched-terms" comparison: coupled6 run in a mode with off-diagonal
  coupling zeroed and out-of-plane DOFs suppressed, vs. Unity as-is.
- "Full coupled6" comparison: coupled6 run with full physics, vs. Unity as-is.
  Expected to diverge more — the gap here is attributable to physics Unity
  doesn't model (radiation damping frequency dependence, off-diagonal coupling,
  out-of-plane coupling), not to bugs. Report this gap explicitly so it isn't
  misread as your model being wrong.

## 3. Test protocol

1. **Parameter sync**: mass, inertia (Ixx/Iyy/Izz — Unity's Rigidbody only
   takes a diagonal inertia tensor via `inertiaTensor`, so off-diagonal inertia
   terms in coupled6 must be dropped or approximated for this comparison), CG
   offset, thruster position(s), water density, gravity — set identically in
   both. Any mismatch here shows up as trajectory error that isn't a physics
   discrepancy at all; eliminate before comparing anything else.
2. **Command replay, not independent control**: record the exact
   time-stamped thrust/rudder command sequence from a coupled6 run and replay
   it into Unity via `ReplayThruster.cs` (below), rather than running the
   "same maneuver" through two separate controllers. Removes control-loop
   timing as a confound.
3. **Maneuver set** (reuse existing MSS/USV-Bench scenario definitions where
   possible): straight-line thrust ramp, turning circle, zig-zag, station-
   keeping (Vehicle C). Keep to the horizontal plane per 2.2.
4. **Fixed timestep**: match Unity's `Time.fixedDeltaTime` to coupled6's
   step size, or run both at a small enough step that numerical integration
   error is negligible relative to the physics gap you're trying to measure.
5. **Logging**: both sims log planar SI data using the same CSV schema:
   `t,N,E,yaw,u,v,r`, where `N`/`E` are NED world-frame positions in metres,
   `yaw` is heading in radians positive from North toward East, `u`/`v` are
   body-frame surge/sway velocities in m/s, and `r` is yaw rate in rad/s.
6. **Comparison**: run `compare_trajectories.py` (below) on the two logs.

## 4. Deliverables in this handoff
- `ReplayThruster.cs` — Unity component that replays a recorded command
  file instead of taking live input, with an optional first-order lag.
- `TrajectoryLogger.cs` — Unity component that logs pose/velocity to CSV
  in the shared schema.
- `coefficient_mapping.json` — template for syncing coupled6 diagonal
  coefficients into Unity's FossenDynamics public fields.
- `compare_trajectories.py` — computes planar position, heading, and velocity
  RMSE and writes a Markdown report, with an optional divergence plot.

## 5. Open decisions for you / PI before running this for real
- Confirm whether "matched-terms" (2.2 option a, zeroed coupling) or
  "full coupled6" is the number that matters for the go/no-go decision —
  recommend reporting both, but the gate criterion should be decided
  explicitly rather than defaulting to whichever number looks better.
- Confirm tolerance band — recommend setting this from the observed
  noise floor of a repeated identical-config run rather than guessing a
  percentage up front.
- Confirm whether Vehicle C's dual-azimuth allocation logic needs its own
  translation layer into Unity's engine/propeller joint system (Unity's
  `ShipController` assumes fixed engine/propeller pairs per hull side, not
  a generic multi-thruster allocator) — this may need custom Unity-side
  code beyond `ReplayThruster.cs` if you want azimuth angle commands
  replayed directly rather than resolved thrust vectors.
